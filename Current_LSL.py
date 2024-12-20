#!/usr/bin/env python
# -*- coding: utf-8 -*-


import os
import sys
from time import perf_counter
import numpy as np
from dynamixel_sdk import * 
from pylsl import StreamInlet, resolve_stream, StreamInfo, StreamOutlet
from gpiozero import LED
import threading


if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


# DYNAMIXEL SETTINGS
ADDR_TORQUE_ENABLE          = 64
ADDR_OP_MODE                = 11
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132
ADDR_PRESENT_CURRENT        = 126
ADDR_PRESENT_VELOCITY       = 128
ADDR_GOAL_CURRENT           = 102
BAUDRATE                    = 2000000
PROTOCOL_VERSION            = 2.0
DXL_ID                      = 1
ADDR_CURRENT_LIMIT          = 38
ADDR_WATCHDOG               = 98

DEVICENAME                  = '/dev/ttyUSB0'

TORQUE_ENABLE               = 1
TORQUE_DISABLE              = 0
DXL_MOVING_STATUS_THRESHOLD = 10

CURRENT_CONTROL             = 0   # Value for switching to current control mode

# Bus Watchdog value
watchdog_time = 5          # [1 = 20 ms]
watchdog_clear = 0

# Unit conversion
pos_unit = 0.088        # [deg] = [dxl_unit] * [pos_unit]
vel_unit = 0.229           # [rpm] = [dxl_unit] * [vel_unit]
cur_unit = 2.69            # [mA]  = [dxl_unit] * [cur_unit]

# Current limit
max_torque  = 8.0                                                          # [Nm] Dodaj implementacijo za spreminjanje preko LSL
max_current = 8.247191-8.247191*np.sqrt(1-0.082598*max_torque)           # [A]
max_current = max_current * 1000.0                                         # [mA]
max_current = round(max_current / cur_unit)                              # [dxl units]
T_d = 0.0

# Position offset and limits values
dxl_goal_position = 90.0     # [deg] Torque/current is 0 at this position
min_pos = 55.0               # [deg] Torque/current is set to 0 if limit is exceeded
max_pos = 180.0              # [deg] Torque/current is set to 0 if limit is exceeded
DXL_MINIMUM_POSITION_VALUE  = round(min_pos / pos_unit)
DXL_MAXIMUM_POSITION_VALUE  = round(max_pos / pos_unit)
recieved_position = 90
recieved_torque = 0

# Initial settings (later replaced by user input values)
K_s = 0.07                   # [Nm/deg] Initial spring coefficient
K_d = 0.006

print_param = False # Set to True if printing present parameters is desired

#setup LED
led = LED(27)

# LOCKER INITIALIZATION
position_lock = threading.Lock()
velocity_lock = threading.Lock()
torque_lock = threading.Lock()
dxl_lock = threading.Lock()
stop_event = threading.Event() #Flag for signaling threads to stop

#Lab Streaming Layer setup
# Position and Torque
stream_xT = resolve_stream('type', 'EEG')         # Resolve a stream
inlet1 = StreamInlet(stream_xT[0])                 # Create an inlet
# Parameters for Impdeance control (K_s, K_d)
# stream_P = resolve_stream('type', 'EEG_KD')         # Resolve a stream
# inlet2 = StreamInlet(stream_P[0])                 # Create an inlet
print("Receiving data...")

# Stream for sending motor data
info = StreamInfo('Stream_EXO', 'EXO', 3, 10000, 'float32', 'test_LSL')


# Initialize Dynamixel and ports
global portHandler, packetHandler
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

#Function to initalize Dynamixel Motor
def start_system():
    '''Function for seting up motor:
        - Open port
        - Set Baudrate
        - Set operating mode to current control
        - Set current limit
        - Clear Bus Watchdog'''

    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        sys.exit()

    # Set baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        sys.exit()

    with dxl_lock:
        # Set operating mode to current control
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_OP_MODE, CURRENT_CONTROL)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Operating mode has been switched to current control.")

        # Set the current limit
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_CURRENT_LIMIT, max_current)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Current limit has been set.")

        # Clear Bus Watchdog
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_WATCHDOG, watchdog_clear)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print(f"Watchdog is cleared.")

def torque_watchdog():
    '''Function for enabling Torque and setting Bus Watchdog'''       
    # Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Torque is enabled.")
        led.on() #Turn LED ON

    # Enable Bus Watchdog
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_WATCHDOG, watchdog_time)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print(f"Watchdog is set to {watchdog_time*20} ms.")
        
def stop_system():
    '''Function for stoping motor safely:
        - Disable Bus Watchdog
        - Set goal Current to 0
        - Disable torque'''    
    # Clear Bus Watchdog
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_WATCHDOG, watchdog_clear)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print(f"Watchdog is disabled.")

    # Set goal current to 0
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_CURRENT, 0)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Goal current is set to 0.")

    # Disable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Torque is disabled.")
        led.off()# Turn LED off

def write_current(goal_cur):
    '''Functiong for writing goal current to motor'''
    with dxl_lock:
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_CURRENT, goal_cur)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

def LSL_get():
    while not stop_event.is_set():
        sample_xT, timestamp = inlet1.pull_sample(timeout=1.0)

        if sample_xT is not None:
            b4 = int(float(sample_xT[0]) * (float(DXL_MAXIMUM_POSITION_VALUE - DXL_MINIMUM_POSITION_VALUE)) + float(DXL_MINIMUM_POSITION_VALUE))  # convert input to integer
            b5 = int(sample_xT[1])
            global recieved_position
            global recieved_torque
            recieved_position = b4 * pos_unit
            recieved_torque = b5

def motor_data():
    '''Function for reading current position, velocity and Torque of motor'''

    while not stop_event.is_set():
        try:

            # Read present Position
            with dxl_lock:
                dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
            b1 = dxl_present_position.to_bytes(4, byteorder=sys.byteorder, signed = False) 
            dxl_present_position = int.from_bytes(b1, byteorder=sys.byteorder, signed = True)
            with position_lock:
                global dxl_present_position_deg
                dxl_present_position_deg = float(dxl_present_position*pos_unit)            #[deg]
                #global  dxl_present_position_rad
                #dxl_present_position_rad = (dxl_present_position_deg*np.pi)/180     # [rad]
            pass    

            # Read present Velocity
            with dxl_lock:
                dxl_present_velocity, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_VELOCITY)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
            b2 = dxl_present_velocity.to_bytes(4, byteorder=sys.byteorder, signed = False) 
            dxl_present_velocity = int.from_bytes(b2, byteorder=sys.byteorder, signed = True)
            with velocity_lock:
                global  dxl_present_velocity_deg
                dxl_present_velocity_deg = float(dxl_present_velocity) * vel_unit * 6.0     # [deg/s]
                # global dxl_present_velocity_rad
                # dxl_present_velocity_rad = dxl_present_velocity * vel_unit * np.pi/30            #[rad/s]
            pass

            # Read present Current
            with dxl_lock:
                dxl_present_current, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_CURRENT)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
            b3 = dxl_present_current.to_bytes(2, byteorder=sys.byteorder, signed = False) 
            dxl_present_current = int.from_bytes(b3, byteorder=sys.byteorder, signed = True)
            with torque_lock:
                global  dxl_present_torque
                dxl_present_torque = 0.082598 * (1 - (1 - dxl_present_current * cur_unit / 8.247191)**2)    # [mNm]
            pass

            DATA = [dxl_present_position_deg, dxl_present_velocity_deg, dxl_present_torque]
            # Send motor position data via stream
            outlet.push_sample(DATA)

        except Exception as e:
            print(f'Error in velocity_read: {e}')
            break

t_print = 0.1

while 1:
    # Start of new loop
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() ==chr(0x1b):
        break
    
    outlet = StreamOutlet(info) # Create an LSL outlet
    start_system() #Initialize motor
    
    # # Get user input for spring and damper coefficients and enable/disable gravity compensation
    # K_s = input('Enter spring coefficient value in Nm/deg and press Enter (default 0.07)\n')
    # if K_s == "": # if there is no input set a default
    #     K_s = "0.07"
    # print('Spring coefficient = '+K_s+"Nm/deg")    
    # K_s = float(K_s)

    # K_d = input('Enter damping coefficient value in Nm*s/deg and press Enter (default 0.006)\n')
    # if K_d == "": # if there is no input set a default
    #     K_d = "0.006"
    # print('Damping coefficient = '+K_d+"Nm*s/deg")    
    # K_d = float(K_d)

    t0 = t1 = t5 = perf_counter() # Used for results printing timing

    # Initialize threads 
    motor_data_thread = threading.Thread(target = motor_data, daemon = True)
    motor_data_thread.start()
    LSL_get_thread = threading.Thread(target = LSL_get, daemon = True)
    LSL_get_thread.start()

    try:
        #enable Torque and set Bus Watchdog
        with dxl_lock:
            torque_watchdog()

        dxl_present_velocity_main_deg = 0 #velocity for initial calculation             
        target_position = 0

        a = 0 # Loop counter
        b = perf_counter() # Loop Timer

        while 1:
            
            # Read present position     
            with position_lock:
                dxl_present_position_main_deg = dxl_present_position_deg
            # with velocity_lock:
            #     dxl_present_velocity_main_deg = dxl_present_velocity_deg 

            target_position = recieved_position
            if target_position is not None:
                dxl_goal_position = target_position

            target_torque = recieved_torque
            if target_torque is not None:
                T_d = target_torque
            
            # Calculate goal current
            if (dxl_present_position_deg<min_pos or dxl_present_position_deg>max_pos):           # Check if position is within position limits 
                dxl_goal_current = dxl_goal_torque = 0
                write_current(0)
            else:
                dxl_goal_torque = -K_s*(dxl_present_position_main_deg-dxl_goal_position)-K_d*(dxl_present_velocity_main_deg) + T_d # [Nm] Torque should point in opposite direction as displacement
                dxl_goal_current = round((8.247191-8.247191*np.sqrt(1-0.082598*dxl_goal_torque)) * 1000 / cur_unit)     # [A] * 1000 --> [mA] / cur_unit --> [dxl unit]
            
                # Write goal current
                if -max_current<dxl_goal_current<max_current:   # Check if goal current is within current limits
                    write_current(dxl_goal_current)
                    # dxl_goal_current = dxl_goal_current * cur_unit  # [mA]
                else:
                    print("Goal current is too high.")
                    break

            t2 = perf_counter() -t1  #Used for results printing timing

            with velocity_lock:
                dxl_present_velocity_main_deg = dxl_present_velocity_deg
        
            a += 1

            if print_param is True:
                # Loop timing
                t4 = perf_counter()-t5
                t5 = perf_counter()
                print(f'Loop time: {t4}')

                if (t2>t_print):
                    t1 = perf_counter()
                    # Read present current 
                    with dxl_lock:   
                        dxl_present_current, dxl_comm_result2, dxl_error2 = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_CURRENT)
                        if dxl_comm_result2 != COMM_SUCCESS:
                            print("%s" % packetHandler.getTxRxResult(dxl_comm_result2))
                        elif dxl_error2 != 0:
                            print("%s" % packetHandler.getRxPacketError(dxl_error2))
                    b3 = dxl_present_current.to_bytes(2, byteorder=sys.byteorder, signed = False) 
                    dxl_present_current = int.from_bytes(b3, byteorder=sys.byteorder, signed = True) # [dxl units]
                    dxl_present_current = dxl_present_current * cur_unit # [mA]
                    dxl_present_current = dxl_present_current/1000       # [A]
                    dxl_present_torque =  2.936*dxl_present_current - 0.178*dxl_present_current**2 #[Nm]
                                
                    # Loop timing
                    t4 = perf_counter()-t5
                    t5 = perf_counter()

                    # Elapsed time from the start of the program
                    t3 = -(t0-perf_counter())
                    # Print results
                    print("SpringCoeff: %.4f Nm/deg  DampingCoeff: %.4f Nm*s/deg  PresPos: %.4f deg  PresVel: %.4f deg/s  GoalTorque: %.4f Nm  PresTorque: %.4f Nm Time: %.4f s Loop Time: %.4f s " % (K_s, K_d, dxl_present_position_main_deg, dxl_present_velocity_main_deg, dxl_goal_torque, dxl_present_torque, t3, t4))

    except KeyboardInterrupt:
        print("Loop ended.")        
        print(f'Average loop frequency was: {round(a/(perf_counter() - b), 2)}Hz')

    finally:
        stop_event.set()
        LSL_get_thread.join()
        del outlet
        motor_data_thread.join()
        stop_event.clear()
        
        with dxl_lock:
            stop_system()

        # Reboot and Close port
        with dxl_lock:
            portHandler.clearPort()
            #packetHandler.factoryReset(portHandler, DXL_ID, CURRENT_CONTROL)
            portHandler.closePort()































                    


