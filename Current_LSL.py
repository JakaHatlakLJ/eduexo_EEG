#!/usr/bin/env python
# -*- coding: utf-8 -*-


import os
import sys
from time import perf_counter
import numpy as np
from dynamixel_sdk import * 
from pylsl import StreamInlet, resolve_stream
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
DXL_MINIMUM_POSITION_VALUE  = 500
DXL_MAXIMUM_POSITION_VALUE  = 3400
BAUDRATE                    = 57600
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
pos_unit = 360/4095        # [deg] = [dxl_unit] * [pos_unit]
vel_unit = 0.229           # [rpm] = [dxl_unit] * [vel_unit]
cur_unit = 2.69            # [mA]  = [dxl_unit] * [cur_unit]

# Current limit
max_torque  = 8                                                          # [Nm] Dodaj implementacijo za spreminjanje preko LSL
max_current = 8.247191-8.247191*np.sqrt(1-0.082598*max_torque)           # [A]
max_current = max_current * 1000                                         # [mA]
max_current = round(max_current / cur_unit)                              # [dxl units]
T_d = 0

# Position offset and limits values
dxl_position_offset = 90   # [deg] Torque/current is 0 at this position
min_pos = 45               # [deg] Torque/current is set to 0 if limit is exceeded
max_pos = 170              # [deg] Torque/current is set to 0 if limit is exceeded

# Initial settings (later replaced by user input values)
K_s = 0.00                   # [Nm/deg] Initial spring coefficient
K_d = 0.00

ard_line = 0

#Lab Streaming Layer setup
#streams = resolve_stream('type', 'EEG')         # Resolve a stream
#inlet = StreamInlet(streams[0])                 # Create an inlet
#print("Receiving data...")

# DYNAMIXEL INITIALIZATION
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

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


# LOCKER INITIALIZATION
position_lock = threading.Lock()
dxl_lock = threading.Lock()

# Function fo reading current position of motor
#def position_read():
#    while 1:
#        # Read present position
#        with dxl_lock:
##            dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
 #           if dxl_comm_result != COMM_SUCCESS:
 #               print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
#            elif dxl_error != 0:
#                print("%s" % packetHandler.getRxPacketError(dxl_error))
#        b1 = dxl_present_position.to_bytes(4, byteorder=sys.byteorder, signed = False) 
#        dxl_present_position = int.from_bytes(b1, byteorder=sys.byteorder, signed = True)
#        with position_lock:
#            global dxl_present_position_deg
#            dxl_present_position_deg = dxl_present_position*pos_unit            #[deg]
#            global  dxl_present_position_rad
#            dxl_present_position_rad = (dxl_present_position_deg*np.pi)/180     # [rad]

def get_motor_position():
    """Get the current position of the Dynamixel motor"""
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Error: {packet_handler.getTxRxResult(dxl_comm_result)}")
        return None
    b1 = dxl_present_position.to_bytes(4, byteorder=sys.byteorder, signed = False) 
    dxl_present_position = int.from_bytes(b1, byteorder=sys.byteorder, signed = True)
    return dxl_present_position

# Set operating mode to current control
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_OP_MODE, CURRENT_CONTROL)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Operating mode has been switched to position control.")

# Set the current limit (used to limit the maximum value of goal current in Current control mode) 
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_CURRENT_LIMIT, max_current)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Current limit has been set.")


t_print = 0.1

#Turn LED ON
#led = LED(27)
#led.on()


while 1:#
    # Start of new loop
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() ==chr(0x1b):
        break
    
    # Get user input for spring and damper coefficients and enable/disable gravity compensation
    K_s = input('Enter spring coefficient value in Nm/deg and press Enter (e.g. 0.1)\n')
    if K_s == "": # if there is no input set a default
        K_s = "0.2"
    print('Spring coefficient = '+K_s+"Nm/deg")    
    K_s = float(K_s)

    K_d = input('Enter damping coefficient value in Nm*s/deg and press Enter (e.g. 0.01)\n')
    if K_d == "": # if there is no input set a default
        K_d = "0.01"
    print('Damping coefficient = '+K_d+"Nm*s/deg")    
    K_d = float(K_d)

    # Clear Bus Watchdog
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_WATCHDOG, watchdog_clear)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Torque is enabled.")
    

    t0 = t1 = t5 = perf_counter() # Used for results printing timing

    dxl_present_position_deg = 0
    dxl_present_position_rad = 0

    # Initialize threads
    #position_thread = threading.Thread(target= get_motor_position, daemon = True)
    #position_thread.start()


    try:   
             
        # Enable Bus Watchdog
        with dxl_lock:
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_WATCHDOG, watchdog_time)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            else:
                print(f"Watchdog is set to {watchdog_time*20} ms.")

        # Used for initial velocity calculation
        with position_lock:
            dxl_present_position_main_deg = get_motor_position() * pos_unit
            dxl_present_position_main_rad = (dxl_present_position_deg*np.pi)/180


        while 1:
            
            # Loop timing
            t4 = perf_counter()-t5
            t5 = perf_counter()
            
            # Save previous position for velocity calculation
            dxl_present_position_old = dxl_present_position_main_deg

            # Read present position     
            with position_lock:
                dxl_present_position_main_deg = get_motor_position() * pos_unit
                dxl_present_position_main_rad = (dxl_present_position_deg*np.pi)/180
            
            # Calculate present velocity
            dxl_present_velocity = (dxl_present_position_main_deg-dxl_present_position_old)/t4    # [deg/s]

            # Calculate goal current
            if (dxl_present_position_main_deg<min_pos or dxl_present_position_main_deg>max_pos):           # Check if position is within position limits 
                dxl_goal_current = 0
                dxl_goal_torque = 0
            else:
                dxl_goal_torque = -K_s*(dxl_present_position_main_deg-dxl_position_offset)-K_d*(dxl_present_velocity) + T_d # [Nm] Torque should point in opposite direction as displacement
                dxl_goal_current = 8.247191-8.247191*np.sqrt(1-0.082598*dxl_goal_torque)     # [A]
                dxl_goal_current = dxl_goal_current * 1000                                   # [mA]
                dxl_goal_current = round(dxl_goal_current / cur_unit)                        # [dxl units]
            
            # Write goal current
            if -max_current<dxl_goal_current<max_current:   # Check if goal current is within current limits
                with dxl_lock:
                    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_CURRENT, dxl_goal_current)
                    if dxl_comm_result != COMM_SUCCESS:
                        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                    elif dxl_error != 0:
                        print("%s" % packetHandler.getRxPacketError(dxl_error))
                dxl_goal_current = dxl_goal_current * cur_unit  # [mA]
            else:
                print("Goal current is too high.")
                break
            t2 = perf_counter() -t1  #Used for results printing timing
            

            if (t2>t_print):
                t1 = perf_counter()
                # Read present current 
                with dxl_lock:   
                    dxl_present_current, dxl_comm_result2, dxl_error2 = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_CURRENT)
                    if dxl_comm_result2 != COMM_SUCCESS:
                        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                    elif dxl_error2 != 0:
                        print("%s" % packetHandler.getRxPacketError(dxl_error))
                b3 = dxl_present_current.to_bytes(2, byteorder=sys.byteorder, signed = False) 
                dxl_present_current = int.from_bytes(b3, byteorder=sys.byteorder, signed = True) # [dxl units]
                dxl_present_current = dxl_present_current * cur_unit # [mA]
                dxl_present_current = dxl_present_current/1000       # [A]
                dxl_present_torque =  2.936*dxl_present_current - 0.178*dxl_present_current**2 #[Nm]

                # Elapsed time from the start of the program
                t3 = -(t0-perf_counter())
                # Print results
                print("SpringCoeff: %.4f Nm/deg  DampingCoeff: %.4f Nm*s/deg  PresPos: %.4f deg  PresVel: %.4f deg/s  GoalTorque: %.4f Nm  PresTorque: %.4f Nm Time: %.4f s Loop Time: %.4f s " % (K_s, K_d, dxl_present_position_main_deg, dxl_present_velocity, dxl_goal_torque, dxl_present_torque, t3, t4))
                print(ard_line)








    except KeyboardInterrupt:
        print("Loop ended.")

    finally:

        # Close port
        portHandler.closePort()

        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        portHandler = PortHandler(DEVICENAME)

        # Open port
        if portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()
        
        # Set port baudrate
        if portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

        # Set goal current to 0
        dxl_goal_current = 0
        with dxl_lock:
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_CURRENT, dxl_goal_current)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            else:
                print("Goal current is set to 0.")
        
        # Disable Dynamixel Torque
        with dxl_lock:
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            else:
                print("Torque is disabled.")
        
        # Turn LED off
        #led.off()


        



























#except KeyboardInterrupt:
 #   print("Program interrupted")
  #  led.off()
#
#finally:            
 #   # Disable Dynamixel Torque before closing the port
  #  dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
   # if dxl_comm_result != COMM_SUCCESS:
    #    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    #elif dxl_error != 0:
    #    print("%s" % packetHandler.getRxPacketError(dxl_error))
    #else:
    #    print("Torque is disabled.")

    # Close port
#    portHandler.closePort()
 #   print("Port closed.")
