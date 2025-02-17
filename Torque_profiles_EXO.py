#!/usr/bin/env python
# -*- coding: utf-8 -*-


import os
import sys
from time import perf_counter
import numpy as np
from dynamixel_sdk import * 
from gpiozero import LED
from Torque_profiles import TorqueProfiles
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

# BAUDRATE dictionary
baud_dict = {9600 : 0, 57600 : 1, 115200 : 2, 1000000 : 3, 2000000 : 4, 3000000 : 5, 4000000 : 6, 4500000 : 7}

# DYNAMIXEL SETTINGS
ADDR_TORQUE_ENABLE          = 64
ADDR_OP_MODE                = 11
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132
ADDR_PRESENT_CURRENT        = 126
ADDR_PRESENT_VELOCITY       = 128
ADDR_GOAL_CURRENT           = 102
ADDR_BAUDRATE               = 8
DEFAULT_BAUDRATE            = 57600
PROTOCOL_VERSION            = 2.0
DXL_ID                      = 1
ADDR_CURRENT_LIMIT          = 38
ADDR_WATCHDOG               = 98

DEVICENAME                  = '/dev/ttyUSB0'

TORQUE_ENABLE               = 1
TORQUE_DISABLE              = 0
DXL_MOVING_STATUS_THRESHOLD = 10

#### Define control mode and BAUDRATE
CURRENT_CONTROL             = 0                                                         # Value for switching to current control mode
BAUDRATE                    = 1000000               # [bps]                             # set BAUDRATE: 9600/57600/115200/1M/2M/3M/4M/4.5M
BAUDRATE_VALUE              = baud_dict[BAUDRATE]

# Bus Watchdog value
watchdog_time = 5           # [1 = 20 ms]
watchdog_clear = 0

# Unit conversion
pos_unit = 0.088            # [deg] = [dxl_unit] * [pos_unit]
vel_unit = 0.229            # [rpm] = [dxl_unit] * [vel_unit]
cur_unit = 2.69             # [mA]  = [dxl_unit] * [cur_unit]

# Current limit
torque_limit  = 8.0                                                         # [Nm]
current_limit = 8.247191-8.247191*np.sqrt(1-0.082598*torque_limit)          # [A]
current_limit = current_limit * 1000.0                                      # [mA]
current_limit = round(current_limit / cur_unit)                             # [dxl units]
T_d = 0.0

# Max torque profile value
max_torque  = 4                                                           # [Nm]
max_current = 8.247191-8.247191*np.sqrt(1-0.082598*max_torque)              # [A]
max_current = max_current * 1000.0                                          # [mA]
max_current = round(max_current / cur_unit)                                 # [dxl units]

# Position offset and limits values
dxl_goal_position = 90.0     # [deg] Torque/current is 0 at this position
min_pos = 55.0               # [deg] Torque/current is set to 0 if limit is exceeded
max_pos = 180.0              # [deg] Torque/current is set to 0 if limit is exceeded
DXL_MINIMUM_POSITION_VALUE  = round(min_pos / pos_unit)
DXL_MAXIMUM_POSITION_VALUE  = round(max_pos / pos_unit)
recieved_position = 90
recieved_torque = 0




t_prof = TorqueProfiles()
y_trap = t_prof.trapezoid()
y_tri = t_prof.triangle()
y_sin = t_prof.sinus()
y_pulse = t_prof.pulse()
y_smtrap = t_prof.smoothed_trapezoid()
x_list = t_prof.x
freq = t_prof.loop_frequency

active = False


#setup LED
led = LED(27)

# LOCKER INITIALIZATION
torque_lock = threading.Lock()
position_lock = threading.Lock()
dxl_lock = threading.Lock()
stop_event = threading.Event()                                                          # Flag for signaling threads to stop

# Initialize Dynamixel and ports
global portHandler, packetHandler
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Function to find current baudrate
def find_current_baudrate():
    print(f"Trying to find current baudrate")
    for baudrate in baud_dict.keys():
        if portHandler.setBaudRate(baudrate):
            # Try to ping the motor
            dxl_model_number, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, DXL_ID)
            if dxl_comm_result == COMM_SUCCESS:
                print(f"Successfully connected at baudrate: {baudrate}")
                return baudrate
        else:
            print(f"Failed to set baudrate: {baudrate}")
    return None

# function for setting BAUDRATE of Dynamixel
def set_baudrate(NEW_BAUDRATE_VALUE, NEW_BAUDRATE, CURRENT_BAUDRATE = None, start_system = True):
    if CURRENT_BAUDRATE == None:
        CURRENT_BAUDRATE = find_current_baudrate()

    # Set NEW BAUDRATE on Dynamixel
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_BAUDRATE, NEW_BAUDRATE_VALUE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print(f"Baudrate has been switched to {NEW_BAUDRATE} on Dynamixel.")

    if start_system:
        # Set BAUDRATE to NEW BAUDRATE on RasPi
        if portHandler.setBaudRate(NEW_BAUDRATE):
            print(f"Succeeded to change the baudrate to {NEW_BAUDRATE} on RasPi")
        else:
            print("Failed to change the baudrate")
            sys.exit()

        return CURRENT_BAUDRATE

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

    # Set NEW BAUDRATE
    set_baudrate(BAUDRATE_VALUE, BAUDRATE)

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
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_CURRENT_LIMIT, current_limit)
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
        
def stop_system(CURRENT_BAUDRATE):
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

    # reset BAUDRATE back to default value
    set_baudrate(1, DEFAULT_BAUDRATE, CURRENT_BAUDRATE, start_system=False)

def write_current(goal_cur):
    '''Functiong for writing goal current to motor'''
    with dxl_lock:
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_CURRENT, goal_cur)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

g_direction = 1
def position_read():
    global g_direction  # Ensure we modify the global variable
    while not stop_event.is_set():
        try:
            # Read present position
            with dxl_lock:
                dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(
                    portHandler, DXL_ID, ADDR_PRESENT_POSITION
                )
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                    continue
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
                    continue
            b1 = dxl_present_position.to_bytes(4, byteorder=sys.byteorder, signed=False)
            dxl_present_position = int.from_bytes(b1, byteorder=sys.byteorder, signed=True)

            if dxl_present_position >= (max_pos - 20) / pos_unit and g_direction == 1:
                with position_lock:
                    g_direction = -1
            elif dxl_present_position <= (min_pos + 20) / pos_unit and g_direction == -1:
                with position_lock:
                    g_direction = 1
        except Exception as e:
            print(f'Error in position_read: {e}')
            break


while 1:
    # Start of new loop
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() ==chr(0x1b):
        break
    else:
        stop_event.clear()
        CURRENT_BAUDRATE = start_system() #Initialize motor

        position_thread = threading.Thread(target= position_read, daemon = True)
        position_thread.start()
        
        direction = 1

    try:
        #enable Torque and set Bus Watchdog
        with dxl_lock:
            torque_watchdog()

        while 1:
            
            with position_lock:
                direction = g_direction

            if not active:
                print("Press 1 for Trapezoid, 2 for Triangle, 3 for Sinus, 4 for Pulse, 5 for Smoothed Trapezoid")
                key = getch()
                if key == chr(0x31):                    # 1
                    y_list = y_trap
                elif key == chr(0x32):                  # 2
                    y_list = y_tri
                elif key == chr(0x33):                  # 3
                    y_list = y_sin
                elif key == chr(0x34):                  # 4
                    y_list = y_pulse
                elif key == chr(0x35):                  # 5
                    y_list = y_smtrap
                else:
                    print("Invalid input")
                    break

                i = 0
                previous_time = perf_counter() # Loop Timer
                active = True                

            current_time = perf_counter()

            if current_time - previous_time >= 1/freq:
                if i >= len(x_list):
                    active = False
                    i = 0
                    continue
                if direction == 1:
                    dxl_goal_current = int(round(0.85 * direction * max_current * y_list[i]))
                else:
                    dxl_goal_current = int(round(1.15 * direction * max_current * y_list[i]))
                write_current(dxl_goal_current)
                i += 1
                previous_time = current_time
    	
    except KeyboardInterrupt:
        print("Loop ended.")

    except Exception as e:
        print(e)

    finally:    
        stop_event.set()
        position_thread.join()
        with dxl_lock:
            stop_system(CURRENT_BAUDRATE)

        # Close port
        with dxl_lock:
            portHandler.clearPort()
