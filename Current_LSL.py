#!/usr/bin/env python
# -*- coding: utf-8 -*-


import os
import sys
from time import perf_counter
import numpy as np
from dynamixel_sdk import * 
from pylsl import StreamInlet, resolve_stream
from gpiozero import LED


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
DXL_MINIMUM_POSITION_VALUE  = 500
DXL_MAXIMUM_POSITION_VALUE  = 3400
BAUDRATE                    = 57600
PROTOCOL_VERSION            = 2.0
DXL_ID                      = 1
ADDR_CURRENT_LIMIT          = 38

DEVICENAME                  = '/dev/ttyUSB0'

TORQUE_ENABLE               = 1
TORQUE_DISABLE              = 0
DXL_MOVING_STATUS_THRESHOLD = 10

CURRENT_CONTROL             = 0   # Value for switching to current control mode

# Unit conversion
pos_unit = 360/4095        # [deg] = [dxl_unit] * [pos_unit]
vel_unit = 0.229           # [rpm] = [dxl_unit] * [vel_unit]
cur_unit = 2.69            # [mA]  = [dxl_unit] * [cur_unit]

# Current limit
max_torque  = 1                                                          # [Nm] Dodaj implementacijo za spreminjanje preko LSL
max_current = 8.247191-8.247191*np.sqrt(1-0.082598*max_torque)           # [A]
max_current = max_current * 1000                                         # [mA]
max_current = round(max_current / cur_unit)                              # [dxl units]

#Lab Streaming Layer setup
streams = resolve_stream('type', 'EEG')         # Resolve a stream
inlet = StreamInlet(streams[0])                 # Create an inlet
print("Receiving data...")

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

# Set operating mode to current control
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_OP_MODE, CURRENT_CONTROL)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Operating mode has been switched to position control.")

# Set the current limit (used to limit the maximum value of goal current in Current based Position control mode) 
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_CURRENT_LIMIT, max_current)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Current limit has been set.")

# Enable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Torque is enabled.")

#Turn LED ON
led = LED(27)
led.on()

try:
    while True:
        print("Press any key to continue or press ESC to quit:")        
        if getch() == chr(0x1b):
            break


























except KeyboardInterrupt:
    print("Program interrupted")
    led.off()

finally:            
    # Disable Dynamixel Torque before closing the port
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Torque is disabled.")

    # Close port
    portHandler.closePort()
    print("Port closed.")
