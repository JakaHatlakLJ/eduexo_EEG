#!/usr/bin/env python
# -*- coding: utf-8 -*-


import os
import sys
import time
from time import perf_counter
import numpy as np
from dynamixel_sdk import *  # Dynamixel SDK library
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
DXL_MINIMUM_POSITION_VALUE  = 600
DXL_MAXIMUM_POSITION_VALUE  = 3000
BAUDRATE                    = 57600
PROTOCOL_VERSION            = 2.0
DXL_ID                      = 1
DEVICENAME                  = '/dev/ttyUSB0'

TORQUE_ENABLE               = 1
TORQUE_DISABLE              = 0
DXL_MOVING_STATUS_THRESHOLD = 1

POSITION_CONTROL            = 3   # Value for switching to position control mode

# Unit conversion
pos_unit = 360/4095         # [deg] = [dxl_unit] * [pos_unit]

# DYNAMIXEL INITIALIZATION
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

#Lab Streaming Layer setup

streams = resolve_stream('type', 'EEG') # Resolve a stream

inlet = StreamInlet(streams[0]) # Create an inlet

print("Receiving data...")


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

# Set operating mode to position control
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_OP_MODE, POSITION_CONTROL)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Operating mode has been switched to position control.")

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

# Main loop to move to a desired position
try:
    while True:
        print("Press any key to continue or press ESC to quit:")        
        sample, timestamp = inlet.pull_sample()
        if getch() == 'esc':
            break

        try:
            target_position = int(float(sample[0]) * (float(DXL_MAXIMUM_POSITION_VALUE - DXL_MINIMUM_POSITION_VALUE)) + float(DXL_MINIMUM_POSITION_VALUE))  # convert input to integer

            #if target_position < DXL_MINIMUM_POSITION_VALUE or target_position > DXL_MAXIMUM_POSITION_VALUE:
            #    print(f"Position must be between {DXL_MINIMUM_POSITION_VALUE} and {DXL_MAXIMUM_POSITION_VALUE}")
            #    continue

            # Send the goal position command
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, target_position)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            else:
                print(f"Motor is moving to position {target_position * pos_unit}.")
            
            # Wait until the motor reaches the target position
            while True:
                dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
                else:
                    dxl_present_position_deg = dxl_present_position * pos_unit  # convert to degrees
                    print(f"Current position: {dxl_present_position_deg} degrees")
                    if abs(dxl_present_position - target_position) <= DXL_MOVING_STATUS_THRESHOLD:
                        print(f"Motor has reached the target position {target_position}.")
                        break
                time.sleep(0.1)  # Small delay before checking again

        except ValueError:
            print("Invalid input. Please enter a valid integer position.")

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
