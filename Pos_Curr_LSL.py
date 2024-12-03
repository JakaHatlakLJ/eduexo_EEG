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
DXL_MINIMUM_POSITION_VALUE  = 500
DXL_MAXIMUM_POSITION_VALUE  = 3400
BAUDRATE                    = 57600
PROTOCOL_VERSION            = 2.0
DXL_ID                      = 1
ADDR_CURRENT_LIMIT          = 38
POSITION_D_GAIN             = 80
POSITION_I_GAIN             = 82
POSITION_P_GAIN             = 84

DEVICENAME                  = '/dev/ttyUSB0'

TORQUE_ENABLE               = 1
TORQUE_DISABLE              = 0
DXL_MOVING_STATUS_THRESHOLD = 10

CURRENT_POSITION_CONTROL    = 5   # Value for switching to position control mode

# Unit conversion
pos_unit = 360/4095        # [deg] = [dxl_unit] * [pos_unit]
vel_unit = 0.229           # [rpm] = [dxl_unit] * [vel_unit]
cur_unit = 2.69            # [mA] = [dxl_unit] * [cur_unit]

# Current limit
max_torque  = 1                                                          # [Nm] Dodaj implementacijo za spreminjanje preko LSL
max_current = 8.247191-8.247191*np.sqrt(1-0.082598*max_torque)           # [A]
max_current = max_current * 1000                                         # [mA]
max_current = round(max_current / cur_unit)                              # [dxl units]


# Position PID parameters and FeedForward parameters:
P_gain = 800        # Default 800
I_gain = 0          # Default 0
D_gain = 0          # Default 0
PID_setup   = [POSITION_P_GAIN, POSITION_I_GAIN, POSITION_D_GAIN]
PID_values  = [P_gain, I_gain, D_gain]
PID_names   = ['P', 'I', 'D']

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

# Set position PID parameters of motor
for i in PID_setup:
    a = PID_setup.index(i)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, i, PID_values[a])
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print(f'{PID_names[a]} gain has been set')

# Set operating mode to position control
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_OP_MODE, CURRENT_POSITION_CONTROL)
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

# Main loop to move to a desired position
try:
    while True:
        print("Press any key to continue or press ESC to quit:")        
        if getch() == chr(0x1b):
            break

        try:
            while True:    
                sample, timestamp = inlet.pull_sample()
                target_position = int(float(sample[0]) * (float(DXL_MAXIMUM_POSITION_VALUE - DXL_MINIMUM_POSITION_VALUE)) + float(DXL_MINIMUM_POSITION_VALUE))  # convert input to integer
                target_position_deg = target_position * pos_unit

                if target_position < DXL_MINIMUM_POSITION_VALUE or target_position > DXL_MAXIMUM_POSITION_VALUE:
                    print(f"Position must be between {DXL_MINIMUM_POSITION_VALUE} and {DXL_MAXIMUM_POSITION_VALUE}, your value {target_position}")
                    break

                # Send the goal position command
                dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, target_position)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
                else:
                    print(f"Motor is moving to position {round(target_position_deg, 2)} deg")
                
                # Wait until the motor reaches the target position
                while True:
                    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
                    if dxl_comm_result != COMM_SUCCESS:
                        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                    elif dxl_error != 0:
                        print("%s" % packetHandler.getRxPacketError(dxl_error))
                    else:
                        dxl_present_position_deg = dxl_present_position * pos_unit  # convert to degrees
                        print(f"Current position: {round(dxl_present_position_deg, 2)} deg")
                        if abs(dxl_present_position - target_position) <= DXL_MOVING_STATUS_THRESHOLD:
                            print(f"Motor has reached the target position {round(target_position_deg, 2)} deg")
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
