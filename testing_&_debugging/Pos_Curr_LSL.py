 #!/usr/bin/env python
# -*- coding: utf-8 -*-


import os
import sys
import time
from time import perf_counter
import numpy as np
from dynamixel_sdk import *  # Dynamixel SDK library
from pylsl import StreamInlet, resolve_byprop

from main.EXO_setup import SetupEXO, getch

def LSL_get():
    sample, timestamp = inlet.pull_sample(timeout=1.0)
    if sample is not None:
        target_position_deg = round(sample[0] * (EXO.max_pos - EXO.min_pos) + EXO.min_pos, 2)
        torque_limit = sample[1]
    else:
        return None, None

    if target_position_deg < EXO.min_pos or target_position_deg > EXO.max_pos:
        print(f"Position must be between {EXO.min_pos} and {EXO.max_pos}, your value {target_position_deg}")
        return None, None
    else:
        return target_position_deg, torque_limit
        
#Lab Streaming Layer setup
streams = resolve_byprop('type', 'Instructions')         # Resolve a stream
inlet = StreamInlet(streams[0])                 # Create an inlet
print("Receiving data...")

DXL_MOVING_STATUS_THRESHOLD = 10
POSITION_D_GAIN             = 80
POSITION_I_GAIN             = 82
POSITION_P_GAIN             = 84

# Position PID parameters and FeedForward parameters:
P_gain = 800        # Default 800
I_gain = 0          # Default 0
D_gain = 0          # Default 0
PID_setup   = [POSITION_P_GAIN, POSITION_I_GAIN, POSITION_D_GAIN]
PID_values  = [P_gain, I_gain, D_gain]
PID_names   = ['P', 'I', 'D']

EXO = SetupEXO(torque_limit=1,CONTROL_MODE=5)
EXO.start_system()
packetHandler = EXO.packetHandler
portHandler = EXO.portHandler

# Set position PID parameters of motor
for i in PID_setup:
    a = PID_setup.index(i)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, EXO.DXL_ID, i, PID_values[a])
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print(f'{PID_names[a]} gain has been set')


# Main loop to move to a desired position
try:
    while True:
        target_position_deg = EXO.mid_pos
        dxl_target_position = round(target_position_deg / EXO.pos_unit)
        print("Press any key to continue or press ESC to quit:")        
        if getch() == chr(0x1b):
            break

        try:
            EXO._enable_disable_torque()
            while True:    

                recieved_position_deg, recieved_torque_limit = LSL_get()

                # Send the goal position command
                if recieved_position_deg is not None:
                    target_position_deg = recieved_position_deg
                    dxl_target_position = round(target_position_deg / EXO.pos_unit)
                    if recieved_torque_limit != 0:
                        EXO._set_torque_limit(recieved_torque_limit)

                EXO.write_position(dxl_target_position, print_result=True)

                # Wait until the motor reaches the target position
                while True:
                    present_position_deg = EXO._read_position()
                    
                    print(f"Current position: {round(present_position_deg, 2)} deg")
                    if abs(present_position_deg / EXO.pos_unit - dxl_target_position) <= DXL_MOVING_STATUS_THRESHOLD:
                        print(f"Motor has reached the target position {target_position_deg} deg")
                        break
                    time.sleep(0.1)  # Small delay before checking again

        except ValueError:
            print("Invalid input. Please enter a valid integer position.")

except KeyboardInterrupt:
    print("Program interrupted")

finally:            
    EXO.stop_system()