#!/usr/bin/env python
# -*- coding: utf-8 -*-

# This script controls an exoskeleton system using Lab Streaming Layer (LSL) for real-time data exchange.  
# It implements impedance control, receiving movement instructions (desired position and torque)  
# and control parameters (stiffness K_s and damping coefficient K_d) via LSL streams.  
# These parameters are used to compute motor commands based on impedance control principles.  
# The script also logs control loop frequencies and ensures safe motor operation.  
# Use with LSL_parameter_sender.py and LSL_outlet.py on PC (in Eduexo_PC).  


import os
import sys
from time import perf_counter
import numpy as np
from dynamixel_sdk import * 
from pylsl import StreamInlet, resolve_byprop, StreamInfo, StreamOutlet
from gpiozero import LED
import threading
from main.EXO_setup import SetupEXO, getch

print_param = False                                                                     # Set to True to printing present parameters each loop

# Path for saving frequencies of each loop
frequency_path = "./frequency_data"
save_freq = False                                                                       # Set to True to save control frequencies of each loop

#### Lab Streaming Layer setup
# Position and Torque
while True:
    stream_xT = resolve_byprop('type', 'Instructions', timeout=3.0)                         # Resolve a stream
    if stream_xT:
        break
    print("No stream found, retrying...")
inlet1 = StreamInlet(stream_xT[0])                                                      # Create an inlet

LSL_parameters = True                                                                  # Set to True if you want to recive parameters through stream
if LSL_parameters:
    while True:
        # Parameters for Impdeance control (K_s, K_d)
        stream_P = resolve_byprop('type', 'params', timeout=3.0)                                         # Resolve a stream
        if stream_P:
            break
        print("No stream found, retrying...")
    inlet2 = StreamInlet(stream_P[0])                                                   # Create an inlet
print("Receiving data...")

# Stream for sending motor data
info = StreamInfo('Stream_EXO', 'EXO', 3, 10000, 'float32', 'test_LSL')

# frequency loging
def create_file(frequency_path):
    os.makedirs(os.path.join(frequency_path), exist_ok=True)
    file_idx = len([filename for filename in os.listdir(os.path.join(frequency_path)) if filename.startswith("frequency_data")])
    file_idx = f'{file_idx:02d}'
    frequency_file = open(os.path.join(frequency_path, f"frequency_data_EXO_{file_idx}.txt"), "w")
    return frequency_file

def LSL_get():
    while not EXO.stop_event.is_set():
        sample_xT, timestamp_xt = inlet1.pull_sample(timeout=1.0)
        if sample_xT is not None:
            b4 = sample_xT[0] * (EXO.max_pos - EXO.min_pos) + EXO.min_pos  # convert input to integer
            b5 = int(sample_xT[1])
            global demanded_position
            global demanded_torque
            demanded_position = b4
            demanded_torque = b5

        if LSL_parameters:
            sample_P, timestamp_p = inlet2.pull_sample(timeout=1.0)
            if sample_P is not None:
                global K_stiffness
                global K_damping
                K_stiffness = sample_P[0]
                K_damping = sample_P[1]            

t_print = 0.1
demanded_position = 90          # Initial value
demanded_torque = 0             # Initial value

while 1:
    # Start of new loop
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() ==chr(0x1b):
        break
    
    if save_freq:
        frequency_file = create_file(frequency_path)
    freqs = []

    outlet = StreamOutlet(info) # Create an LSL outlet

    EXO = SetupEXO()
    EXO.start_system() #Initialize motor
    
    # Get user input for spring and damper coefficients
    K_s = input('Enter spring coefficient value in Nm/deg and press Enter (default 0.07)')
    if K_s == "":   # if there is no input set a default
        K_s = "0.07"
    print('Spring coefficient = '+K_s+"Nm/deg")    
    K_s = float(K_s)
    K_stiffness = K_s

    K_d = input('Enter damping coefficient value in Nm*s/deg and press Enter (default 0.006)')
    if K_d == "":   # if there is no input set a default
        K_d = "0.006"
    print('Damping coefficient = '+K_d+"Nm*s/deg")    
    K_d = float(K_d)
    K_damping = K_d

    t0 = t1 = t5 = perf_counter() # Used for results printing timing

    # Initialize threads
    motor_data_thread = threading.Thread(
        target=EXO.motor_data,
        args=(True, False, False, True),
        daemon=True
        )
    motor_data_thread.start()
    LSL_get_thread = threading.Thread(target = LSL_get, daemon = True)
    LSL_get_thread.start()

    try:
        #enable Torque and set Bus Watchdog
        EXO.torque_watchdog()

        present_velocity_deg = 0 # velocity for initial calculation 

        previous_time = perf_counter() # Loop Timer


        while 1:
            current_time = perf_counter()

            if current_time - previous_time >= 1 / EXO.loop_frequency:
                
                # Read present position     
                with EXO.dxl_lock:
                    present_position_deg = EXO.present_position_deg    
                    # dxl_present_velocity_main_deg = EXO.present_velocity_deg                            # Currently updated at the end of the loop, uncomment this to update at the begining
 
                goal_position = demanded_position
                T_d = demanded_torque
                if LSL_parameters:
                    K_s = round(K_stiffness, 5)
                    K_d = round(K_damping, 5)
                
                # Calculate goal current
                if (present_position_deg < EXO.min_pos or present_position_deg > EXO.max_pos):           # Check if position is within position limits 
                    dxl_goal_current = dxl_goal_torque = 0
                    EXO.write_current(0)
                else:
                    goal_torque = -K_s*(present_position_deg - goal_position) - K_d*(present_velocity_deg) + T_d                        # [Nm] Torque should point in opposite direction as displacement
                    dxl_goal_current = round((8.247191-8.247191*np.sqrt(max(0, 1-0.082598*goal_torque))) * 1000 / EXO.cur_unit)                 # [A] * 1000 --> [mA] / cur_unit --> [dxl unit]
                
                    # Write goal current
                    if -EXO.current_limit > dxl_goal_current:       # Check if goal current is within current limits
                        dxl_goal_current = -EXO.current_limit
                    elif dxl_goal_current > EXO.current_limit:
                        dxl_goal_current = EXO.current_limit
                    EXO.write_current(int(dxl_goal_current))

                t2 = perf_counter() -t1  #Used for results printing timing

                # Update current velocity
                with EXO.dxl_lock:
                    present_velocity_deg = EXO.present_velocity_deg
              
                freq = 1 / (current_time - previous_time)
                freqs.append(freq)

                previous_time = current_time

                if print_param is True:
                    # Loop timing
                    t4 = perf_counter()-t5
                    t5 = perf_counter()
                    print(f'Loop time: {t4}')

                    if (t2>t_print):
                        t1 = perf_counter()
                        # Read present current 
                        with EXO.dxl_lock:   
                            dxl_present_current, dxl_comm_result2, dxl_error2 = EXO.packetHandler.read2ByteTxRx(EXO.portHandler, EXO.DXL_ID, EXO.ADDR_PRESENT_CURRENT)
                            if dxl_comm_result2 != COMM_SUCCESS:
                                print("%s" % EXO.packetHandler.getTxRxResult(dxl_comm_result2))
                            elif dxl_error2 != 0:
                                print("%s" % EXO.packetHandler.getRxPacketError(dxl_error2))
                        b3 = dxl_present_current.to_bytes(2, byteorder=sys.byteorder, signed = False) 
                        dxl_present_current = int.from_bytes(b3, byteorder=sys.byteorder, signed = True) # [dxl units]
                        dxl_present_current = dxl_present_current * EXO.cur_unit # [mA]
                        dxl_present_current = dxl_present_current/1000       # [A]
                        dxl_present_torque =  2.936*dxl_present_current - 0.178*dxl_present_current**2 #[Nm]
                                    
                        # Loop timing
                        t4 = perf_counter()-t5
                        t5 = perf_counter()

                        # Elapsed time from the start of the program
                        t3 = -(t0-perf_counter())
                        # Print results
                        print("SpringCoeff: %.4f Nm/deg  DampingCoeff: %.4f Nm*s/deg  PresPos: %.4f deg  PresVel: %.4f deg/s  GoalTorque: %.4f Nm  PresTorque: %.4f Nm Time: %.4f s Loop Time: %.4f s " % (K_s, K_d, present_position_deg, present_velocity_deg, dxl_goal_torque, dxl_present_torque, t3, t4))

    except KeyboardInterrupt:
        print("Loop ended.")
        if save_freq:
            frequency_file.write("\n".join(str(f) for f in freqs) + "\n")
            freqs = []
            frequency_file.close()

    finally:
        EXO.stop_event.set()
        LSL_get_thread.join()
        motor_data_thread.join()
        EXO.stop_event.clear()
        EXO.stop_system()
        EXO.portHandler.closePort()