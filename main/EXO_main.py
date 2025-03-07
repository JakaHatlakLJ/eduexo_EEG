#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
from time import perf_counter
import numpy as np
from dynamixel_sdk import * 
import threading
import json

from EXO_setup import SetupEXO, getch
from Torque_profiles import TorqueProfiles
from EXO_LSL import LSLResolver

def initialize_EXO(EXO_config, setup_dict=None):
    if setup_dict is None:
        setup_dict = {}
        
    # Set up configuration parameters
    setup_dict["torque_limit"] = EXO_config["torque_limit"]
    setup_dict["max_torque"] = EXO_config["max_torque_during_trial"]
    setup_dict["min_pos"] = EXO_config["min_pos"]
    setup_dict["max_pos"] = EXO_config["max_pos"]   
    setup_dict["control_mode"] = EXO_config["DXL_control_mode"]
    setup_dict["baudrate"] = EXO_config["baudrate"]
    setup_dict["loop_frequency"] = EXO_config["control_frequency"]
    setup_dict["trial_time"] = EXO_config["duration_of_trials"]
    setup_dict["position_control"] = EXO_config["incorect_execution_positon_control"]
    setup_dict["tprofile_time"] = EXO_config["incorrect_execution_time_ms"]
    setup_dict["device_name"] = EXO_config["port_name"]
    setup_dict["frequency_path"] = EXO_config["frequency_path"]
    setup_dict["save_data"] = True if EXO_config["save_data"] == 1 else False

    # Unit conversion
    pos_unit = 0.088            # [deg] = [dxl_unit] * [pos_unit]
    vel_unit = 0.229            # [rpm] = [dxl_unit] * [vel_unit]
    cur_unit = 2.69             # [mA]  = [dxl_unit] * [cur_unit]
    max_current = 8.247191 - 8.247191 * np.sqrt(1 - 0.082598 * setup_dict["max_torque"])  # [A]
    max_current = max_current * 1000.0  # [mA]
    setup_dict["max_current"] = round(max_current / cur_unit)  # [dxl units]

    # Initialize torque profiles
    profiles_position = TorqueProfiles(loop_frequency=setup_dict["loop_frequency"], pulse_portion=1)
    profiles_position_dict = {0: profiles_position.y_trap, 1: profiles_position.y_tri, 2: profiles_position.y_sin, 3: profiles_position.y_pulse, 4: profiles_position.y_smtrap}
    profiles_time = TorqueProfiles(loop_frequency=setup_dict["loop_frequency"], time_interval=setup_dict["tprofile_time"], pulse_portion=1)
    profiles_time_dict = {0: profiles_time.y_trap, 1: profiles_time.y_tri, 2: profiles_time.y_sin, 3: profiles_time.y_pulse, 4: profiles_time.y_smtrap}

    # Create EXO setup list
    EXO_setup_list = [
        setup_dict["torque_limit"],
        setup_dict["min_pos"],
        setup_dict["max_pos"],
        setup_dict["control_mode"],
        setup_dict["baudrate"],
        setup_dict["loop_frequency"],
        setup_dict["device_name"]
    ]

    return setup_dict, profiles_position, profiles_position_dict, profiles_time, profiles_time_dict, EXO_setup_list

# Function to create a file for frequency logging
def create_file(frequency_path):
    os.makedirs(os.path.join(frequency_path), exist_ok=True)
    file_idx = len([filename for filename in os.listdir(os.path.join(frequency_path)) if filename.startswith("frequency_data")])
    file_idx = f'{file_idx:02d}'
    frequency_file = open(os.path.join(frequency_path, f"frequency_data_EXO_{file_idx}.txt"), "w")
    return frequency_file

if __name__ == "__main__":

    # Start of new loop
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):
        sys.exit()
    else:        
        exo_config = json.load(open(r"main/EXO_configuration.json", "r"))
        setup_dict, profiles_position, profiles_position_dict, profiles_time, profiles_time_dict, EXO_setup = initialize_EXO(exo_config)

        if setup_dict["save_data"]:
            frequency_file = create_file(setup_dict["frequency_path"])
        freqs = []

        EXO = SetupEXO(*EXO_setup)
        LSL = LSLResolver(setup_dict["loop_frequency"], EXO.stop_event)
        CURRENT_BAUDRATE = EXO.start_system()  # Initialize motor

        t0 = t1 = t5 = perf_counter()  # Used for results printing timing
        execute = False

        # Initialize threads
        motor_data_thread = threading.Thread(
            target=EXO.motor_data,
            daemon=True
        )
        motor_data_thread.start()

        LSL_inlet_thread = threading.Thread(
            target=LSL.LSL_inlet,
            daemon=True
        )
        LSL_inlet_thread.start()

        LSL.timestamp = 0
        previous_timestamp = 0
        LSL.torque_profile = 0
        LSL.direction = 0
        LSL.correctness = 0
        i = 0
        dxl_goal_current = 0
        travel = 0

        try:
            # Enable Torque and set Bus Watchdog
            EXO.torque_watchdog()

            previous_time = perf_counter()  # Loop Timer
            previous_execute = False

            while 1:
                current_time = perf_counter()

                if current_time - previous_time >= 1 / setup_dict["loop_frequency"]:                
                    # Read present position     
                    with EXO.dxl_lock:
                        present_position_deg = EXO.present_position_deg    
                        present_torque = EXO.present_torque                              

                    if LSL.timestamp is not None:
                        timestamp = LSL.timestamp

                    if timestamp != previous_timestamp:
                        y_list_position = profiles_position_dict[LSL.torque_profile]
                        y_list_time = profiles_time_dict[LSL.torque_profile]
                        direction = LSL.direction
                        correctness = LSL.correctness

                        i = 0
                        execute = True
                        start_time = perf_counter()
                        print(f"Torque Profile: {LSL.torque_profile}, Correctness: {correctness}, Direction: {direction}")
                        
                    if present_position_deg < EXO.min_pos + 15 or EXO.max_pos - 15 < present_position_deg:
                        execute = False 
                        EXO.execution = 0
                        dxl_goal_current = 0
                        EXO.write_current(dxl_goal_current)
                    else:
                        if execute:  
                            if current_time - start_time >= setup_dict["trial_time"]:
                                execute = False 
                                EXO.execution = 0
                                dxl_goal_current = 0
                            else:
                                if EXO.execution != 1:
                                    EXO.execution = 1                                         
                                if correctness == 1:
                                    if direction == 20:
                                        travel = int(round((present_position_deg - (EXO.mid_pos - 2)) / (EXO.max_pos - EXO.mid_pos - 15) * profiles_position.instances))
                                        travel = min(travel, len(y_list_position) - 1)
                                        if travel <= 0:
                                            travel = 1
                                        dxl_goal_current = int(round(0.7 * setup_dict["max_current"] * y_list_position[travel]))
                                    else:
                                        travel = int(round((present_position_deg - (EXO.mid_pos + 2)) / (EXO.min_pos + 15 - (EXO.mid_pos + 2)) * profiles_position.instances))
                                        travel = min(travel, len(y_list_position) - 1)
                                        if travel <= 7:
                                            travel = 7
                                        dxl_goal_current = int(round(-setup_dict["max_current"] * y_list_position[travel]))
                                else:
                                    if setup_dict["position_control"]:    
                                        if direction == 20:
                                            travel = int(round((present_position_deg - (EXO.mid_pos - 2)) / (EXO.max_pos - EXO.mid_pos - 15) * profiles_position.instances))
                                            travel = min(travel, len(y_list_position) - 1)
                                            if travel <= 0:
                                                travel = 1
                                            dxl_goal_current = int(round(0.25 * setup_dict["max_current"] * y_list_position[travel]))
                                        else:
                                            travel = int(round((present_position_deg - (EXO.mid_pos + 2)) / (EXO.min_pos + 15 - (EXO.mid_pos + 2)) * profiles_position.instances))
                                            travel = min(travel, len(y_list_position) - 1)
                                            if travel <= 7:
                                                travel = 7
                                            dxl_goal_current = int(round(-0.2 * setup_dict["max_current"] * y_list_position[travel]))                                    
                                    else:
                                        if i >= len(y_list_time):
                                            execute = False
                                            i = 0
                                            continue
                                        if LSL.direction == 20:
                                            dxl_goal_current = int(round(0.75 * setup_dict["max_current"] * y_list_time[i]))
                                        else:
                                            dxl_goal_current = int(round(-1.15 * setup_dict["max_current"] * y_list_time[i]))
                                        i += 1

                            EXO.write_current(dxl_goal_current)
                            
                    freq = 1 / (current_time - previous_time)
                    freqs.append(freq)
                    previous_timestamp = timestamp
                    previous_time = current_time
                    LSL.LSL_outlet(EXO)

        except KeyboardInterrupt:
            print("Loop ended.")
            if setup_dict["save_data"]:
                frequency_file.write("\n".join(str(f) for f in freqs) + "\n")
                freqs = []
                frequency_file.close()

        finally:
            EXO.stop_event.set()
            LSL_inlet_thread.join()
            motor_data_thread.join()
            EXO.stop_event.clear()
            EXO.stop_system()