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

def initialize_EXO(EXO_config, LSL, setup_dict=None):
    if setup_dict is None:
        setup_dict = {}
        
    # Set up instructions from EXO
    setup_dict["torque_limit"] = LSL.torque_limit
    setup_dict["min_pos"] = LSL.min_p
    setup_dict["max_pos"] = LSL.max_p
    setup_dict["center_offset"] = LSL.center_offset
    setup_dict["edge_offset"] = LSL.edge_offset
    setup_dict["time_control"] = LSL.incorect_execution_time_control
    setup_dict["tprofile_time"] = LSL.incorrect_execution_time_ms

    # Set up configuration parameters
    setup_dict["control_mode"] = EXO_config["DXL_control_mode"]
    setup_dict["baudrate"] = EXO_config["baudrate"]
    setup_dict["loop_frequency"] = EXO_config["control_frequency"]
    setup_dict["device_name"] = EXO_config["port_name"]
    setup_dict["frequency_path"] = EXO_config["frequency_path"]
    setup_dict["save_data"] = True if EXO_config["save_data"] == 1 else False

    setup_dict["current_limit"] = torque_to_current(LSL.torque_limit)        # [dxl units]

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

def torque_to_current(torque):
    current = 8.247191 - 8.247191 * np.sqrt(1 - 0.082598 * torque)      # [A]
    current = current * 1000                                            # [mA]
    current = round(current / SetupEXO.cur_unit)                                 # [dxl_units]
    return current

def angle_interpolation(direction, angle):
    if direction == 10:
        travel = (angle - (EXO.mid_pos - center_offset)) / ((EXO.min_pos + edge_offset) - (EXO.mid_pos - center_offset)) * profiles_position.instances
    elif direction == 20:
        travel = (angle - (EXO.mid_pos + center_offset)) / ((EXO.max_pos - edge_offset) - (EXO.mid_pos + center_offset)) * profiles_position.instances

    travel = int(round(travel))
    travel = max(min(travel, len(y_list_position) - 1), 0)
    return travel
    
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
                
        LSL = LSLResolver()
        exo_config = json.load(open(r"main/EXO_configuration.json", "r"))
        setup_dict, profiles_position, profiles_position_dict, profiles_time, profiles_time_dict, EXO_setup = initialize_EXO(exo_config, LSL)

        if setup_dict["save_data"]:
            frequency_file = create_file(setup_dict["frequency_path"])
        freqs = []

        EXO = SetupEXO(*EXO_setup)
        LSL.set_stop_event_frequency(setup_dict["loop_frequency"], EXO.stop_event)
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
        LSL.torque = 1
        i = 0
        dxl_goal_current = 0
        travel = 0
        center_offset = setup_dict["center_offset"]
        edge_offset = setup_dict["edge_offset"]

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
                        if LSL.direction == 99:
                            raise KeyboardInterrupt
                        elif LSL.direction not in (10, 20):
                            execute = False
                            EXO.execution = 0
                            dxl_goal_current = 0
                            EXO.write_current(dxl_goal_current)
                        else:    
                            y_list_position = profiles_position_dict[LSL.torque_profile]
                            y_list_time = profiles_time_dict[LSL.torque_profile]
                            direction = LSL.direction
                            correctness = LSL.correctness
                            torque = round(LSL.torque, 3)

                            i = 0
                            execute = True
                            start_time = perf_counter()
                            print(f"Torque Profile: {LSL.torque_profile}, Correctness: {correctness}, Direction: {direction}, torque level: {torque}")
                        
                    if present_position_deg < EXO.min_pos + edge_offset or EXO.max_pos - edge_offset < present_position_deg:
                        execute = False 
                        EXO.execution = 0
                        dxl_goal_current = 0
                        EXO.write_current(dxl_goal_current)
                    else:
                        if execute:  
                            if i >= len(y_list_time):
                                execute = False
                            else:
                                if EXO.execution != 1:
                                    EXO.execution = 1                                         
                                travel = angle_interpolation(direction, present_position_deg)
                                if direction == 20:
                                    if correctness == 1:
                                        dxl_goal_current = int(round(torque_to_current(torque) * y_list_position[travel]))
                                    else:
                                        if not setup_dict["time_control"]:                                          
                                            dxl_goal_current = int(round(-torque_to_current(torque) * y_list_position[travel]))
                                        else:
                                            dxl_goal_current = int(round(-torque_to_current(torque) * y_list_time[i]))
                                            i += 1
                                else:
                                    if correctness == 1:
                                        dxl_goal_current = int(round(-torque_to_current(torque) * y_list_position[travel]))
                                    else:
                                        if not setup_dict["time_control"]:                                          
                                            dxl_goal_current = int(round(torque_to_current(torque) * y_list_position[travel]))
                                        else:
                                            dxl_goal_current = int(round(torque_to_current(torque) * y_list_time[i]))
                                            i += 1
                            
                            EXO.demanded_torque = (1 - (1 - dxl_goal_current * 1000*SetupEXO.cur_unit / 8.247191)**2) / 0.082598  # [Nm]
                            if dxl_goal_current < 0:
                                EXO.demanded_torque = -EXO.demanded_torque
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