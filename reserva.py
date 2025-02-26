#!/usr/bin/env python
# -*- coding: utf-8 -*-


import os
import sys
from time import perf_counter, sleep
import numpy as np
from dynamixel_sdk import * 
from pylsl import StreamInlet, resolve_byprop, StreamInfo, StreamOutlet
from gpiozero import LED
import threading
import json

from EXO_setup import SetupEXO, getch
from Torque_profiles import TorqueProfiles
from EXO_LSL import LSLResolver

def initialize_EXO(EXO_config, setup_dict = None):
    if setup_dict is None:
        setup_dict = {}
        
    setup_dict["torque_limit"] = EXO_config["torque_limit"]
    setup_dict["max_torque"] = EXO_config["max_torque_during_trial"]
    setup_dict["min_pos"] = EXO_config["min_pos"]
    setup_dict["max_pos"] = EXO_config["max_pos"]   
    setup_dict["control_mode"] = EXO_config["DXL_control_mode"]
    setup_dict["baudrate"] = EXO_config["baudrate"]
    setup_dict["loop_frequency"] = EXO_config["control_frequency"]
    setup_dict["device_name"] = EXO_config["port_name"]
    setup_dict["frequency_path"] = EXO_config["frequency_path"]
    setup_dict["save_data"] = True if EXO_config["save_data"] == 1 else False

    # Unit conversion
    pos_unit = 0.088            # [deg] = [dxl_unit] * [pos_unit]
    vel_unit = 0.229            # [rpm] = [dxl_unit] * [vel_unit]
    cur_unit = 2.69             # [mA]  = [dxl_unit] * [cur_unit]
    max_current = 8.247191-8.247191*np.sqrt(1-0.082598*setup_dict["max_torque"])            # [A]
    max_current = max_current * 1000.0                                                      # [mA]
    setup_dict["max_current"] = round(max_current / cur_unit)                               # [dxl units]

    # Torque Profiles
    t_prof = TorqueProfiles(loop_frequency=setup_dict["loop_frequency"])
    y_trap = t_prof.trapezoid()
    y_tri = t_prof.triangle()
    y_sin = t_prof.sinus()
    y_pulse = t_prof.pulse(width_portion=1)
    y_smtrap = t_prof.smoothed_trapezoid()
    x_list = t_prof.x
    t_profile_dict = {0 : y_trap, 1 : y_tri, 2 : y_sin, 3 : y_pulse, 4 : y_smtrap}

    EXO_setup_list = [
        setup_dict["torque_limit"],
        setup_dict["min_pos"],
        setup_dict["max_pos"],
        setup_dict["control_mode"],
        setup_dict["baudrate"],
        setup_dict["loop_frequency"],
        setup_dict["device_name"]
        ]

    return setup_dict, x_list, t_profile_dict, EXO_setup_list

# frequency loging
def create_file(frequency_path):
    os.makedirs(os.path.join(frequency_path), exist_ok=True)
    file_idx = len([filename for filename in os.listdir(os.path.join(frequency_path)) if filename.startswith("frequency_data")])
    file_idx = f'{file_idx:02d}'
    frequency_file = open(os.path.join(frequency_path, f"frequency_data_EXO_{file_idx}.txt"), "w")
    return frequency_file

if __name__ == "__main__":

    while 1:
        # Start of new loop
        print("Press any key to continue! (or press ESC to quit!)")
        if getch() ==chr(0x1b):
            break
        
        exo_config = json.load(open(r"EXO_configuration.json", "r"))
        setup_dict, x_list, t_profile_dict, EXO_setup = initialize_EXO(exo_config)

        if setup_dict["save_data"]:
            frequency_file = create_file(setup_dict["frequency_path"])
        first_loop = True
        freqs = []

        EXO = SetupEXO(*EXO_setup)
        LSL = LSLResolver(EXO.stop_event)
        CURRENT_BAUDRATE = EXO.start_system() #Initialize motor

        t0 = t1 = t5 = perf_counter() # Used for results printing timing
        execute = False

        # Initialize threads
        motor_data_thread = threading.Thread(
            target=EXO.motor_data,
            args=(True, True, False),
            daemon=True
            )
        motor_data_thread.start()

        LSL_inlet_thread = threading.Thread(
            target = LSL.LSL_inlet,
            daemon = True
            )
        LSL_inlet_thread.start()

        LSL_outlet_thread = threading.Thread(
            target = LSL.LSL_outlet,
            args = [EXO], 
            daemon = True
            )
        LSL_outlet_thread.start()

        try:
            #enable Torque and set Bus Watchdog
            with EXO.dxl_lock:
                EXO.torque_watchdog()

            present_velocity_deg = 0 # velocity for initial calculation 

            previous_time = perf_counter() # Loop Timer
            previous_execute = False

            while 1:
                current_time = perf_counter()

                if current_time - previous_time >= 1 / setup_dict["loop_frequency"]:                
                    # Read present position     
                    with EXO.position_lock:
                        present_position_deg = EXO.present_position_deg    
                    with EXO.velocity_lock:
                        present_velocity_deg = EXO.present_velocity_deg    
                    with EXO.torque_lock:
                        present_torque = EXO.present_torque    
                    

                    if LSL.torque_profile is not None:
                        execute = True
                        y_list = t_profile_dict[LSL.torque_profile]
                        i = 0

                    if execute:
                        
                        # Calculate goal current
                        if (present_position_deg < setup_dict["min_pos"] + 5 or present_position_deg > setup_dict["max_pos"] - 5):           # Check if position is within position limits 
                            dxl_goal_current = dxl_goal_torque = 0
                            EXO.write_current(0)
                            execute = False
                            y_list = None
                        else:
                            if LSL.correctness == 1:                        
                                if LSL.direction == 20:
                                    travel = int(round((present_position_deg - (EXO.max_pos + EXO.min_pos)/2) * 100 / (EXO.max_pos - 5 - (EXO.max_pos + EXO.min_pos)/2)))
                                    travel = max(0, min(travel, len(y_list) - 1))
                                    if travel < 0:
                                        travel = 0
                                    dxl_goal_current = int(round(0.75 * setup_dict["max_current"] * y_list[travel]))
                                else:
                                    travel = int(round((present_position_deg - (EXO.max_pos + EXO.min_pos)/2) * 100 / (EXO.min_pos + 5 - ((EXO.max_pos + EXO.min_pos)/2))))
                                    if travel < 0:
                                        travel = 0
                                    dxl_goal_current = int(round(-1.15 * setup_dict["max_current"] * y_list[travel]))
                            else:
                                if i >= len(x_list):
                                    execute = False
                                    y_list = None
                                    i = 0
                                    continue
                                else:
                                    if LSL.direction == 20:
                                        dxl_goal_current = int(round(0.75 * setup_dict["max_current"] * y_list[i]))
                                    else:
                                        dxl_goal_current = int(round(-1.15 * setup_dict["max_current"] * y_list[i]))
                                    i += 1
                            # Write goal current
                            if -setup_dict["max_current"] < dxl_goal_current < setup_dict["max_current"]:       # Check if goal current is within current limits
                                EXO.write_current(dxl_goal_current)
                                if previous_execute == False:
                                    EXO.execution = 1
                            else:
                                print("Goal current is too high.")
                                break
                    previous_execute = execute
                
                    freq = 1 / (current_time - previous_time)
                    freqs.append(freq)

                    previous_time = current_time

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