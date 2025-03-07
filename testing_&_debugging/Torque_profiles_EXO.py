import sys
from time import perf_counter
import numpy as np
from dynamixel_sdk import * 
import threading

from main.Torque_profiles import TorqueProfiles
from main.EXO_setup import SetupEXO, getch

# Initialize EXO
EXO = SetupEXO()

# Initalize torque profiles
t_prof = TorqueProfiles()
y_trap = t_prof.trapezoid()
y_tri = t_prof.triangle()
y_sin = t_prof.sinus()
y_pulse = t_prof.pulse()
y_smtrap = t_prof.smoothed_trapezoid()
x_list = t_prof.x
freq = t_prof.loop_frequency

# Max torque profile value
max_torque  = 1.0                                                           # [Nm]
max_current = 8.247191-8.247191*np.sqrt(1-0.082598*max_torque)              # [A]
max_current = max_current * 1000.0                                          # [mA]
max_current = round(max_current / EXO.cur_unit)                             # [dxl units]


# Start of a loop
print("Press any key to continue! (or press ESC to quit!)")
if getch() in (chr(0x1b), chr(0x03)):
    sys.exit()
else:
    EXO.start_system()

    position_thread = threading.Thread(target=EXO.motor_data, args=(True, False, False, False), daemon = True)
    position_thread.start()

    position = 0
    active = False
    direction = 1
    previous_time = perf_counter()

    try:
        #enable Torque on EXO and initialize Bus Watchdog
        EXO.torque_watchdog()

        while True:
            with EXO.dxl_lock:
                position = EXO.present_position_deg
            if position >= (EXO.max_pos - 10) and direction == 1:
                direction = -1
            elif position <= (EXO.min_pos + 10) and direction == -1:
                direction = 1

            if not active:
                i = 0
                previous_time = perf_counter() # Loop Timer
                active = True

                print("Press 1 for Trapezoid, 2 for Triangle, 3 for Sinus, 4 for Pulse, 5 for Smoothed Trapezoid, (or ESC to quit!)")
                key = getch()
                if key in (chr(0x1b), chr(0x03)):
                    raise KeyboardInterrupt()
                elif key == chr(0x31):                  # 1
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
                    active = False                    

            if active:
                current_time = perf_counter()
                if current_time - previous_time >= 1/EXO.loop_frequency:
                    if i >= len(x_list):
                        active = False
                        dxl_goal_current = int(0)
                    else:
                        if direction == 1:
                            dxl_goal_current = int(round(0.85 * direction * max_current * y_list[i]))
                        else:
                            dxl_goal_current = int(round(1.15 * direction * max_current * y_list[i]))
                    EXO.write_current(dxl_goal_current)
                    i += 1
                    previous_time = current_time                        

    except KeyboardInterrupt:
        print("Loop ended")
    
    except Exception as e:
        print(e)

    finally:
        EXO.stop_event.set()
        position_thread.join()
        EXO.stop_event.clear()
        EXO.stop_system()