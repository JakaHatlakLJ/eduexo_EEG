#!/usr/bin/env python
# -*- coding: utf-8 -*-


import os
import sys
from time import perf_counter
import numpy as np
from dynamixel_sdk import * 
from gpiozero import LED
from main.Torque_profiles import TorqueProfiles
import threading
from main.EXO_setup import SetupEXO


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


cur_unit = 2.69                                                             # [mA]  = [dxl_unit] * [cur_unit]
max_torque  = 4                                                             # [Nm]
max_current = 8.247191-8.247191*np.sqrt(1-0.082598*max_torque)              # [A]
max_current = max_current * 1000.0                                          # [mA]
max_current = round(max_current / cur_unit)                                 # [dxl units]

t_prof = TorqueProfiles()
y_trap = t_prof.trapezoid()
y_tri = t_prof.triangle()
y_sin = t_prof.sinus()
y_pulse = t_prof.pulse()
y_smtrap = t_prof.smoothed_trapezoid()
x_list = t_prof.x
freq = t_prof.loop_frequency

EXO = SetupEXO()
g_direction = 0
active = False

def position_read(EXO):
    global g_direction  # Ensure we modify the global variable
    while not EXO.stop_event.is_set():
        try:
            # Read present position
            with EXO.dxl_lock:
                dxl_present_position, dxl_comm_result, dxl_error = EXO.packetHandler.read4ByteTxRx(
                    EXO.portHandler, EXO.DXL_ID, EXO.ADDR_PRESENT_POSITION
                )
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % EXO.packetHandler.getTxRxResult(dxl_comm_result))
                    continue
                elif dxl_error != 0:
                    print("%s" % EXO.packetHandler.getRxPacketError(dxl_error))
                    continue
            b1 = dxl_present_position.to_bytes(4, byteorder=sys.byteorder, signed=False)
            dxl_present_position = int.from_bytes(b1, byteorder=sys.byteorder, signed=True)

            if dxl_present_position >= (EXO.max_pos - 20) / EXO.pos_unit and g_direction == 1:
                g_direction = -1
            elif dxl_present_position <= (EXO.min_pos + 20) / EXO.pos_unit and g_direction == -1:
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
        EXO.stop_event.clear()
        CURRENT_BAUDRATE = EXO.start_system() #Initialize motor

        position_thread = threading.Thread(target= position_read, args = (EXO,), daemon = True)
        position_thread.start()
        
        direction = 1

    try:
        #enable Torque and set Bus Watchdog
        with EXO.dxl_lock:
            EXO.torque_watchdog()

        while 1:
            
            with EXO.dxl_lock:
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
            print(g_direction)

            if current_time - previous_time >= 1/freq:
                if i >= len(x_list):
                    active = False
                    i = 0
                    continue
                if direction == 1:
                    dxl_goal_current = int(round(0.85 * direction * max_current * y_list[i]))
                else:
                    dxl_goal_current = int(round(1.15 * direction * max_current * y_list[i]))
                EXO.write_current(dxl_goal_current)
                i += 1
                previous_time = current_time
    	
    except KeyboardInterrupt:
        print("Loop ended.")

    except Exception as e:
        print(e)

    finally:    
        EXO.stop_event.set()
        position_thread.join()
        EXO.stop_event.clear()
        EXO.stop_system()
