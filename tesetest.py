import threading
from pylsl import resolve_byprop, StreamInlet, StreamOutlet, StreamInfo
from time import perf_counter
import numpy as np

from Torque_profiles import TorqueProfiles
# from EXO_LSL import LSLResolver
from EXO_setup import SetupEXO, getch

def LSL_get():
    while not EXO.stop_event.is_set():
        sample, timestamp = inlet.pull_sample(timeout=1.0)

        if sample is not None:
            b1 = sample[0]
            b2 = sample[1]
            b3 = sample[2]
            global recieved_torque
            global recieved_correctness
            global recieved_direction
            global tstamp
            recieved_torque = int(b1)
            recieved_correctness = int(b2)
            recieved_direction = int(b3)
            tstamp = timestamp

# def LSL_send(outlet, pos, vel, tor, execution):
#     DATA = [
#     pos, 
#     vel,
#     tor, 
#     execution  # Assuming execution is updated elsewhere
#     ]

#     # Send motor position data via stream
#     outlet.push_sample(DATA)

if __name__ == "__main__":
    EXO = SetupEXO(max_pos=165)

    pos_unit = 0.088            # [deg] = [dxl_unit] * [pos_unit]
    cur_unit = 2.69             # [mA]  = [dxl_unit] * [cur_unit]
    max_torque = 4
    max_current = 8.247191-8.247191*np.sqrt(1-0.082598*max_torque)            # [A]
    max_current = max_current * 1000.0                                        # [mA]
    max_current = round(max_current / cur_unit)                               # [dxl units]



        # Create LSL stream for sending instructions to EXO
    info = StreamInfo(
        'Stream_EXO',           # name
        'EXO',                  # type
        4,                      # channel_count
        10000,                  # nominal rate=0 for irregular streams
        'float32',              # channel format
        'Eduexo_PC'             # source_id
    )
    outlet = StreamOutlet(info, max_buffered=1)
    print("Stream to EXO is online...")

    #Lab Streaming Layer setup
    streams = resolve_byprop('type', 'Instructions')
    inlet = StreamInlet(streams[0])                 # Create an inlet
    print("Receiving data...")

    t_prof = TorqueProfiles(500)
    y_trap = t_prof.trapezoid()
    y_tri = t_prof.triangle()
    y_sin = t_prof.sinus()
    y_pulse = t_prof.pulse(1)
    y_smtrap = t_prof.smoothed_trapezoid()
    x_list = t_prof.x
    freq = t_prof.loop_frequency
    t_profile_dict = {0 : y_trap, 1 : y_tri, 2 : y_sin, 3 : y_pulse, 4 : y_smtrap}

    active = False

    recieved_torque = None
    recieved_direction = None
    recieved_correctness = None
    interval = 200 # [ms]
    

    while 1:
        # Start of new loop
        print("Press any key to continue! (or press ESC to quit!)")
        if getch() ==chr(0x1b):
            break
        else:
            EXO.stop_event.clear()
            CURRENT_BAUDRATE = EXO.start_system() #Initialize motor

            motor_data_thread = threading.Thread(target= EXO.motor_data, args = (True, False, False), daemon = True)
            motor_data_thread.start()
            LSL_thread = threading.Thread(target= LSL_get, daemon = True)
            LSL_thread.start()

            direction = 10
            correctness = 1
            tstamp = 0
            previous_tstamp = 0
            i = None
            dxl_goal_current = None
            travel = None
            y_list = []
            execution = 0
            present_velocity_deg = 0
            present_torque = 0

        try:
            #enable Torque and set Bus Watchdog
            with EXO.dxl_lock:
                EXO.torque_watchdog()
            previous_time = perf_counter() # Loop Timer

            while 1:
                current_time = perf_counter()

                if current_time - previous_time >= 1/freq:

                    with EXO.dxl_lock:
                        present_position_deg = EXO.present_position_deg
                    #     present_velocity_deg = EXO.present_velocity_deg
                    #     present_torque = EXO.present_torque

                    if tstamp is not None:
                        timestamp = tstamp

                    if timestamp != previous_tstamp:
                        if recieved_torque is not None:
                            y_list = t_profile_dict[recieved_torque]

                        if recieved_direction is not None:
                            direction = recieved_direction

                        if recieved_correctness is not None:
                            correctness = recieved_correctness
                        
                        i = 0
                        active = True
                        start_time = perf_counter()
                        
                    if present_position_deg < EXO.min_pos + 15 or EXO.max_pos - 15 < present_position_deg:
                        active = False 
                        execution = False
                        dxl_goal_current = 0
                        EXO.write_current(dxl_goal_current)
                    else:
                        if active:
                            if current_time - start_time >= 2.5:
                                execute = False 
                                dxl_goal_current = 0
                                execution = 0
                            else:                
                                if execution != 1:
                                    execution = 1            
                                if correctness == 1:
                                    if direction == 20:
                                        travel = int(round((present_position_deg - ((EXO.max_pos + EXO.min_pos)/2 - 2)) / (EXO.max_pos - (EXO.max_pos + EXO.min_pos)/2 - 15) * t_prof.instances))
                                        travel = max(0, min(travel, len(y_list) - 1))
                                        if travel <= 0:
                                            travel = 1
                                        dxl_goal_current = int(round(0.75 * max_current * y_list[travel]))
                                    else:
                                        travel = int(round((present_position_deg - ((EXO.max_pos + EXO.min_pos)/2 + 2 )) / (EXO.min_pos + 15 - (((EXO.max_pos + EXO.min_pos)/2 + 2))) * t_prof.instances))
                                        if travel <= 7:
                                            travel = 7
                                        dxl_goal_current = int(round(-1.1 * max_current * y_list[travel]))
                                else:
                                    if timestamp != previous_tstamp:
                                        step = len(y_list) / (t_prof.loop_frequency * 200/1000)
                                        y_list = [y_list[int(i * step)] for i in range(int(t_prof.loop_frequency * 200/1000))]
                                    if i >= len(y_list):
                                        active = False
                                        execution = 0
                                        i = 0
                                        continue
                                    if direction == 20:
                                        dxl_goal_current = int(round(0.75 * max_current * y_list[i]))
                                    else:
                                        dxl_goal_current = int(round(-1.1 * max_current * y_list[i]))
                                    i += 1

                            EXO.write_current(dxl_goal_current)
                            print(active, recieved_torque, direction, correctness, timestamp, round(present_position_deg), round(present_velocity_deg, 3), round(present_torque,3), travel, perf_counter(), len(y_list))


                    previous_time = current_time
                    previous_tstamp = timestamp
                    outlet.push_sample([present_position_deg, present_velocity_deg, present_torque, execution])
        except KeyboardInterrupt:
            print("Loop ended.")

        except Exception as e:
            print(e)

        finally:    
            EXO.stop_event.set()
            motor_data_thread.join()
            EXO.stop_system()
            EXO.portHandler.closePort()
