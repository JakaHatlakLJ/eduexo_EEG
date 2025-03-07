import sys
from time import sleep
import threading
from pylsl import StreamInfo, StreamOutlet
from main.EXO_setup import SetupEXO, getch

# Create a new stream outlet
info = StreamInfo("Stream_EXO", "EXO", 4, 10000, 'float32', 'test_LSL')
outlet = StreamOutlet(info, max_buffered=1)

# Choose which parameters you want to send
send_position = True
send_velocity = False
send_torque = False
send_derived_velocity = False

if send_velocity:
    send_derived_velocity = False

EXO = SetupEXO()

data_thread = threading.Thread(target=EXO.motor_data, args=(True, False, True, True, False))

print(f'Sending motor position through stream: {info.name}, of type: {info.type} \nPress any key to contionue (or press ESC to quit!)')
if getch() == chr(0x1b):
    print("Quiting...")
    sys.exit()
else:
    EXO.start_system()
    data_thread.start()
    active = True
    print(f'Position sending is active')

    try:    
        while active:
            position = EXO.present_position_deg
            velocity = EXO.present_velocity_deg
            torque = EXO.present_torque
            execution = 0

            # Send motor position data via stream
            DATA = [position, velocity, torque, execution]
            outlet.push_sample(DATA)
            sleep(0.001)

    except KeyboardInterrupt:
        EXO.stop_event.set()
        data_thread.join()
        EXO.stop_event.clear()
        print("Program interrupted")

    finally:
        EXO.stop_system()
