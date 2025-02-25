from pylsl import resolve_byprop, StreamInfo, StreamInlet, StreamOutlet, local_clock
from time import sleep
import threading

class LSLResolver:

    def __init__(self, stop_event = threading.Event(), receive = True, send = True):

        if send:
            # Create LSL stream for sending instructions to EXO
            info = StreamInfo(
                'Stream_EXO',           # name
                'EXO',                  # type
                4,                      # channel_count
                10000,                  # nominal rate=0 for irregular streams
                'float32',              # channel format
                'Eduexo_PC'             # source_id
            )
            self.outlet = StreamOutlet(info, max_buffered=1)
            print("Stream to EXO is online...")


        if receive:
            # Resolve LSL stream for receiving EXO data and create an inlet
            streams = resolve_byprop('type', 'Instructions', timeout= 5)
            if not streams:
                raise RuntimeError(f"No LSL stream found of type: 'Instructions'")
            self.inlet = StreamInlet(streams[0])
            print("Receiving instructions from PC...")

        self.stop_event = stop_event
        self.torque_profile = None

    def LSL_inlet(self):
        while not self.stop_event.is_set():
            sample, timestamp = self.inlet.pull_sample(timeout=1.0)

            if sample is not None:
                try:
                    self.torque_profile = int(sample[0])
                    self.correctness = int(sample[1])
                    self.direction = sample[2]
                    self.timestamp = timestamp
            
                except Exception as e:
                    print(f"Error: {e}")  # Handle potential decoding errors
        


    def LSL_outlet(self, motor_instance):
        while not self.stop_event.is_set():
            DATA = [
            motor_instance.present_position_deg, 
            motor_instance.present_velocity_deg, 
            motor_instance.present_torque, 
            motor_instance.execution  # Assuming execution is updated elsewhere
        ]

            # Send motor position data via stream
            self.outlet.push_sample(DATA)

