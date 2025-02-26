from pylsl import resolve_byprop, StreamInfo, StreamInlet, StreamOutlet, local_clock
from time import sleep, perf_counter
import threading

class LSLResolver:

    def __init__(self, loop_frequency = 200, stop_event = threading.Event(), receive = True, send = True):
        
        self.loop_frequency = loop_frequency

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
        previous_t = perf_counter()
        while not self.stop_event.is_set():
            current_t = perf_counter()
            if current_t - previous_t >= 1 / (1.1 * self.loop_frequency):
                sample, timestamp = self.inlet.pull_sample(timeout=1.0)

                if sample is not None:
                    try:
                        self.torque_profile = int(sample[0])
                        self.correctness = int(sample[1])
                        self.direction = sample[2]
                        self.timestamp = timestamp
                
                    except Exception as e:
                        print(f"Error: {e}")  # Handle potential decoding errors
                
                previous_t = current_t
        
    def LSL_outlet(self, motor_instance):
        previous_t = perf_counter()
        while not self.stop_event.is_set():
            current_t = perf_counter()
            if current_t - previous_t >= 1 / (1.1 * self.loop_frequency):
                DATA = [
                motor_instance.present_position_deg, 
                motor_instance.present_velocity_deg, 
                motor_instance.present_torque, 
                motor_instance.execution  # Assuming execution is updated elsewhere
                ]

                # Send motor position data via stream
                self.outlet.push_sample(DATA)
                previous_t = current_t

