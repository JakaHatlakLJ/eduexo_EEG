from pylsl import resolve_byprop, StreamInfo, StreamInlet, StreamOutlet, local_clock
from time import sleep, perf_counter
import threading, json

class LSLResolver:
    """
    Class to handle LSL (Lab Streaming Layer) communication for the EXO system.
    """

    def __init__(self, receive=True, send=True):
        """
        Initialize the LSLResolver.

        Parameters:
        loop_frequency (int): Frequency of the loop in Hz.
        stop_event (threading.Event): Event to signal stopping of threads.
        receive (bool): Flag to enable receiving data.
        send (bool): Flag to enable sending data.
        """

        if receive:
            print(f"Looking for LSL stream of type: 'SETUP'...")
            while True:
                # Resolve LSL stream for receiving SETUP parameters
                streams = resolve_byprop('type', 'SETUP', timeout=10)
                if streams:
                    break
                print(f"No LSL stream found of type: 'SETUP'. Retrying...")
            
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
            inlet = StreamInlet(streams[0])
            sample = None
            while sample is None:
                sample, _ = inlet.pull_sample(timeout=1.0)
            json_string = sample[0]
            instructions = json.loads(json_string)
            self.max_p = instructions["maximum_arm_position_deg"]
            self.min_p = instructions["minimum_arm_position_deg"]
            self.center_offset = instructions["center_offset_deg"]
            self.edge_offset = instructions["edge_offset_deg"]
            self.torque_limit = instructions["torque_limit"]
            self.duration_of_trials = instructions["duration_of_trials"]
            self.incorect_execution_positon_control = instructions["incorect_execution_positon_control"]
            self.incorrect_execution_time_ms = instructions["incorrect_execution_time_ms"]

            print("Received SETUP parameters from PC!")

            print(f"Looking for LSL stream of type: 'Instructions'...")
            while True:
                # Resolve LSL stream for receiving EXO data and create an inlet
                streams = resolve_byprop('type', 'Instructions', timeout=10)
                if streams:
                    break
                print(f"No LSL stream found of type: 'Instructions'. Retrying...")
            self.inlet = StreamInlet(streams[0])
            print("Receiving instructions from PC...")

        self.torque_profile = None

    def set_stop_event_frequency(self, loop_frequency: int, stop_event=threading.Event()):
        self.stop_event = stop_event
        self.loop_frequency = loop_frequency

    def LSL_inlet(self):
        """
        Method to continuously receive data from the LSL stream.
        """
        previous_t = perf_counter()
        while not self.stop_event.is_set():
            current_t = perf_counter()
            if current_t - previous_t >= 1 / self.loop_frequency:
                self.inlet.flush()
                sample, timestamp = self.inlet.pull_sample(timeout=1.0)

                if sample is not None:
                    try:
                        # Extract data from the sample
                        self.torque_profile = int(sample[0])
                        self.correctness = int(sample[1])
                        self.direction = int(sample[2])
                        self.power = sample[3]
                        self.timestamp = timestamp
                    except Exception as e:
                        print(f"Error: {e}")  # Handle potential decoding errors
                
                previous_t = current_t
        
    def LSL_outlet(self, motor_instance):
        """
        Method to send data to the LSL stream.

        Parameters:
        motor_instance: Instance of the motor to get data from.
        """
        DATA = [
            motor_instance.present_position_deg, 
            motor_instance.present_velocity_deg, 
            motor_instance.present_torque, 
            motor_instance.execution  # Assuming execution is updated elsewhere
        ]

        # Send motor position data via stream
        self.outlet.push_sample(DATA)


