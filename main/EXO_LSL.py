from pylsl import resolve_byprop, StreamInfo, StreamInlet, StreamOutlet, local_clock
from time import sleep, perf_counter
import threading, json

class LSLResolver:
    """
    Handles Lab Streaming Layer (LSL) communication for the EXO system.

    This class can both send and receive data streams using LSL, facilitating communication
    of setup parameters, control instructions, and motor data between the EXO hardware and PC.
    """

    def __init__(self, receive=True, send=True):
        """
        Initialize the LSLResolver, resolving and/or creating the necessary LSL streams.

        Args:
            receive (bool): If True, resolves input LSL streams for receiving data.
            send (bool): If True, creates an LSL stream for sending data.
        """

        if receive:
            print(f"Looking for LSL stream of type: 'SETUP'...")
            while True:
                # Try to resolve an LSL stream of type 'SETUP' (from PC)
                streams = resolve_byprop('type', 'SETUP', timeout=10)
                if streams:
                    break
                print(f"No LSL stream found of type: 'SETUP'. Retrying...")
            
        if send:
            # Create an LSL stream for sending EXO data to the PC
            info = StreamInfo(
                'Stream_EXO',           # Stream name
                'EXO',                  # Stream type
                6,                      # Number of channels
                10000,                  # Data rate (Hz)
                'float32',              # Channel format
                'Eduexo_PC'             # Source ID
            )
            self.outlet = StreamOutlet(info, max_buffered=1)
            print("Stream to EXO is online...")

        if receive:
            # Receive setup parameters as a JSON-formatted string
            inlet = StreamInlet(streams[0])
            sample = None
            while sample is None:
                sample, _ = inlet.pull_sample(timeout=1.0)
            json_string = sample[0]
            instructions = json.loads(json_string)

            # Store setup parameters as attributes
            self.max_p = instructions["maximum_arm_position_deg"]
            self.min_p = instructions["minimum_arm_position_deg"]
            self.center_offset = instructions["center_offset_deg"]
            self.edge_offset = instructions["edge_offset_deg"]
            self.torque_limit = instructions["torque_limit"]
            self.incorect_execution_time_control = instructions["incorect_execution_time_control"]
            self.incorrect_execution_time_ms = instructions["incorrect_execution_time_ms"]

            print("Received SETUP parameters from PC!")

            print(f"Looking for LSL stream of type: 'Instructions'...")
            while True:
                # Resolve the stream for receiving instructions
                streams = resolve_byprop('type', 'Instructions', timeout=10)
                if streams:
                    break
                print(f"No LSL stream found of type: 'Instructions'. Retrying...")
            self.inlet = StreamInlet(streams[0])
            print("Receiving instructions from PC...")

        self.torque_profile = None  # Placeholder for the latest torque profile

    def set_stop_event_frequency(self, loop_frequency: int, stop_event=threading.Event()):
        """
        Set the stop event and loop frequency for streaming threads.

        Args:
            loop_frequency (int): Frequency of the loop in Hz.
            stop_event (threading.Event): Event to signal threads to stop.
        """
        self.stop_event = stop_event
        self.loop_frequency = loop_frequency

    def LSL_inlet(self):
        """
        Continuously receive and process samples from the LSL inlet stream.

        Updates the torque profile and other attributes based on incoming data.
        """
        previous_t = perf_counter()
        while not self.stop_event.is_set():
            current_t = perf_counter()
            if current_t - previous_t >= 1 / self.loop_frequency:
                self.inlet.flush()
                sample, timestamp = self.inlet.pull_sample(timeout=1.0)

                if sample is not None:
                    try:
                        # Unpack incoming sample values
                        self.torque_profile = int(sample[0])
                        self.correctness = int(sample[1])
                        self.direction = int(sample[2])
                        self.torque = sample[3]
                        self.timestamp = timestamp
                    except Exception as e:
                        print(f"Error: {e}")  # Handle conversion or unpacking errors
                
                previous_t = current_t
        
    def LSL_outlet(self, motor_instance):
        """
        Send current motor information as a sample through the LSL outlet stream.

        Args:
            motor_instance: Instance of the motor class holding state to transmit.
        """
        DATA = [
            motor_instance.present_position_deg,    # Current position in degrees
            motor_instance.present_velocity_deg,    # Current velocity in degrees/sec
            motor_instance.present_torque,          # Current torque
            motor_instance.execution,               # Execution state
            motor_instance.demanded_torque,         # Commanded torque
            motor_instance.present_force            # Current force on Load-cell
        ]

        # Transmit the data sample to the stream
        self.outlet.push_sample(DATA)