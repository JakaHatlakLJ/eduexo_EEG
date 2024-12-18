from pylsl import StreamInlet, resolve_stream
import time

sample_names = ['Position', 'Velocity', 'Current', 'Torque']

# Resolve a stream
streams = resolve_stream('type', 'EEG')

# Create an inlet
inlet = StreamInlet(streams[0])  

print("Receiving data...")
while True:
    sample, timestamp = inlet.pull_sample(timeout=0.001)
    print(f"{sample} Timestamp: {timestamp}")
  