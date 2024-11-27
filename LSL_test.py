from pylsl import StreamInlet, resolve_stream
import time

def find_stream(timeout=1):
    """Attempts to find a stream with a timeout."""
    print("Searching for an EEG stream...")
    while True:
        try:
            streams = resolve_stream('type', 'EEG', timeout)
            if streams:
                return streams[0]  # Return the first stream found
            else:
                print("No streams found. Retrying...")
        except KeyboardInterrupt:
            print("\nSearch stopped by user.")
            exit()

# Search for a stream with periodic checks
streams = find_stream(timeout=1.0)

# Create an inlet
inlet = StreamInlet(streams[0])

print("Receiving data...")
try:
    while True:
        sample, timestamp = inlet.pull_sample()
        print(f"Sample: {round(sample[0], 3)}, Timestamp: {timestamp}")
except:
    KeyboardInterrupt