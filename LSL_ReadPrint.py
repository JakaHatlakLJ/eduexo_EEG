from pylsl import StreamInlet, resolve_stream

try:
    # Resolve a stream
    stream1 = resolve_stream('type', 'EEG')

    # Create an inlet
    inlet = StreamInlet(stream1[0])

    print("Receiving data...")
    while True:
        # Use a timeout of 1 second
        sample, timestamp = inlet.pull_sample(timeout=1.0)
        if sample is not None:
            print(f"{sample} Timestamp: {timestamp}")

except KeyboardInterrupt:
    print("Stream reading interrupted by user.")
    K = D = 0
    print(K,D)
except Exception as e:
    K = D = 0
    print(K,D)
    print(f"An error occurred: {e}")