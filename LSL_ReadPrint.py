from pylsl import StreamInlet, resolve_byprop

try:
    # Resolve a stream
    stream1 = resolve_byprop('type', 'Events')

    # Create an inlet
    inlet = StreamInlet(stream1[0])

    print("Receiving data...")
    while True:
        # Use a timeout of 1 second
        sample, timestamp = inlet.pull_sample()
        if sample is not None:
            if "Event_ID" in sample[0]:
                print(f"{sample[0]}")

except KeyboardInterrupt:
    print("Stream reading interrupted by user.")
    K = D = 0
    print(K,D)
except Exception as e:
    K = D = 0
    print(K,D)
    print(f"An error occurred: {e}")