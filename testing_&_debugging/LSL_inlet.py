from pylsl import StreamInlet, resolve_byprop

# Resolve a stream
stream1 = resolve_byprop('type', 'EEG')
# stream2 = resolve_byprop('type', 'EEG_KD')

# Create an inlet
inlet1 = StreamInlet(stream1[0])
# inlet2 = StreamInlet(stream2[0])

print("Receiving data...")
while True:
    sample1, timestamp1 = inlet1.pull_sample()
    # sample2, timestamp2 = inlet2.pull_sample()

    print(f"{sample1} Timestamp: {timestamp1}")
    # print(f"{sample2} Timestamp: {timestamp2}")
  
