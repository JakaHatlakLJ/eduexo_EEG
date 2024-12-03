from pylsl import StreamInlet, resolve_stream

sample_names = ['Position', 'Velocity', 'Current', 'Torque']

# Resolve a stream
stream1 = resolve_stream('type', 'EEG')
stream2 = resolve_stream('type', 'EEG_KD')

# Create an inlet
inlet1 = StreamInlet(stream1[0])
inlet2 = StreamInlet(stream2[0])

print("Receiving data...")
while True:
    sample1, timestamp1 = inlet1.pull_sample()
    #sample_round = list(zip(sample_names, [round(num, 2) for num in sample1]))
    #sample_formated = "\t".join([f"{name}: {value:<{8}}" for name, value in sample_round])

    sample2, timestamp2 = inlet2.pull_sample()

    print(f"{sample1} Timestamp: {timestamp1}")
    print(f"{sample2} Timestamp: {timestamp1}")
  
