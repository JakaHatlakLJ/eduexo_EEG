import time
from time import perf_counter
import numpy as np
from ahrs.filters import Tilt
import serial

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1.0)      # open serial # timeout is the amount of time the command waits for data
time.sleep(3)                                               # gives Arduino time to start serial
ser.reset_input_buffer()                                    # fresh buffer
print('Serial OK')
prev_t = 0

try:
    while 1:
        # Get force sensor data from Arduino
        curent_t = perf_counter()
        if curent_t-prev_t >= 0.005:
            ard_line = ser.readline().decode('utf-8').rstrip()  # reads up to new line symbol # removes newline symbol
            frequency = 1/(curent_t-prev_t)   
            print(frequency)
            prev_t = curent_t
except KeyboardInterrupt:
    ser.close()