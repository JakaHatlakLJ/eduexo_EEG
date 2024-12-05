from time import perf_counter, sleep
try:
    while 1:
        print(perf_counter())
        sleep(1)
except KeyboardInterrupt as e:
    print(e)
