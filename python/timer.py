import time

from support_loader import espp

start = time.time()
def timer_func():
    global start
    elapsed = time.time() - start
    print(f"[{elapsed:.3f}] Hello from timer")
    return False # we don't want to stop the timer

timer = espp.Timer(espp.Timer.Config(
    "test timer", #name
    0.5, # period
    0.0, # delay
    timer_func,
    True, # auto start
    4096, # stack size (unused except on ESP)
    1, # priority (unused except on ESP)
    -1, # core (unused except on ESP)
    espp.Logger.Verbosity.none
))

time.sleep(5)

exit()
