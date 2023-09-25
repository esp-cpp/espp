import time

try:
    print("trying to import espp...")
    import espp
except ImportError:
    print("espp not found, trying to import from ../lib/pc")
    print("NOTE: in general, you should add espp/lib/pc to your PYTHONPATH")
    import sys
    sys.path.append("../lib/pc")
    import espp
else:
    print("espp imported")

start = time.time()
def timer_func():
    global start
    elapsed = time.time() - start
    print(f"[{elapsed:.3f}] Hello from timer")
    return False # we don't want to stop the timer

timer = espp.Timer(espp.TimerConfig(
    "test timer", #name
    0.5, # period
    0.0, # delay
    timer_func,
    True, # auto start
    4096, # stack size (unused except on ESP)
    1, # priority (unused except on ESP)
    -1, # core (unused except on ESP)
    espp.Verbosity.NONE
))

time.sleep(5)

exit()
