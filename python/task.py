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
def task_func():
    global start
    elapsed = time.time() - start
    print(f"[{elapsed:.3f}] Hello from task")
    time.sleep(.5)
    return False # we don't want to stop the task

task = espp.Task(espp.TaskSimpleConfig(
    task_func, #function
    # config
    espp.TaskBaseConfig("test task")
))
task.start()

time.sleep(5)

exit()
