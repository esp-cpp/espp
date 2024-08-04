import time

from support_loader import espp

start = time.time()
def task_func():
    global start
    elapsed = time.time() - start
    print(f"[{elapsed:.3f}] Hello from task")
    time.sleep(.5)
    return False # we don't want to stop the task

task = espp.Task(espp.Task.SimpleConfig(
    task_func, #function
    # config
    espp.Task.BaseConfig("test task")
))
task.start()

time.sleep(5)

exit()
