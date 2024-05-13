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

udp_client = espp.UdpSocket(espp.UdpSocketConfig(espp.Verbosity.DEBUG))

start = time.time()
def task_func():
    global start
    global udp_client
    port = 5555
    ip = "127.0.0.1"
    send_config = espp.UdpSendConfig(
        ip, port
    )
    elapsed = time.time() - start
    print(f"[{elapsed:.3f}] Hello from task")
    udp_client.send("Hello world!\n", send_config)
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
