import sys
import time

from support_loader import espp

udp_client = espp.UdpSocket(espp.UdpSocket.Config(espp.Logger.Verbosity.debug))

# defined out here so that it's only initialized once, not on each callback
start = time.time()
def task_func() -> bool:
    global start
    global udp_client
    port = 5555
    ip = "127.0.0.1"
    send_config = espp.UdpSocket.SendConfig(
        ip, port
    )
    elapsed = time.time() - start
    print(f"[{elapsed:.3f}] Hello from task")
    udp_client.send("Hello world!\n", send_config)
    time.sleep(.5)
    return False # we don't want to stop the task

task = espp.Task(
    task_func,
    espp.Task.BaseConfig("test task")
)
task.start()

time.sleep(5)

sys.exit(0)
