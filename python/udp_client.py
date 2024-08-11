import time

from support_loader import espp

udp_client = espp.UdpSocket(espp.UdpSocket.Config(espp.Logger.Verbosity.debug))

start = time.time()
def task_func():
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

task = espp.Task(espp.Task.SimpleConfig(
    task_func, #function
    # config
    espp.Task.BaseConfig("test task")
))
task.start()

time.sleep(5)

exit()
