import time

from support_loader import espp

start = time.time()
def on_receive_data(data, sender_info):
    global start
    port = 5555
    elapsed = time.time() - start
    print(f"[{elapsed:.3f}] Received: '{data}'")
    str_data = ''.join(chr(x) for x in data)
    print(f"       as string: '{str_data}'")
    print(f"            from: {sender_info.address}:{sender_info.port}")
    ret_data = list("Data returned to sender goes here...")
    print(f"      responding: {len(ret_data)} bytes to {sender_info.address}:{sender_info.port}")
    # turn the list of characters into a list of integers
    ret_data = [ord(x) for x in ret_data]
    return ret_data

udp_client = espp.UdpSocket(espp.UdpSocket.Config(espp.Logger.Verbosity.debug))
port = 5555
buffer_size = 1024
receive_config = espp.UdpSocket.ReceiveConfig(
    port,
    buffer_size,
    False,
    '',
    on_receive_data
)
udp_client.start_receiving(espp.Task.Config("udp_task", None), receive_config)

time.sleep(10)

exit()
