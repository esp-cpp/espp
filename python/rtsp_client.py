import sys
import time

import socket

import cv2
import numpy as np

import queue

from zeroconf import ServiceBrowser, ServiceListener, Zeroconf

from support_loader import espp

class MyListener(ServiceListener):
    def __init__(self):
        self.service_ip = None
        self.service_port = None
        self.found_service = False

    def update_service(self, zc: Zeroconf, type_: str, name: str) -> None:
        print(f"Service {name} updated")

    def remove_service(self, zc: Zeroconf, type_: str, name: str) -> None:
        print(f"Service {name} removed")

    def add_service(self, zc: Zeroconf, type_: str, name: str) -> None:
        info = zc.get_service_info(type_, name)
        print(f"Service {name} added, service info: {info}")
        self.service_ip = socket.inet_ntoa(info.addresses[0])
        self.service_port = info.port
        self.found_service = True

frame_queue = queue.Queue()
# This function will be called when a new JPEG frame is received
def on_receive_frame(frame):
    global frame_queue
    # print(f"Received frame of size {frame.get_width()}x{frame.get_height()} pixels")
    buf = frame.get_data()
    decoded = cv2.imdecode(np.frombuffer(buf, dtype=np.uint8), cv2.IMREAD_COLOR)
    if decoded is not None:
        frame_queue.put_nowait(decoded)
    else:
        print("Failed to decode frame")
    return

# use zeroconf to discover the rtsp service at "_rtsp._tcp.local."
service = "_rtsp._tcp.local."
zeroconf = Zeroconf()
listener = MyListener()
browser = ServiceBrowser(zeroconf, service, listener)

print("Finding Service...\n")
while not listener.found_service:
    time.sleep(0.1)

server_address, server_port = listener.service_ip, listener.service_port
media_path = "/mjpeg/1"
rtsp_uri = f"rtsp://{server_address}:{server_port}{media_path}"
print(f"Found service at uri: {rtsp_uri}\n")
client_config = espp.RtspClient.Config(
    server_address = server_address,
    rtsp_port = server_port,
    path = media_path,
    on_jpeg_frame = on_receive_frame,  # callback for received frames
    log_level = espp.Logger.Verbosity.info
)
rtsp_client = espp.RtspClient(client_config)

# initialize the RTSP client and connect to the server
ec = espp.Error()

print("Connecting to RTSP server...")
rtsp_client.connect(ec)
if ec:
    print(f"Error connecting to RTSP server: {ec}")
    sys.exit(1)

print("Describing RTSP client...")
rtsp_client.describe(ec)
if ec:
    print(f"Error describing RTSP client: {ec}")
    sys.exit(1)

print("Setting up RTSP client...")
rtsp_client.setup(ec)
if ec:
    print(f"Error setting up RTSP client: {ec}")
    sys.exit(1)

print("Playing RTSP client...")
rtsp_client.play(ec)
if ec:
    print(f"Error playing RTSP client: {ec}")
    sys.exit(1)

# right now our client should be connected and receiving frames, calling the
# callback.

window_name = 'RTSP Client'
cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
# set the window as always on top
cv2.setWindowProperty(window_name, cv2.WND_PROP_TOPMOST, 1)

# capture start time here so we can measure FPS and print it at the end
start_time = time.time()
num_frames = 0

print("RTSP client is now connected and receiving frames")
print("\nPress 'q' to quit or Ctrl+C to stop the client and exit.\n")
# wait until the user presses ctrl+c to stop the client
try:
    while True:
        try:
            frame = frame_queue.get_nowait()
            cv2.imshow('RTSP Client', frame)
            num_frames += 1
        except queue.Empty:
            pass
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):  # press 'q' to quit
            break
except KeyboardInterrupt:
    pass
finally:
    print("Stopping RTSP client...")

# calculate the elapsed time and FPS
elapsed_time = time.time() - start_time
fps = num_frames / elapsed_time if elapsed_time > 0 else 0
print(f"Captured {num_frames} frames in {elapsed_time:.2f} seconds ({fps:.2f} FPS)")

# stop the client and unregister the service
rtsp_client.teardown(ec)
if ec:
    print(f"Error tearing down RTSP client: {ec}")
zeroconf.close()

sys.exit(0)
