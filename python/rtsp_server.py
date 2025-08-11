import time

import cv2

from zeroconf import ServiceInfo, Zeroconf

import socket

from simplejpeg import encode_jpeg

from support_loader import espp

# get the address of this machine using python's socket library
fqdn = socket.gethostname()
hostname = fqdn.split('.')[0]  # Get the hostname without domain
ip_addr = socket.gethostbyname(hostname)
server_ip = ip_addr
server_port = 8554
media_path = 'mjpeg/1'
print(f"Starting RTSP Server IP: {server_ip}, Port: {server_port}, Path: {media_path}")
rtsp_server = espp.RtspServer(espp.RtspServer.Config(
    server_address = server_ip,
    port = server_port,
    path = media_path,
    log_level = espp.Logger.Verbosity.warn
))
rtsp_server.start()

# use zeroconf/mDNS to advertise the rtsp server as '_rtsp._tcp.local.'
zeroconf = Zeroconf()
info = ServiceInfo(
    "_rtsp._tcp.local.",
    f"{hostname}._rtsp._tcp.local.",
    addresses=[socket.inet_pton(socket.AF_INET, server_ip)],
    port=server_port,
    properties={},
    server=hostname + '.local.'
)
zeroconf.register_service(info)
print(f"Registered RTSP service: {info}")

time.sleep(1)  # wait for the server to start

# capture a frame from the default camera
cap = cv2.VideoCapture(0)

# make a task to grab a video or the display, resize it, and call
# `rtsp_server.send_frame(frame)` periodically
def task_func():
    global rtsp_server
    global cap
    # capture a frame from the display or a video source
    # for example, using OpenCV or PIL
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame")
        return True # stop the task
    # resize the frame to a smaller size (320x240)
    image_size = (320, 240)
    resized_frame = cv2.resize(frame, image_size)
    jpeg_quality = 20

    # # use opencv to convert to jpeg
    # ret, image_bytes = cv2.imencode('.jpg', resized_frame, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])
    # image_bytes = image_bytes.tobytes()
    # if not ret:
    #     print("Failed to encode frame as JPEG")
    #     return False

    # encode the frame as JPEG
    image_bytes = encode_jpeg(resized_frame, quality=jpeg_quality, colorspace='BGR')

    # create a JpegFrame object with the image data and bytes
    frame = espp.JpegFrame(image_bytes)
    # print(f"Captured frame {frame.get_width()} x {frame.get_height()} pixels")

    # send the frame to the RTSP server
    # print(f"Sending frame of size {len(image_bytes)} bytes to RTSP clients...")
    rtsp_server.send_frame(frame)
    time.sleep(.1)
    return False # we don't want to stop the task

task = espp.Task(espp.Task.Config(
    task_func, #function
    # config
    espp.Task.BaseConfig("rtsp server task")
))
task.start()

print("RTSP Server started, press Ctrl+C to stop...")
try:
    while True:
        time.sleep(1)  # Keep the main thread alive
except KeyboardInterrupt:
    print("\nKeyboard interrupt received, stopping...")

print("Releasing camera...")
cap.release()
print("Unregistering service...")
zeroconf.unregister_service(info)
print("Closing zeroconf...")
zeroconf.close()

exit()
