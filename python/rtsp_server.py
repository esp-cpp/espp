import sys
import time

import asyncio

import argparse

import mss
import cv2
import numpy as np

from zeroconf import IPVersion, ServiceInfo, Zeroconf

import socket

from simplejpeg import encode_jpeg

from support_loader import espp

# This script sets up an RTSP server that captures frames from the specified
# camera (default is 0) or the display, encodes them as JPEG, and serves them
# over RTSP.

cap = None
use_display = False

# initialize the RTSP server
async def init_server(server_ip, server_port=8554, hostname='localhost', media_path='mjpeg/1'):
    print(f"Starting RTSP Server IP: {server_ip}, Port: {server_port}, Path: {media_path}")
    rtsp_server = espp.RtspServer(espp.RtspServer.Config(
        server_address = server_ip,
        port = server_port,
        path = media_path,
        log_level = espp.Logger.Verbosity.warn
    ))
    rtsp_server.start()

    info = ServiceInfo(
        "_rtsp._tcp.local.",
        f"python rtsp server._rtsp._tcp.local.",
        addresses=[socket.inet_aton(server_ip)],
        port=server_port,
        properties={"path": media_path},
        server=hostname + '.local.'
    )

    ip_version = IPVersion.V4Only

    # use zeroconf/mDNS to advertise the rtsp server as '_rtsp._tcp.local.'
    zeroconf = Zeroconf(ip_version=ip_version)
    await zeroconf.async_register_service(info)
    print(f"Registered RTSP service: {info}")

    return rtsp_server, zeroconf, info

# function to capture a frame, resize it, and encode it as JPEG
def get_frame(image_size=(320, 240), jpeg_quality=10):
    global cap
    global use_display
    if cap is None:
        print("Video capture is not initialized")
        return None
    if use_display:
        frame = np.array(cap.grab(cap.monitors[1]))  # capture the primary monitor
        if frame is None:
            print("Failed to capture frame from display")
            return None
        # convert from rgba to bgr
        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)
    else:
        # capture a frame from the display or a video source
        # for example, using OpenCV or PIL
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame")
            return None
    # resize the frame to a smaller size (320x240)
    resized_frame = cv2.resize(frame, image_size)

    # encode the frame as JPEG
    image_bytes = encode_jpeg(resized_frame, quality=jpeg_quality, colorspace='BGR')

    return image_bytes

# make a task to grab a video or the display, resize it, and call
# `rtsp_server.send_frame(frame)` periodically
def task_func(rtsp_server, jpeg_quality=10, image_size=(320, 240)):
    image_bytes = get_frame(image_size=image_size, jpeg_quality=jpeg_quality)
    if image_bytes is None:
        return True # stop the task if we failed to get a frame
    # create a JpegFrame object with the image data and bytes
    frame = espp.JpegFrame(image_bytes)
    # send the frame to the RTSP server
    rtsp_server.send_frame(frame)
    time.sleep(.01)
    return False # we don't want to stop the task

async def main():
    global cap, use_display

    parser = argparse.ArgumentParser(description='RTSP Server for MJPEG streaming')
    parser.add_argument('--port', type=int, default=8554, help='Port number for RTSP server (default: 8554)')
    parser.add_argument('--use-display', action='store_true', help='Use display as video source instead of webcam')
    parser.add_argument('--camera', type=int, default=0, help='Camera index to use (default: 0)')
    parser.add_argument('--jpeg-quality', type=int, default=50, help='JPEG quality (1-100, default: 50)')
    parser.add_argument('--image-width', type=int, default=320, help='Width of the image (default: 320)')
    parser.add_argument('--image-height', type=int, default=240, help='Height of the image (default: 240)')
    args = parser.parse_args()

    # if use_display is set, we will use the display as the video source
    use_display = args.use_display
    camera_index = args.camera
    if use_display:
        print("Using display as video source")
    else:
        print(f"Using camera index {camera_index} as video source")
    jpeg_quality = args.jpeg_quality
    image_size = (args.image_width, args.image_height)

    # get the address of this machine using python's socket library
    fqdn = socket.gethostname()
    hostname = fqdn.split('.')[0]  # Get the hostname without domain
    ip_addr = socket.gethostbyname(hostname)
    port = args.port

    rtsp_server, zeroconf, info = await init_server(server_ip=ip_addr, server_port=port, hostname=hostname)

    # initialize the video capture
    if use_display:
        cap = mss.mss()  # use mss to capture the display
    else:
        # capture a frame from the default camera
        cap = cv2.VideoCapture(camera_index)

    task = espp.Task(
        lambda: task_func(rtsp_server, jpeg_quality=jpeg_quality, image_size=image_size),
        espp.Task.BaseConfig("rtsp server task")
    )
    task.start()

    print("RTSP Server started, press Ctrl+C to stop...")
    try:
        while True:
            await asyncio.sleep(1)
    except asyncio.CancelledError:
        print("\nKeyboard interrupt received, stopping...")
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received, stopping...")

    print("Releasing resources...")
    if use_display:
        cap.close()  # Close mss display capture
    else:
        cap.release()

    print("Unregistering service...")
    await zeroconf.async_unregister_service(info)

    print("Stopping zeroconf...")
    zeroconf.close()

    sys.exit(0)

if __name__ == "__main__":
    asyncio.run(main())
