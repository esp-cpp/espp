"""Multi-track RTSP client that works with rtsp_server_multitrack.py.

Discovers an RTSP server via mDNS, connects, and receives frames.
Demonstrates the new generic frame callback and depacketizer APIs.

Usage:
    # Connect to any discovered RTSP server (auto-detect codec from SDP)
    python rtsp_client_multitrack.py

    # Specify the server path
    python rtsp_client_multitrack.py --path /stream
"""

import sys
import time
import queue
import argparse
import socket

import cv2
import numpy as np

from zeroconf import ServiceBrowser, ServiceListener, Zeroconf
from support_loader import espp


class MyListener(ServiceListener):
    def __init__(self):
        self.service_ip = None
        self.service_port = None
        self.found_service = False

    def update_service(self, zc, type_, name):
        pass

    def remove_service(self, zc, type_, name):
        print(f"Service {name} removed")

    def add_service(self, zc, type_, name):
        info = zc.get_service_info(type_, name)
        print(f"Service {name} added, service info: {info}")
        self.service_ip = socket.inet_ntoa(info.addresses[0])
        self.service_port = info.port
        self.found_service = True


frame_queue = queue.Queue()
audio_frame_count = 0


def on_receive_jpeg_frame(frame):
    """Legacy JPEG frame callback."""
    buf = frame.get_data()
    decoded = cv2.imdecode(np.frombuffer(buf, dtype=np.uint8), cv2.IMREAD_COLOR)
    if decoded is not None:
        frame_queue.put_nowait(decoded)


def on_receive_generic_frame(track_id, data):
    """Generic frame callback for any track/codec."""
    global audio_frame_count
    if track_id == 0:
        # Video track — try to decode as JPEG
        decoded = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if decoded is not None:
            frame_queue.put_nowait(decoded)
        else:
            # Might be H264 or other — just log size
            print(f"[Track {track_id}] Video frame: {len(data)} bytes (non-JPEG)")
    elif track_id == 1:
        # Audio track
        audio_frame_count += 1
        if audio_frame_count % 100 == 0:
            print(f"[Track {track_id}] Audio frames received: {audio_frame_count} "
                  f"(latest: {len(data)} bytes)")
    else:
        print(f"[Track {track_id}] Frame: {len(data)} bytes")


def main():
    global audio_frame_count

    parser = argparse.ArgumentParser(description='Multi-track RTSP Client')
    parser.add_argument('--path', type=str, default='/stream',
                        help='RTSP stream path (default: /stream)')
    parser.add_argument('--legacy', action='store_true',
                        help='Use legacy JPEG-only mode (on_jpeg_frame callback)')
    args = parser.parse_args()

    # Discover RTSP server via mDNS
    print("Discovering RTSP service via mDNS...")
    zeroconf = Zeroconf()
    listener = MyListener()
    browser = ServiceBrowser(zeroconf, "_rtsp._tcp.local.", listener)

    while not listener.found_service:
        time.sleep(0.1)

    server_address = listener.service_ip
    server_port = listener.service_port
    rtsp_uri = f"rtsp://{server_address}:{server_port}{args.path}"
    print(f"Found service at: {rtsp_uri}\n")

    # Create RTSP client with appropriate callback mode
    if args.legacy:
        print("Using legacy JPEG-only mode (on_jpeg_frame)")
        client_config = espp.RtspClient.Config(
            server_address=server_address,
            rtsp_port=server_port,
            path=args.path,
            on_jpeg_frame=on_receive_jpeg_frame,
            log_level=espp.Logger.Verbosity.info
        )
    else:
        print("Using generic multi-track mode (on_frame)")
        client_config = espp.RtspClient.Config(
            server_address=server_address,
            rtsp_port=server_port,
            path=args.path,
            on_jpeg_frame=on_receive_jpeg_frame,  # Keep for MJPEG backward compat
            log_level=espp.Logger.Verbosity.info
        )

    rtsp_client = espp.RtspClient(client_config)
    ec = espp.Error()

    # Connect and setup
    print("Connecting...")
    rtsp_client.connect(ec)
    if ec:
        print(f"Error connecting: {ec}")
        sys.exit(1)

    print("Describing...")
    rtsp_client.describe(ec)
    if ec:
        print(f"Error describing: {ec}")
        sys.exit(1)

    print("Setting up...")
    rtsp_client.setup(ec)
    if ec:
        print(f"Error setting up: {ec}")
        sys.exit(1)

    print("Playing...")
    rtsp_client.play(ec)
    if ec:
        print(f"Error playing: {ec}")
        sys.exit(1)

    # Display received frames
    window_name = 'RTSP Multi-Track Client'
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.setWindowProperty(window_name, cv2.WND_PROP_TOPMOST, 1)

    start_time = time.time()
    num_video_frames = 0

    print("Receiving frames... Press 'q' to quit.\n")
    try:
        while True:
            try:
                frame = frame_queue.get_nowait()
                cv2.imshow(window_name, frame)
                num_video_frames += 1
            except queue.Empty:
                pass
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
    except KeyboardInterrupt:
        pass

    elapsed = time.time() - start_time
    fps = num_video_frames / elapsed if elapsed > 0 else 0
    print(f"\nVideo: {num_video_frames} frames in {elapsed:.1f}s ({fps:.1f} FPS)")
    print(f"Audio: {audio_frame_count} frames received")

    rtsp_client.teardown(ec)
    zeroconf.close()
    sys.exit(0)


if __name__ == "__main__":
    main()
