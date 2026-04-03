"""Multi-track RTSP server demonstrating the new multi-codec APIs.

Supports streaming video (MJPEG or H264) plus optional simulated audio.
Uses the new add_track() / send_frame(track_id, data) APIs.

Usage:
    # MJPEG video only (default, backward-compatible path)
    python rtsp_server_multitrack.py

    # MJPEG video + simulated audio track
    python rtsp_server_multitrack.py --audio

    # H264 video (simulated, uses packetizer only — no real encoder)
    python rtsp_server_multitrack.py --codec h264

    # H264 video + audio
    python rtsp_server_multitrack.py --codec h264 --audio
"""

import sys
import time
import struct
import math
import asyncio
import argparse
import socket

import mss
import cv2
import numpy as np

from zeroconf import IPVersion, ServiceInfo, Zeroconf
from simplejpeg import encode_jpeg

from support_loader import espp

cap = None
use_display = False


# --- Frame capture (same as rtsp_server.py) ---
def get_frame(image_size=(320, 240), jpeg_quality=50):
    global cap, use_display
    if cap is None:
        return None
    if use_display:
        frame = np.array(cap.grab(cap.monitors[1]))
        if frame is None:
            return None
        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)
    else:
        ret, frame = cap.read()
        if not ret:
            return None
    resized_frame = cv2.resize(frame, image_size)
    return encode_jpeg(resized_frame, quality=jpeg_quality, colorspace='BGR')


# --- Simulated audio: generate a 160-sample (20 ms @ 8 kHz) sine-tone buffer ---
audio_phase = 0.0

def get_audio_frame(frequency=440.0, sample_rate=8000, frame_samples=160):
    """Generate a PCM 16-bit mono sine wave frame."""
    global audio_phase
    buf = bytearray(frame_samples * 2)
    for i in range(frame_samples):
        sample = int(16000 * math.sin(audio_phase))
        struct.pack_into('<h', buf, i * 2, sample)
        audio_phase += 2.0 * math.pi * frequency / sample_rate
    # Wrap phase to avoid float accumulation drift
    audio_phase %= (2.0 * math.pi)
    return bytes(buf)


# --- Simulated H264: fake Annex B access unit (for packetizer testing) ---
def get_fake_h264_frame(seq):
    """Generate a fake H264 Annex B access unit with SPS, PPS, and IDR NALs."""
    sps = bytes([0x00, 0x00, 0x00, 0x01, 0x67, 0x42, 0xC0, 0x1E,
                 0xD9, 0x00, 0xA0, 0x47, 0xFE, 0xC8])
    pps = bytes([0x00, 0x00, 0x00, 0x01, 0x68, 0xCE, 0x38, 0x80])
    # Fake IDR slice with some varying data
    idr_header = bytes([0x00, 0x00, 0x00, 0x01, 0x65])
    idr_payload = bytes([(seq + i) & 0xFF for i in range(500)])
    return sps + pps + idr_header + idr_payload


async def init_server(server_ip, server_port, hostname, media_path):
    print(f"Starting RTSP Server IP: {server_ip}, Port: {server_port}, Path: {media_path}")
    rtsp_server = espp.RtspServer(espp.RtspServer.Config(
        server_address=server_ip,
        port=server_port,
        path=media_path,
        log_level=espp.Logger.Verbosity.warn
    ))

    info = ServiceInfo(
        "_rtsp._tcp.local.",
        "python rtsp multitrack._rtsp._tcp.local.",
        addresses=[socket.inet_aton(server_ip)],
        port=server_port,
        properties={"path": media_path},
        server=hostname + '.local.'
    )
    ip_version = IPVersion.V4Only
    zeroconf = Zeroconf(ip_version=ip_version)
    await zeroconf.async_register_service(info)
    print(f"Registered RTSP service: {info}")
    return rtsp_server, zeroconf, info


async def main():
    global cap, use_display

    parser = argparse.ArgumentParser(description='Multi-track RTSP Server')
    parser.add_argument('--port', type=int, default=8554)
    parser.add_argument('--codec', choices=['mjpeg', 'h264'], default='mjpeg',
                        help='Video codec (default: mjpeg)')
    parser.add_argument('--audio', action='store_true',
                        help='Add a simulated audio track')
    parser.add_argument('--use-display', action='store_true')
    parser.add_argument('--camera', type=int, default=0)
    parser.add_argument('--jpeg-quality', type=int, default=50)
    parser.add_argument('--image-width', type=int, default=320)
    parser.add_argument('--image-height', type=int, default=240)
    args = parser.parse_args()

    use_display = args.use_display
    image_size = (args.image_width, args.image_height)

    fqdn = socket.gethostname()
    hostname = fqdn.split('.')[0]
    ip_addr = socket.gethostbyname(hostname)

    media_path = "/stream"
    rtsp_server, zeroconf, info = await init_server(ip_addr, args.port, hostname, media_path)

    # ---------------------------------------------------------------
    # Configure tracks using the new multi-track API
    # ---------------------------------------------------------------
    video_track_id = 0
    audio_track_id = 1

    if args.codec == 'mjpeg':
        print("Video codec: MJPEG (track 0)")
        video_packetizer = espp.MjpegPacketizer(espp.MjpegPacketizer.Config(
            max_payload_size=1400
        ))
    else:
        print("Video codec: H264 (track 0, simulated)")
        cfg = espp.H264Packetizer.Config()
        cfg.max_payload_size = 1400
        cfg.payload_type = 96
        cfg.profile_level_id = "42C01E"
        cfg.packetization_mode = 1
        cfg.sps = [0x67, 0x42, 0xC0, 0x1E, 0xD9, 0x00, 0xA0, 0x47, 0xFE, 0xC8]
        cfg.pps = [0x68, 0xCE, 0x38, 0x80]
        video_packetizer = espp.H264Packetizer(cfg)

    rtsp_server.add_track(espp.RtspServer.TrackConfig(
        track_id=video_track_id,
        packetizer=video_packetizer
    ))

    if args.audio:
        print("Audio: simulated PCM L16 mono @ 8 kHz (track 1)")
        audio_cfg = espp.GenericPacketizer.Config()
        audio_cfg.payload_type = 97
        audio_cfg.clock_rate = 8000
        audio_cfg.encoding_name = "L16"
        audio_cfg.channels = 1
        audio_cfg.media_type = espp.MediaType.audio
        audio_packetizer = espp.GenericPacketizer(audio_cfg)
        rtsp_server.add_track(espp.RtspServer.TrackConfig(
            track_id=audio_track_id,
            packetizer=audio_packetizer
        ))

    rtsp_server.start()

    # Initialize video capture
    if use_display:
        cap = mss.mss()
    else:
        cap = cv2.VideoCapture(args.camera)

    print("RTSP Server started, press Ctrl+C to stop...")
    frame_seq = 0
    try:
        while True:
            # --- Video ---
            if args.codec == 'mjpeg':
                jpeg_bytes = get_frame(image_size=image_size, jpeg_quality=args.jpeg_quality)
                if jpeg_bytes is not None:
                    rtsp_server.send_frame(video_track_id, jpeg_bytes)
            else:
                fake_h264 = get_fake_h264_frame(frame_seq)
                rtsp_server.send_frame(video_track_id, fake_h264)
            frame_seq += 1

            # --- Audio ---
            if args.audio:
                audio_data = get_audio_frame()
                rtsp_server.send_frame(audio_track_id, audio_data)

            await asyncio.sleep(0.033)  # ~30 fps

    except (asyncio.CancelledError, KeyboardInterrupt):
        print("\nStopping...")

    if use_display:
        cap.close()
    else:
        cap.release()

    await zeroconf.async_unregister_service(info)
    zeroconf.close()
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
