"""Multi-track RTSP server demonstrating the new multi-codec APIs.

Supports streaming video (MJPEG or H264) plus audio by default. The camera path
captures live microphone audio, while the display and synthetic paths use a
simulated tone unless `--audio-source microphone` is selected explicitly.
Uses the new add_track() / send_frame(track_id, data) APIs.

Usage:
    # MJPEG camera video + live microphone audio (default)
    python rtsp_server_multitrack.py

    # MJPEG video only
    python rtsp_server_multitrack.py --no-audio

    # H264 video (simulated, uses packetizer only — no real encoder)
    python rtsp_server_multitrack.py --codec h264 --no-audio

    # H264 video + audio
    python rtsp_server_multitrack.py --codec h264
"""

import sys
import time
import queue
import multiprocessing as mp
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

try:
    import sounddevice as sd
except ImportError:
    sd = None

cap = None
video_source = "camera"


# --- Frame capture (same as rtsp_server.py) ---
def get_frame(image_size=(320, 240), jpeg_quality=50):
    global cap, video_source
    if cap is None:
        return None
    if video_source == "display":
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


def get_synthetic_frame(frame_seq, image_size=(320, 240), jpeg_quality=50):
    width, height = image_size
    x = np.linspace(0, 255, width, dtype=np.uint16)
    y = np.linspace(0, 255, height, dtype=np.uint16)
    x_grid = np.tile(x, (height, 1))
    y_grid = np.tile(y[:, None], (1, width))
    frame = np.zeros((height, width, 3), dtype=np.uint16)
    frame[..., 0] = (x_grid + frame_seq * 5) % 256
    frame[..., 1] = (y_grid + frame_seq * 3) % 256
    frame[..., 2] = ((x_grid // 2) + (y_grid // 2) + frame_seq * 7) % 256
    frame = frame.astype(np.uint8)
    cv2.putText(frame, f"seq:{frame_seq}", (10, height // 2), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (255, 255, 255), 2, cv2.LINE_AA)
    return encode_jpeg(frame, quality=jpeg_quality, colorspace='BGR')


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


def resample_audio_frame(frame_bytes, input_rate, output_rate):
    if input_rate == output_rate:
        return frame_bytes
    input_samples = np.frombuffer(frame_bytes, dtype='<i2')
    if input_samples.size == 0:
        return b''
    output_samples = max(1, round(input_samples.size * output_rate / input_rate))
    sample_positions = np.linspace(0, input_samples.size - 1, num=output_samples)
    resampled = np.interp(sample_positions, np.arange(input_samples.size), input_samples)
    return np.round(resampled).astype('<i2').tobytes()


def microphone_capture_worker(sample_rate, frame_samples, channels, device, frame_queue,
                              status_queue, stop_event):
    stream = None
    overflow_count = 0
    dropped_frames = 0
    try:
        if sd is None:
            raise RuntimeError("sounddevice is required for live microphone capture")
        device_info = sd.query_devices(device, "input")
        if device_info["max_input_channels"] < channels:
            raise RuntimeError(
                f"input device '{device_info['name']}' only supports "
                f"{device_info['max_input_channels']} channels"
            )
        native_sample_rate = int(round(device_info["default_samplerate"]))
        native_frame_samples = max(1, round(native_sample_rate * frame_samples / sample_rate))
        stream = sd.RawInputStream(
            samplerate=native_sample_rate,
            channels=channels,
            dtype="int16",
            blocksize=native_frame_samples,
            device=device,
        )
        stream.start()
        status_queue.put({
            "ok": True,
            "device_name": device_info["name"],
            "device_sample_rate": native_sample_rate,
        })
        while not stop_event.is_set():
            frame, overflowed = stream.read(native_frame_samples)
            if overflowed:
                overflow_count += 1
            try:
                frame_queue.put_nowait(
                    resample_audio_frame(bytes(frame), native_sample_rate, sample_rate)
                )
            except queue.Full:
                try:
                    frame_queue.get_nowait()
                except queue.Empty:
                    # The queue was drained before we could drop the oldest frame.
                    pass
                dropped_frames += 1
                try:
                    frame_queue.put_nowait(
                        resample_audio_frame(bytes(frame), native_sample_rate, sample_rate)
                    )
                except queue.Full:
                    dropped_frames += 1
        status_queue.put({
            "ok": True,
            "stopped": True,
            "overflow_count": overflow_count,
            "dropped_frames": dropped_frames,
        })
    except Exception as exc:
        try:
            status_queue.put({"ok": False, "error": str(exc)})
        except Exception:
            # If status reporting fails during teardown, preserve the original failure path.
            pass
    finally:
        if stream is not None:
            try:
                stream.stop()
            finally:
                stream.close()


class MicrophoneAudioSource:
    def __init__(self, sample_rate=8000, frame_samples=160, channels=1, device=None):
        self.sample_rate = sample_rate
        self.frame_samples = frame_samples
        self.channels = channels
        self.device = device
        self.ctx = mp.get_context("spawn")
        self.frame_queue = self.ctx.Queue(maxsize=128)
        self.status_queue = self.ctx.Queue()
        self.stop_event = self.ctx.Event()
        self.process = None
        self.dropped_frames = 0
        self.overflow_count = 0

    def start(self):
        self.stop_event.clear()
        self.process = self.ctx.Process(
            target=microphone_capture_worker,
            args=(
                self.sample_rate,
                self.frame_samples,
                self.channels,
                self.device,
                self.frame_queue,
                self.status_queue,
                self.stop_event,
            ),
            daemon=True,
        )
        self.process.start()

        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            try:
                status = self.status_queue.get(timeout=0.1)
            except queue.Empty:
                if self.process.exitcode is not None:
                    raise RuntimeError("microphone capture process exited unexpectedly")
                continue
            if status.get("ok") is False:
                raise RuntimeError(status["error"])
            if status.get("device_name"):
                return {"name": status["device_name"]}
            self._apply_status(status)
        raise RuntimeError("timed out starting microphone capture")

    def stop(self):
        self.stop_event.set()
        if self.process is not None:
            self.process.join(timeout=1.0)
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(timeout=1.0)
        self.process = None
        self._drain_status_queue()

    def get_frame(self):
        self._drain_status_queue()
        try:
            return self.frame_queue.get_nowait()
        except queue.Empty:
            return None

    def get_latest_frame(self):
        self._drain_status_queue()
        frame = self.get_frame()
        if frame is None:
            return None
        while True:
            try:
                frame = self.frame_queue.get_nowait()
            except queue.Empty:
                return frame

    def _drain_status_queue(self):
        while True:
            try:
                status = self.status_queue.get_nowait()
            except queue.Empty:
                break
            self._apply_status(status)

    def _apply_status(self, status):
        self.overflow_count += status.get("overflow_count", 0)
        self.dropped_frames += status.get("dropped_frames", 0)


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


def make_service_info(server_ip, server_port, hostname, media_path, service_name):
    return ServiceInfo(
        "_rtsp._tcp.local.",
        f"{service_name}._rtsp._tcp.local.",
        addresses=[socket.inet_aton(server_ip)],
        port=server_port,
        properties={"path": media_path},
        server=hostname + '.local.'
    )


async def init_server(server_ip, server_port, media_path):
    print(f"Starting RTSP Server IP: {server_ip}, Port: {server_port}, Path: {media_path}")
    rtsp_server = espp.RtspServer(espp.RtspServer.Config(
        server_address=server_ip,
        port=server_port,
        path=media_path,
        log_level=espp.Logger.Verbosity.warn
    ))
    ip_version = IPVersion.V4Only
    zeroconf = Zeroconf(ip_version=ip_version)
    return rtsp_server, zeroconf


async def main(argv=None):
    global cap, video_source

    parser = argparse.ArgumentParser(description='Multi-track RTSP Server')
    parser.add_argument('--port', type=int, default=8554)
    parser.add_argument('--codec', choices=['mjpeg', 'h264'], default='mjpeg',
                        help='Video codec (default: mjpeg)')
    parser.add_argument('--audio', dest='audio', action='store_true',
                        help='Include the audio track (default)')
    parser.add_argument('--no-audio', dest='audio', action='store_false',
                        help='Disable the audio track')
    parser.add_argument('--service-name', type=str, default='python rtsp multitrack',
                        help='mDNS service name prefix (default: python rtsp multitrack)')
    parser.add_argument('--duration', type=float, default=0.0,
                        help='Auto-stop after this many seconds (default: run until interrupted)')
    parser.add_argument('--path', type=str, default='/stream',
                        help='RTSP stream path (default: /stream)')
    parser.add_argument('--video-source', choices=['camera', 'display', 'synthetic'],
                        default='camera',
                        help='Video source for MJPEG mode (default: camera)')
    parser.add_argument('--audio-source', choices=['auto', 'microphone', 'synthetic'],
                        default='auto',
                        help='Audio source (default: microphone for camera, synthetic otherwise)')
    parser.add_argument('--audio-device', type=int, default=None,
                        help='Optional sounddevice input device index for microphone capture')
    parser.add_argument('--use-display', action='store_true')
    parser.add_argument('--camera', type=int, default=0)
    parser.add_argument('--jpeg-quality', type=int, default=50)
    parser.add_argument('--image-width', type=int, default=320)
    parser.add_argument('--image-height', type=int, default=240)
    parser.set_defaults(audio=True)
    args = parser.parse_args(argv)

    if args.use_display:
        args.video_source = 'display'
    video_source = args.video_source
    image_size = (args.image_width, args.image_height)
    live_audio_source = None
    resolved_audio_source = args.audio_source
    if resolved_audio_source == 'auto':
        resolved_audio_source = 'microphone' if video_source == 'camera' else 'synthetic'

    ip_addr = ""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            ip_addr = s.getsockname()[0]
            print(ip_addr)
    except OSError as exc:
        print(f"UDP IP discovery unavailable, falling back to hostname lookup: {exc}")

    fqdn = socket.gethostname()
    print(f"Fully qualified domain name: {fqdn}")
    # get the address of this machine using python's socket library
    hostname = fqdn.split('.')[0]  # Get the hostname without domain

    if not ip_addr or ip_addr.startswith("127."):
        print(f"Hostname: {hostname}")
        ip_addr = socket.gethostbyname(hostname)

    port = args.port
    print(f"Using IP address: {ip_addr}")
    print(f"Using port: {port}")
    print(f"Using service name: {args.service_name}")

    media_path = args.path
    rtsp_server, zeroconf = await init_server(ip_addr, args.port, media_path)
    info = make_service_info(ip_addr, args.port, hostname, media_path, args.service_name)

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
        if resolved_audio_source == 'microphone':
            print("Audio: live microphone PCM L16 mono @ 8 kHz (track 1)")
        else:
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

    # Initialize video capture
    if video_source == "display":
        cap = mss.mss()
    elif video_source == "camera":
        cap = cv2.VideoCapture(args.camera)
        if not cap.isOpened():
            print(f"Error: could not open camera device {args.camera}")
            zeroconf.close()
            return 1
    else:
        cap = None

    if args.audio and resolved_audio_source == 'microphone':
        live_audio_source = MicrophoneAudioSource(
            sample_rate=8000,
            frame_samples=160,
            channels=1,
            device=args.audio_device,
        )
        try:
            device_info = live_audio_source.start()
            print(f"Using microphone device: {device_info['name']}")
        except Exception as exc:
            if cap is not None:
                if video_source == "display":
                    cap.close()
                elif video_source == "camera":
                    cap.release()
            zeroconf.close()
            print(f"Error: could not start microphone capture: {exc}")
            return 1

    rtsp_server.start()

    print("RTSP Server started, press Ctrl+C to stop...")
    frame_seq = 0
    audio_frame_count = 0
    start_time = time.monotonic()
    next_video_time = start_time
    audio_period = 160 / 8000.0
    next_audio_time = start_time
    service_registered = False
    video_ready = False
    audio_ready = not args.audio
    try:
        while True:
            now = time.monotonic()
            if args.duration > 0 and (now - start_time) >= args.duration:
                break

            sent_frame = False
            if now >= next_video_time:
                if args.codec == 'mjpeg':
                    if video_source == "synthetic":
                        jpeg_bytes = get_synthetic_frame(
                            frame_seq, image_size=image_size, jpeg_quality=args.jpeg_quality
                        )
                    else:
                        jpeg_bytes = get_frame(image_size=image_size, jpeg_quality=args.jpeg_quality)
                    if jpeg_bytes is not None:
                        rtsp_server.send_frame(video_track_id, jpeg_bytes)
                        video_ready = True
                else:
                    fake_h264 = get_fake_h264_frame(frame_seq)
                    rtsp_server.send_frame(video_track_id, fake_h264)
                    video_ready = True
                frame_seq += 1
                next_video_time = max(next_video_time + 1.0 / 30.0, time.monotonic())
                sent_frame = True

            now = time.monotonic()
            if args.audio:
                if resolved_audio_source == 'microphone':
                    if now >= next_audio_time:
                        audio_data = live_audio_source.get_latest_frame()
                        if audio_data is not None:
                            rtsp_server.send_frame(audio_track_id, audio_data)
                            audio_frame_count += 1
                            audio_ready = True
                            next_audio_time = max(next_audio_time + audio_period, time.monotonic())
                            sent_frame = True
                else:
                    if now >= next_audio_time:
                        audio_data = get_audio_frame()
                        rtsp_server.send_frame(audio_track_id, audio_data)
                        audio_frame_count += 1
                        audio_ready = True
                        next_audio_time = max(next_audio_time + audio_period, time.monotonic())
                        sent_frame = True

            if not service_registered and video_ready and audio_ready:
                await zeroconf.async_register_service(info)
                print(f"Registered RTSP service: {info}")
                service_registered = True

            if sent_frame:
                await asyncio.sleep(0)
            else:
                await asyncio.sleep(0.005)

    except (asyncio.CancelledError, KeyboardInterrupt):
        print("\nStopping...")

    print(f"Sent {frame_seq} video frames and {audio_frame_count} audio frames")

    if video_source == "display" and cap is not None:
        cap.close()
    elif video_source == "camera" and cap is not None:
        cap.release()
    if live_audio_source is not None:
        live_audio_source.stop()
        print(f"Microphone capture overflows: {live_audio_source.overflow_count}, "
              f"dropped frames: {live_audio_source.dropped_frames}")

    if service_registered:
        await zeroconf.async_unregister_service(info)
    zeroconf.close()
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
