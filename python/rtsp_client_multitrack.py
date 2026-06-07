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

try:
    import sounddevice as sd
except ImportError:
    sd = None


class MyListener(ServiceListener):
    def __init__(self, service_name=None):
        self.service_ip = None
        self.service_port = None
        self.found_service = False
        self.service_name = service_name

    def update_service(self, zc, type_, name):
        pass

    def remove_service(self, zc, type_, name):
        print(f"Service {name} removed")

    def add_service(self, zc, type_, name):
        if self.service_name and name != f"{self.service_name}._rtsp._tcp.local.":
            return
        info = zc.get_service_info(type_, name)
        print(f"Service {name} added, service info: {info}")
        self.service_ip = socket.inet_ntoa(info.addresses[0])
        self.service_port = info.port
        self.found_service = True


frame_queue = queue.Queue()
audio_frame_count = 0
audio_valid_frame_count = 0
audio_invalid_frame_count = 0
video_frame_count = 0
decoded_video_frame_count = 0
display_video_frames = True
expected_audio_bytes = 320
min_audio_peak = 0
audio_player = None
audio_sample_rate = 8000
audio_channels = 1


class AudioPlayer:
    def __init__(self, sample_rate=8000, channels=1, bytes_per_sample=2, block_samples=160,
                 device=None):
        self.sample_rate = sample_rate
        self.channels = channels
        self.bytes_per_sample = bytes_per_sample
        self.block_samples = block_samples
        self.device = device
        self.frame_queue = queue.Queue(maxsize=128)
        self.pending_bytes = bytearray()
        self.stream = None
        self.started = False
        self.underflow_count = 0
        self.dropped_frames = 0

    def start(self):
        if sd is None:
            print("Audio playback unavailable: install sounddevice")
            return False
        try:
            self.stream = sd.RawOutputStream(
                samplerate=self.sample_rate,
                channels=self.channels,
                dtype='int16',
                blocksize=self.block_samples,
                device=self.device,
                callback=self._callback,
            )
            self.stream.start()
            self.started = True
            print(f"Audio playback enabled at {self.sample_rate} Hz")
            return True
        except Exception as exc:
            print(f"Audio playback unavailable: {exc}")
            self.stream = None
            return False

    def stop(self):
        if self.stream is not None:
            try:
                self.stream.stop()
            finally:
                self.stream.close()
        self.stream = None
        self.started = False

    def enqueue(self, frame_bytes):
        if not self.started:
            return
        try:
            self.frame_queue.put_nowait(frame_bytes)
        except queue.Full:
            try:
                self.frame_queue.get_nowait()
            except queue.Empty:
                pass
            self.dropped_frames += 1
            try:
                self.frame_queue.put_nowait(frame_bytes)
            except queue.Full:
                self.dropped_frames += 1

    def _callback(self, outdata, frames, time_info, status):
        del frames, time_info
        if status.output_underflow:
            self.underflow_count += 1

        bytes_needed = len(outdata)
        while len(self.pending_bytes) < bytes_needed:
            try:
                self.pending_bytes.extend(self.frame_queue.get_nowait())
            except queue.Empty:
                break

        available = min(len(self.pending_bytes), bytes_needed)
        if available:
            outdata[:available] = self.pending_bytes[:available]
            del self.pending_bytes[:available]
        if available < bytes_needed:
            outdata[available:bytes_needed] = b'\x00' * (bytes_needed - available)
            self.underflow_count += 1


def as_frame_bytes(data):
    if isinstance(data, bytes):
        return data
    if isinstance(data, bytearray):
        return bytes(data)
    if isinstance(data, memoryview):
        return data.tobytes()
    return bytes(data)


def validate_audio_frame(frame_bytes):
    global min_audio_peak, expected_audio_bytes
    if len(frame_bytes) != expected_audio_bytes:
        return False, f"unexpected audio frame size: {len(frame_bytes)}"
    if len(frame_bytes) % 2 != 0:
        return False, f"audio frame size is not 16-bit aligned: {len(frame_bytes)}"
    samples = np.frombuffer(frame_bytes, dtype='<i2')
    if samples.size == 0:
        return False, "audio frame contained no samples"
    peak = int(np.max(np.abs(samples)))
    if min_audio_peak > 0 and peak < min_audio_peak:
        return False, f"audio peak too small: {peak}"
    return True, f"{samples.size} samples, peak {peak}"


def on_receive_jpeg_frame(frame):
    """Legacy JPEG frame callback."""
    global video_frame_count, decoded_video_frame_count
    buf = as_frame_bytes(frame.get_data())
    video_frame_count += 1
    decoded = cv2.imdecode(np.frombuffer(buf, dtype=np.uint8), cv2.IMREAD_COLOR)
    if decoded is not None:
        decoded_video_frame_count += 1
        if display_video_frames:
            frame_queue.put_nowait(decoded)


def on_receive_generic_frame(track_id, data):
    """Generic frame callback for any track/codec."""
    global audio_frame_count, audio_valid_frame_count, audio_invalid_frame_count
    global video_frame_count, decoded_video_frame_count
    global audio_player
    frame_bytes = as_frame_bytes(data)
    if track_id == 0:
        video_frame_count += 1
        if frame_bytes.startswith(b'\xff\xd8'):
            decoded = cv2.imdecode(np.frombuffer(frame_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
            if decoded is not None:
                decoded_video_frame_count += 1
                if display_video_frames:
                    frame_queue.put_nowait(decoded)
            else:
                print(f"[Track {track_id}] JPEG frame: {len(frame_bytes)} bytes (decode failed)")
    elif track_id == 1:
        audio_frame_count += 1
        valid, detail = validate_audio_frame(frame_bytes)
        if valid:
            audio_valid_frame_count += 1
            if audio_player is not None:
                audio_player.enqueue(frame_bytes)
        else:
            audio_invalid_frame_count += 1
            print(f"[Track {track_id}] Invalid audio frame: {detail}")
        if audio_frame_count % 50 == 0:
            print(f"[Track {track_id}] Audio frames received: {audio_frame_count} ({detail})")
    else:
        print(f"[Track {track_id}] Frame: {len(frame_bytes)} bytes")


def main(argv=None):
    global audio_frame_count, audio_valid_frame_count, audio_invalid_frame_count
    global video_frame_count, decoded_video_frame_count
    global display_video_frames, expected_audio_bytes, min_audio_peak, audio_player
    global audio_sample_rate, audio_channels

    parser = argparse.ArgumentParser(description='Multi-track RTSP Client')
    parser.add_argument('--path', type=str, default='/stream',
                        help='RTSP stream path (default: /stream)')
    parser.add_argument('--service-name', type=str, default=None,
                        help='Only connect to this mDNS service name')
    parser.add_argument('--discovery-timeout', type=float, default=10.0,
                        help='Seconds to wait for mDNS discovery before failing')
    parser.add_argument('--duration', type=float, default=0.0,
                        help='Auto-stop after this many seconds (default: run until quit)')
    parser.add_argument('--headless', action='store_true',
                        help='Do not create an OpenCV window')
    parser.add_argument('--expect-audio', action='store_true',
                        help='Fail if valid audio frames are not received')
    parser.add_argument('--require-decoded-video', action='store_true',
                        help='Fail if no JPEG video frames are successfully decoded')
    parser.add_argument('--min-video-frames', type=int, default=1,
                        help='Minimum number of video frames expected before success')
    parser.add_argument('--min-audio-frames', type=int, default=1,
                        help='Minimum number of valid audio frames expected before success')
    parser.add_argument('--expected-audio-bytes', type=int, default=None,
                        help='Expected bytes per audio frame (default: derive from SDP or fall back to 320)')
    parser.add_argument('--min-audio-peak', type=int, default=0,
                        help='Minimum absolute sample peak required to treat audio as valid')
    parser.add_argument('--play-audio', dest='play_audio', action='store_true',
                        help='Play the audio track in real time')
    parser.add_argument('--no-audio-playback', dest='play_audio', action='store_false',
                        help='Disable real-time audio playback')
    parser.add_argument('--audio-device', type=int, default=None,
                        help='Optional sounddevice output device index')
    parser.add_argument('--legacy', action='store_true',
                        help='Use legacy JPEG-only mode (on_jpeg_frame callback)')
    parser.set_defaults(play_audio=None)
    args = parser.parse_args(argv)
    display_video_frames = not args.headless
    if args.play_audio is None:
        args.play_audio = not args.headless
    expected_audio_bytes = args.expected_audio_bytes
    min_audio_peak = args.min_audio_peak
    audio_frame_count = 0
    audio_valid_frame_count = 0
    audio_invalid_frame_count = 0
    video_frame_count = 0
    decoded_video_frame_count = 0
    audio_player = None
    audio_sample_rate = 8000
    audio_channels = 1

    # Discover RTSP server via mDNS
    print("Discovering RTSP service via mDNS...")
    zeroconf = Zeroconf()
    listener = MyListener(service_name=args.service_name)
    browser = ServiceBrowser(zeroconf, "_rtsp._tcp.local.", listener)
    discovery_start = time.monotonic()
    while not listener.found_service:
        if args.discovery_timeout > 0 and (time.monotonic() - discovery_start) >= args.discovery_timeout:
            print(f"Error: timed out after {args.discovery_timeout:.1f}s waiting for RTSP service discovery")
            browser.cancel()
            zeroconf.close()
            if audio_player is not None:
                audio_player.stop()
            return 1
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
            on_frame=on_receive_generic_frame,
            log_level=espp.Logger.Verbosity.info
        )

    rtsp_client = espp.RtspClient(client_config)
    ec = espp.Error()

    # Connect and setup
    print("Connecting...")
    rtsp_client.connect(ec)
    if ec:
        print(f"Error connecting: {ec}")
        return 1

    print("Describing...")
    rtsp_client.describe(ec)
    if ec:
        print(f"Error describing: {ec}")
        return 1

    tracks = rtsp_client.tracks()
    audio_track = next((track for track in tracks if track.media_type.lower() == 'audio'), None)
    if audio_track is not None:
        audio_sample_rate = audio_track.clock_rate or audio_sample_rate
        audio_channels = audio_track.channels or audio_channels
        if args.expected_audio_bytes is None:
            expected_audio_bytes = max(2 * audio_channels,
                                       (audio_sample_rate * audio_channels * 2) // 50)
        print(f"Discovered audio track: PT={audio_track.payload_type}, "
              f"{audio_track.encoding_name}/{audio_sample_rate}/{audio_channels}, "
              f"expecting {expected_audio_bytes} bytes per frame")
    elif expected_audio_bytes is None:
        expected_audio_bytes = 320

    if args.play_audio and audio_track is not None:
        audio_player = AudioPlayer(
            sample_rate=audio_sample_rate,
            channels=audio_channels,
            block_samples=max(1, expected_audio_bytes // max(1, 2 * audio_channels)),
            device=args.audio_device,
        )
        if not audio_player.start():
            audio_player = None

    print("Setting up...")
    rtsp_client.setup(ec)
    if ec:
        print(f"Error setting up: {ec}")
        return 1

    print("Playing...")
    rtsp_client.play(ec)
    if ec:
        print(f"Error playing: {ec}")
        return 1

    window_name = 'RTSP Multi-Track Client'
    if not args.headless:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(window_name, cv2.WND_PROP_TOPMOST, 1)

    start_time = time.time()
    print("Receiving frames... Press 'q' to quit.\n")
    try:
        while True:
            try:
                frame = frame_queue.get_nowait()
                if not args.headless:
                    cv2.imshow(window_name, frame)
            except queue.Empty:
                pass
            if args.duration > 0 and (time.time() - start_time) >= args.duration:
                break
            if not args.headless:
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
            else:
                time.sleep(0.01)
    except KeyboardInterrupt:
        pass

    elapsed = time.time() - start_time
    fps = video_frame_count / elapsed if elapsed > 0 else 0
    print(f"\nVideo: {video_frame_count} frames in {elapsed:.1f}s ({fps:.1f} FPS)")
    print(f"Decoded video frames: {decoded_video_frame_count}")
    print(f"Audio: {audio_frame_count} frames received, {audio_valid_frame_count} valid, "
          f"{audio_invalid_frame_count} invalid")

    rtsp_client.teardown(ec)
    browser.cancel()
    zeroconf.close()
    if audio_player is not None:
        audio_player.stop()
        print(f"Audio playback underflows: {audio_player.underflow_count}, "
              f"dropped frames: {audio_player.dropped_frames}")

    failures = []
    if video_frame_count < args.min_video_frames:
        failures.append(
            f"expected at least {args.min_video_frames} video frames, got {video_frame_count}"
        )
    if args.require_decoded_video and decoded_video_frame_count == 0:
        failures.append("expected at least one decoded JPEG video frame")
    if args.expect_audio and audio_valid_frame_count < args.min_audio_frames:
        failures.append(
            f"expected at least {args.min_audio_frames} valid audio frames, "
            f"got {audio_valid_frame_count}"
        )
    if audio_invalid_frame_count > 0:
        failures.append(f"received {audio_invalid_frame_count} invalid audio frames")

    if failures:
        for failure in failures:
            print(f"[FAIL] {failure}")
        return 1

    print("[PASS] Multitrack RTSP client validation succeeded")
    return 0


if __name__ == "__main__":
    sys.exit(main())
