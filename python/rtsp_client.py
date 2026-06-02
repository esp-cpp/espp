"""Audio-enabled MJPEG RTSP client convenience wrapper.

This preserves the original `rtsp_client.py` workflow while using the generic
multitrack client so the default camera / display path validates both video and
audio delivery.
"""

import sys

from rtsp_client_multitrack import main as multitrack_main


def build_argv(argv):
    forwarded = list(argv)
    expect_audio = True
    if "--no-audio-check" in forwarded:
        forwarded.remove("--no-audio-check")
        expect_audio = False

    if "--path" not in forwarded:
        forwarded.extend(["--path", "/mjpeg/1"])
    if "--service-name" not in forwarded:
        forwarded.extend(["--service-name", "python rtsp server"])
    if "--require-decoded-video" not in forwarded:
        forwarded.append("--require-decoded-video")
    if expect_audio and "--expect-audio" not in forwarded:
        forwarded.append("--expect-audio")
    return forwarded


if __name__ == "__main__":
    sys.exit(multitrack_main(build_argv(sys.argv[1:])))
