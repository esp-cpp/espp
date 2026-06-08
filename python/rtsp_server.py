"""Audio-enabled MJPEG RTSP server convenience wrapper.

This preserves the original `rtsp_server.py` camera / `--use-display` workflow
while routing it through the multitrack RTSP server so the stream includes audio
by default. The camera path uses live microphone capture; display capture keeps
the synthetic tone unless `--audio-source microphone` is selected.
"""

import asyncio
import sys

from rtsp_server_multitrack import main as multitrack_main


def build_argv(argv):
    forwarded = list(argv)

    if "--codec" not in forwarded:
        forwarded.extend(["--codec", "mjpeg"])
    if "--path" not in forwarded:
        forwarded.extend(["--path", "/mjpeg/1"])
    if "--service-name" not in forwarded:
        forwarded.extend(["--service-name", "python rtsp server"])
    if "--audio" not in forwarded and "--no-audio" not in forwarded:
        forwarded.append("--audio")
    return forwarded


if __name__ == "__main__":
    sys.exit(asyncio.run(multitrack_main(build_argv(sys.argv[1:]))))
