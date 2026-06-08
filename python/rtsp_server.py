"""Audio-enabled MJPEG RTSP server convenience wrapper.

This preserves the original `rtsp_server.py` camera / `--use-display` workflow
while routing it through the multitrack RTSP server so the stream includes audio
by default. The camera path uses live microphone capture; display capture keeps
the synthetic tone unless `--audio-source microphone` is selected.
"""

import asyncio
import sys

from rtsp_server_multitrack import main as multitrack_main


def has_option(argv, option):
    return any(arg == option or arg.startswith(f"{option}=") for arg in argv)


def build_argv(argv):
    forwarded = list(argv)

    if not has_option(forwarded, "--codec"):
        forwarded.extend(["--codec", "mjpeg"])
    if not has_option(forwarded, "--path"):
        forwarded.extend(["--path", "/mjpeg/1"])
    if not has_option(forwarded, "--service-name"):
        forwarded.extend(["--service-name", "python rtsp server"])
    if not has_option(forwarded, "--audio") and not has_option(forwarded, "--no-audio"):
        forwarded.append("--audio")
    return forwarded


if __name__ == "__main__":
    sys.exit(asyncio.run(multitrack_main(build_argv(sys.argv[1:]))))
