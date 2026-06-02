"""Automated end-to-end multitrack RTSP tests for video + audio paths.

Runs the multitrack RTSP server and client in subprocesses using synthetic video
so the test can execute without a camera or display. Each case validates that
the client receives both video and audio frames.
"""

import subprocess
import sys
import time
import uuid


TEST_CASES = [
    {
        "name": "Default MJPEG wrapper + audio",
        "service_name": "python rtsp wrapper test",
        "port": 8553,
        "server_script": "rtsp_server.py",
        "client_script": "rtsp_client.py",
        "server_args": ["--video-source", "synthetic"],
        "client_args": [],
    },
    {
        "name": "MJPEG default + audio",
        "service_name": "python rtsp multitrack mjpeg test",
        "port": 8554,
        "server_script": "rtsp_server_multitrack.py",
        "client_script": "rtsp_client_multitrack.py",
        "server_args": ["--video-source", "synthetic", "--codec", "mjpeg"],
        "client_args": ["--expect-audio", "--require-decoded-video"],
    },
    {
        "name": "H264 default + audio",
        "service_name": "python rtsp multitrack h264 test",
        "port": 8555,
        "server_script": "rtsp_server_multitrack.py",
        "client_script": "rtsp_client_multitrack.py",
        "server_args": ["--video-source", "synthetic", "--codec", "h264"],
        "client_args": ["--expect-audio"],
    },
]


def read_process_output(process):
    output = ""
    if process.stdout is not None:
        output = process.stdout.read()
    return output


def stop_process(process, timeout=5.0):
    if process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=timeout)


def run_case(case):
    service_name = f"{case['service_name']} {uuid.uuid4().hex[:8]}"
    common_server_args = [
        sys.executable,
        case.get("server_script", "rtsp_server_multitrack.py"),
        "--duration", "8",
        "--service-name", service_name,
        "--port", str(case["port"]),
    ]
    server = subprocess.Popen(
        common_server_args + case["server_args"],
        cwd=".",
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )

    try:
        time.sleep(1.0)
        client = subprocess.run(
            [
                sys.executable,
                case.get("client_script", "rtsp_client_multitrack.py"),
                "--service-name", service_name,
                "--duration", "4",
                "--headless",
                "--min-video-frames", "5",
                "--min-audio-frames", "10",
                "--min-audio-peak", "1000",
                *case["client_args"],
            ],
            cwd=".",
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=15,
        )
        server.wait(timeout=10)
        server_output = read_process_output(server)
        return client.returncode == 0, client.stdout, server_output
    finally:
        stop_process(server)


def main():
    overall_success = True
    for case in TEST_CASES:
        print(f"=== Running {case['name']} ===")
        success, client_output, server_output = run_case(case)
        print("--- Client output ---")
        print(client_output.rstrip())
        print("--- Server output ---")
        print(server_output.rstrip())
        print(f"=== {'PASS' if success else 'FAIL'}: {case['name']} ===\n")
        overall_success = overall_success and success

    sys.exit(0 if overall_success else 1)


if __name__ == "__main__":
    main()
