# ODrive native — real fibre serial-loopback interop

The **real-tool gate** for `components/odrive_native`, mirroring how
`components/rtps` is gated against real FastDDS / ROS 2. A **genuine reference
fibre client** — the pure-python legacy `fibre` shipped in
`odriverobotics/ODrive` @ **`fw-v0.5.1`** (`Firmware/fibre/python/fibre`), the
exact implementation the espp wire codec was written against — connects to a host
build of the `odrive_native` device shim over a **PTY serial loopback**, downloads
endpoint 0, enumerates the object tree, and reads/writes endpoints.

## Pieces
- `odrive_native_interop_device.cpp` — host device shim. Opens a PTY (or a serial
  path arg), registers a small ODrive-like tree on an `OdriveNativeCore`, and runs
  the serve loop: stream bytes → `StreamDeframer` → `process_bytes` → `stream_frame`
  → write. Uses only the host-buildable `detail/` headers (plain `c++ -std=c++20`,
  no espp lib). Prints `PTY_SLAVE <path>` on startup.
- `odrive_fibre_client.py` — drives the real reference fibre library
  (`find_any("serial:<pty>")`): connect, enumerate the tree, read a value,
  write-then-read-back a value, assert. Exit 0 on success.
- `run_interop.sh` — builds the golden host tests + device shim, runs the goldens,
  fetches the reference client (sparse clone + venv with `pyserial`+`appdirs`),
  spawns the shim on a PTY, runs the real client. Prints `RESULT PASS/FAIL: <name>`
  and exits non-zero on any failure.
- `run.sh` — thin host entry (`exec run_interop.sh`); no Docker needed.

## Run locally
```sh
cd components/odrive_native/interop
./run.sh
```
Reuses an existing `odrive-ref/` clone and `.venv-odrive/` on reruns (both
git-ignored). Override the interpreter with `PYTHON=python3.x ./run.sh`.

## CI
`.github/workflows/odrive_native_interop.yml` runs `run.sh` on `ubuntu-latest`
(Python 3.11), gated PASS/FAIL, triggered by `components/odrive_native/**`,
`pc/tests/odrive_native_*`, and the workflow file.

## Wire note (found by this harness)
The endpoint **canary** (`json_crc`) is `calc_crc16(json_bytes, init=PROTOCOL_VERSION=1)`,
**not** the 0x1337 packet-CRC init. This matches the fw-v0.5.1 firmware
(`endpoints_template.j2`) and the reference client (`discovery.py`). Only the UART
*stream* framing CRC16 and endpoint 0's packet trailer use 0x1337 / PROTOCOL_VERSION
respectively. The core was corrected accordingly; see `PROTOCOL.md`.
