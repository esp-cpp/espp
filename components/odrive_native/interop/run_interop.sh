#!/usr/bin/env bash
# ODrive legacy native (Fibre) serial-loopback interop test.
#
# The true real-tool gate for components/odrive_native, mirroring how
# components/rtps is gated against real FastDDS / ROS 2: a GENUINE reference fibre
# client (the pure-python legacy fibre from odriverobotics/ODrive @ fw-v0.5.1)
# connects to the host build of the odrive_native device shim over a PTY serial
# loopback, downloads endpoint 0, enumerates the endpoint tree, and reads/writes
# endpoints.
#
# Steps:
#   1. Build the golden host tests + the device shim with a plain c++ (no espp lib).
#   2. Run the golden wire-format tests (CRC8/CRC16 + frame bytes + round-trips).
#   3. Fetch the reference fibre client (shallow clone) + a venv with pyserial.
#   4. Spawn the device shim on a PTY, run the real client against it.
#
# Prints `RESULT PASS: <name>` / `RESULT FAIL: <name>` lines; exits non-zero on any
# failure. Reuses an existing clone/venv if present (fast local reruns).
set -uo pipefail

cd "$(dirname "$0")"
INTEROP_DIR="$(pwd)"
COMPONENT_DIR="$(cd .. && pwd)"
INC="$COMPONENT_DIR/include"
WORK="${TMPDIR:-/tmp}/odrive_native_interop"
mkdir -p "$WORK"

PASS=0
FAIL=0
note() { echo -e "\n===== $* ====="; }
result() { # name exit_code
  if [ "$2" -eq 0 ]; then echo "RESULT PASS: $1"; PASS=$((PASS + 1));
  else echo "RESULT FAIL: $1"; FAIL=$((FAIL + 1)); fi
}

CXX="${CXX:-c++}"

# --- 1. Build golden tests + device shim ------------------------------------
note "Build golden host tests + device shim ($CXX -std=c++20)"
build_rc=0
"$CXX" -std=c++20 -I"$INC" "$COMPONENT_DIR/test/odrive_native_host_test.cpp" \
  -o "$WORK/host_test" || build_rc=1
"$CXX" -std=c++20 -I"$INC" "$COMPONENT_DIR/test/odrive_native_stream_test.cpp" \
  -o "$WORK/stream_test" || build_rc=1
"$CXX" -std=c++20 -I"$INC" "$INTEROP_DIR/../../../pc/tests/odrive_native_golden.cpp" \
  -o "$WORK/golden" || build_rc=1
"$CXX" -std=c++20 -I"$INC" "$INTEROP_DIR/odrive_native_interop_device.cpp" \
  -o "$WORK/device" || build_rc=1
result "build" $build_rc
if [ $build_rc -ne 0 ]; then
  echo "INTEROP FAIL"; exit 1
fi

# --- 2. Golden wire-format tests (no external tool) -------------------------
note "Golden wire-format tests (packet codec)"
"$WORK/host_test"; result "packet_golden" $?
note "Golden wire-format tests (stream framing)"
"$WORK/stream_test"; result "stream_golden" $?
note "Golden wire-format tests (combined pc golden)"
"$WORK/golden"; result "wire_golden" $?

# --- 3. Reference fibre client (real-tool) ----------------------------------
note "Set up the reference fibre client (odriverobotics/ODrive @ fw-v0.5.1)"
REF_DIR="$INTEROP_DIR/odrive-ref"
FIBRE_PY="$REF_DIR/Firmware/fibre/python"
if [ ! -d "$FIBRE_PY/fibre" ]; then
  echo "cloning ODrive fw-v0.5.1 (sparse: Firmware/fibre/python only)..."
  rm -rf "$REF_DIR"
  git clone --depth 1 --branch fw-v0.5.1 --filter=blob:none --sparse \
    https://github.com/odriverobotics/ODrive.git "$REF_DIR" \
    && git -C "$REF_DIR" sparse-checkout set Firmware/fibre/python
fi
if [ ! -d "$FIBRE_PY/fibre" ]; then
  echo "reference fibre client unavailable (clone failed)"; result "fibre_client_setup" 1
  echo ""; echo "PASS=$PASS FAIL=$FAIL"; echo "INTEROP FAIL"; exit 1
fi
result "fibre_client_setup" 0

VENV="$INTEROP_DIR/.venv-odrive"
PYBIN="$VENV/bin/python"
if [ ! -x "$PYBIN" ]; then
  PY="${PYTHON:-python3}"
  echo "creating venv with $PY and installing pyserial+appdirs..."
  "$PY" -m venv "$VENV" \
    && "$PYBIN" -m pip install --quiet --upgrade pip \
    && "$PYBIN" -m pip install --quiet pyserial appdirs
fi
"$PYBIN" -c "import serial, appdirs" 2>/dev/null
venv_rc=$?
result "venv_deps" $venv_rc
if [ $venv_rc -ne 0 ]; then
  echo ""; echo "PASS=$PASS FAIL=$FAIL"; echo "INTEROP FAIL"; exit 1
fi

# --- 4. Spawn the device shim on a PTY, run the real client -----------------
note "Real fibre client <-> espp device shim (PTY serial loopback)"
DEV_OUT="$WORK/device.out"
DEV_ERR="$WORK/device.err"
: > "$DEV_OUT"
"$WORK/device" > "$DEV_OUT" 2> "$DEV_ERR" &
DEVPID=$!

PTY=""
for _ in $(seq 1 100); do
  PTY=$(grep -oE 'PTY_SLAVE .*' "$DEV_OUT" 2>/dev/null | awk '{print $2}')
  [ -n "$PTY" ] && break
  # bail early if the device died
  kill -0 "$DEVPID" 2>/dev/null || break
  sleep 0.1
done

if [ -z "$PTY" ]; then
  echo "device shim did not report a PTY slave"; cat "$DEV_ERR"
  kill "$DEVPID" 2>/dev/null
  result "real_fibre_interop" 1
else
  echo "device PTY slave = $PTY"
  # The reference fibre client enumerates candidate ports with a plain
  # os.listdir('/dev') (top-level entries only) + pyserial's comports(). On
  # macOS a PTY slave is a top-level node (/dev/ttysNNN) and is found; on
  # Linux it is nested (/dev/pts/N), which that enumeration can never see, so
  # discovery would always time out. Alias the slave to a top-level /dev
  # symlink so the UNMODIFIED reference client can discover it -- this works
  # around only the client's port-scan quirk, not anything on the wire.
  CLIENT_PORT="$PTY"
  PTY_LINK=""
  case "$PTY" in
    /dev/pts/*)
      PTY_LINK="/dev/fibre-interop-$$"
      SUDO=""
      [ "$(id -u)" -ne 0 ] && command -v sudo >/dev/null 2>&1 && SUDO="sudo"
      if $SUDO ln -sf "$PTY" "$PTY_LINK" 2>/dev/null; then
        CLIENT_PORT="$PTY_LINK"
        echo "aliased $PTY -> $PTY_LINK (Linux: fibre's port scan only sees top-level /dev entries)"
      else
        echo "WARNING: could not create $PTY_LINK; the reference client cannot"
        echo "         discover nested /dev/pts/* slaves and will likely time out"
        PTY_LINK=""
      fi
      ;;
  esac
  echo "--- device JSON descriptor ---"; sed -n 's/^\[device\] //p' "$DEV_ERR" | head -1
  "$PYBIN" "$INTEROP_DIR/odrive_fibre_client.py" "$CLIENT_PORT" \
    --fibre-path "$FIBRE_PY" --timeout 20
  client_rc=$?
  kill "$DEVPID" 2>/dev/null
  wait "$DEVPID" 2>/dev/null
  if [ -n "$PTY_LINK" ]; then
    SUDO=""
    [ "$(id -u)" -ne 0 ] && command -v sudo >/dev/null 2>&1 && SUDO="sudo"
    $SUDO rm -f "$PTY_LINK" 2>/dev/null || true
  fi
  result "real_fibre_interop" $client_rc
fi

# --- Summary ----------------------------------------------------------------
echo ""
echo "==================== SUMMARY ===================="
echo "PASS=$PASS FAIL=$FAIL"
if [ $FAIL -eq 0 ]; then echo "INTEROP PASS"; else echo "INTEROP FAIL"; fi
exit $FAIL
