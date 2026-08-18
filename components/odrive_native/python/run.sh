#!/usr/bin/env bash
# Run the espp_odrive client test suite (CRC self-test + end-to-end interop
# against the C++ device shim over a PTY).
#
# Uses ./.venv if present (created with pyserial); otherwise falls back to
# $PYTHON / python3 (which must have pyserial installed).
set -uo pipefail
cd "$(dirname "$0")"

if [ -x ".venv/bin/python" ]; then
  PY=".venv/bin/python"
else
  PY="${PYTHON:-python3}"
  if ! "$PY" -c "import serial" 2>/dev/null; then
    echo "creating .venv with pyserial..."
    "$PY" -m venv .venv \
      && .venv/bin/python -m pip install --quiet --upgrade pip pyserial \
      && PY=".venv/bin/python"
  fi
fi

echo "using python: $PY ($($PY --version 2>&1))"
exec "$PY" tests/test_odrive.py
