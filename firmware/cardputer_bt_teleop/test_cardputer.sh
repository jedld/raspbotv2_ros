#!/usr/bin/env bash
set -euo pipefail

DEVICE_NAME="${1:-RaspbotCardputer}"
SCAN_SECONDS="${SCAN_SECONDS:-20}"

PORT="$(ls -1t /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | head -n1 || true)"
if [[ -n "${PORT}" ]]; then
  echo "[test] Serial device detected: ${PORT}"
else
  echo "[test] No serial device detected (this is okay if USB re-enumerating)."
fi

if command -v bluetoothctl >/dev/null 2>&1; then
  echo "[test] Powering on Bluetooth adapter..."
  bluetoothctl power on >/dev/null 2>&1 || true

  TMP_LOG="$(mktemp)"
  trap 'rm -f "${TMP_LOG}"' EXIT

  echo "[test] Scanning ${SCAN_SECONDS}s for BLE name: ${DEVICE_NAME}"
  bluetoothctl --timeout "${SCAN_SECONDS}" scan on | tee "${TMP_LOG}" >/dev/null || true

  if grep -qi "${DEVICE_NAME}" "${TMP_LOG}"; then
    echo "[test] PASS: Found BLE advertising device '${DEVICE_NAME}'."
    exit 0
  fi

  echo "[test] FAIL: Did not detect '${DEVICE_NAME}' during scan window." >&2
  echo "[test] Tip: Ensure Cardputer rebooted after flash and is running cardputer_bt_teleop.ino" >&2
  exit 2
fi

if python3 -c 'import bleak' >/dev/null 2>&1; then
  echo "[test] bluetoothctl missing; using bleak scan fallback..."
  python3 - <<'PY'
import asyncio
import sys
from bleak import BleakScanner

name = "RaspbotCardputer"

async def main():
    devices = await BleakScanner.discover(timeout=8.0)
    for d in devices:
        if d.name and name.lower() in d.name.lower():
            print(f"[test] PASS: Found BLE advertising device '{name}'.")
            return 0
    print(f"[test] FAIL: Did not detect '{name}' via bleak scan.", file=sys.stderr)
    return 2

raise SystemExit(asyncio.run(main()))
PY
  exit $?
fi

if [[ -n "${PORT}" ]]; then
  echo "[test] Bluetooth scan tools unavailable; running USB serial smoke test..."
  python3 - <<PY
import serial
import sys
port = "${PORT}"
try:
    with serial.Serial(port, 115200, timeout=1):
        pass
    print(f"[test] PASS: Serial port {port} is accessible (USB fallback test).")
except Exception as exc:
    print(f"[test] FAIL: Could not open serial port {port}: {exc}", file=sys.stderr)
    raise SystemExit(2)
PY
  exit 0
fi

echo "[test] FAIL: No Bluetooth scan tools and no serial device available." >&2
exit 2
