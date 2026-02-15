#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKETCH_DIR="${SCRIPT_DIR}"
SKETCH_FILE="${SKETCH_DIR}/cardputer_bt_teleop.ino"

if [[ ! -f "${SKETCH_FILE}" ]]; then
  echo "[flash] Sketch not found: ${SKETCH_FILE}" >&2
  exit 1
fi

if ! command -v arduino-cli >/dev/null 2>&1; then
  echo "[flash] arduino-cli not found. Install first:" >&2
  echo "  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh" >&2
  exit 1
fi

PORT="${1:-}"
if [[ -z "${PORT}" ]]; then
  PORT="$(ls -1t /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | head -n1 || true)"
fi

if [[ -z "${PORT}" ]]; then
  echo "[flash] No serial port found. Connect Cardputer over USB and retry." >&2
  exit 1
fi

echo "[flash] Using serial port: ${PORT}"

ESP32_INDEX_URL="https://espressif.github.io/arduino-esp32/package_esp32_index.json"
M5STACK_INDEX_URL="https://static-cdn.m5stack.com/resource/arduino/package_m5stack_index.json"
USE_M5STACK_CORE="${USE_M5STACK_CORE:-0}"
ESP32_CORE_VERSION="${ESP32_CORE_VERSION:-2.0.17}"

arduino-cli config add board_manager.additional_urls "${ESP32_INDEX_URL}" >/dev/null 2>&1 || true
if [[ "${USE_M5STACK_CORE}" == "1" ]]; then
  arduino-cli config add board_manager.additional_urls "${M5STACK_INDEX_URL}" >/dev/null 2>&1 || true
fi

arduino-cli core update-index

# Use a BLE-compatible ESP32 core for ESP32_BLE_Arduino.
if ! arduino-cli core list | grep -q "^esp32:esp32[[:space:]]\+${ESP32_CORE_VERSION}[[:space:]]"; then
  arduino-cli core install "esp32:esp32@${ESP32_CORE_VERSION}"
fi

# Required libs for this sketch.
arduino-cli lib install "M5Cardputer" >/dev/null 2>&1 || true
arduino-cli lib install "M5Unified" >/dev/null 2>&1 || true
arduino-cli lib install "M5GFX" >/dev/null 2>&1 || true
arduino-cli lib install "ESP32 BLE Arduino" >/dev/null 2>&1 || true

FQBN="${FQBN:-esp32:esp32:esp32s3}"
if [[ "${USE_M5STACK_CORE}" == "1" ]]; then
  M5_FQBN="$(arduino-cli board listall | awk 'BEGIN{IGNORECASE=1} /cardputer/ && NF>=2 {print $2; exit}')"
  if [[ -n "${M5_FQBN}" ]]; then
    FQBN="${M5_FQBN}"
  fi
fi

echo "[flash] Using FQBN: ${FQBN}"

SKETCHBOOK_LIB_DIR="${HOME}/Arduino/libraries"
CONFLICT_BLE_LIB_DIR="${SKETCHBOOK_LIB_DIR}/ESP32_BLE_Arduino"
CONFLICT_BLE_LIB_BACKUP="${HOME}/.arduino15/tmp/ESP32_BLE_Arduino.__disabled_for_flash"
if [[ -d "${CONFLICT_BLE_LIB_DIR}" ]]; then
  echo "[flash] Temporarily disabling conflicting library: ${CONFLICT_BLE_LIB_DIR}"
  mkdir -p "$(dirname "${CONFLICT_BLE_LIB_BACKUP}")"
  rm -rf "${CONFLICT_BLE_LIB_BACKUP}" >/dev/null 2>&1 || true
  mv "${CONFLICT_BLE_LIB_DIR}" "${CONFLICT_BLE_LIB_BACKUP}"
fi

cleanup_conflict_lib() {
  if [[ -d "${CONFLICT_BLE_LIB_BACKUP}" ]]; then
    mv "${CONFLICT_BLE_LIB_BACKUP}" "${CONFLICT_BLE_LIB_DIR}"
  fi
}
trap cleanup_conflict_lib EXIT

echo "[flash] Compiling sketch..."
arduino-cli compile --fqbn "${FQBN}" "${SKETCH_DIR}"

echo "[flash] Uploading sketch..."
arduino-cli upload -p "${PORT}" --fqbn "${FQBN}" "${SKETCH_DIR}"

echo "[flash] Upload complete."
