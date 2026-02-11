#!/usr/bin/env bash
#
# build_and_flash_pico.sh — Compile & flash the BNO055 bridge firmware
#                            to a Raspberry Pi Pico via arduino-cli
#
# Usage:
#   ./build_and_flash_pico.sh              # build + flash (auto-detect port)
#   ./build_and_flash_pico.sh --build      # build only (no flash)
#   ./build_and_flash_pico.sh --flash      # flash only (use last build)
#   ./build_and_flash_pico.sh --setup      # one-time: install board core + libs
#   ./build_and_flash_pico.sh --port /dev/ttyACM1   # specify USB port
#
# First-time setup:
#   1.  ./build_and_flash_pico.sh --setup
#   2.  Put Pico in BOOTSEL mode: hold BOOTSEL button while plugging USB
#   3.  ./build_and_flash_pico.sh
#
# After first flash the Pico has a USB CDC serial port, so subsequent
# flashes do NOT need BOOTSEL — arduino-cli auto-reboots it.
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKETCH_DIR="${SCRIPT_DIR}/raspbot_bno055_bridge"
FQBN="rp2040:rp2040:rpipico"
BOARD_URL="https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json"
BUILD_DIR="${SCRIPT_DIR}/build"

# ── Colours ───────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*" >&2; }

# ── Parse args ────────────────────────────────────────────────────────
ACTION="all"   # all | build | flash | setup
PORT=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build) ACTION="build";  shift ;;
    --flash) ACTION="flash";  shift ;;
    --setup) ACTION="setup";  shift ;;
    --port)  PORT="$2";       shift 2 ;;
    -h|--help)
      head -20 "$0" | grep '^#' | sed 's/^# \?//'
      exit 0
      ;;
    *) error "Unknown option: $1"; exit 1 ;;
  esac
done

# ── Verify arduino-cli ───────────────────────────────────────────────
if ! command -v arduino-cli &>/dev/null; then
  error "arduino-cli not found. Install it:"
  echo "  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=~/.local/bin sh"
  exit 1
fi

# ── One-time setup ────────────────────────────────────────────────────
do_setup() {
  info "Adding Earle Philhower RP2040 board index..."
  arduino-cli config init --overwrite 2>/dev/null || true
  arduino-cli config add board_manager.additional_urls "$BOARD_URL"

  info "Updating board index..."
  arduino-cli core update-index

  info "Installing rp2040:rp2040 board core (this takes a few minutes)..."
  arduino-cli core install rp2040:rp2040

  info "Installing Adafruit BNO055 library + dependencies..."
  arduino-cli lib install "Adafruit BNO055"
  arduino-cli lib install "Adafruit Unified Sensor"

  info "Setup complete! Installed:"
  arduino-cli core list | grep rp2040
  echo ""
  arduino-cli lib list | grep -i "bno055\|unified"
  echo ""
  info "You can now run:  $0"
}

# ── Build ─────────────────────────────────────────────────────────────
do_build() {
  if ! arduino-cli core list 2>/dev/null | grep -q "rp2040:rp2040"; then
    error "Board core rp2040:rp2040 not installed. Run:  $0 --setup"
    exit 1
  fi

  info "Compiling ${SKETCH_DIR} for ${FQBN}..."
  mkdir -p "$BUILD_DIR"

  arduino-cli compile \
    --fqbn "$FQBN" \
    --build-path "$BUILD_DIR" \
    --warnings default \
    "$SKETCH_DIR"

  UF2="${BUILD_DIR}/raspbot_bno055_bridge.ino.uf2"
  if [[ -f "$UF2" ]]; then
    info "Build successful: ${UF2}"
    ls -lh "$UF2"
  else
    error "Build produced no UF2 file"
    exit 1
  fi
}

# ── Auto-detect Pico port ────────────────────────────────────────────
find_pico_port() {
  # Look for a Pico CDC serial port
  local candidates=()

  # Method 1: /dev/serial/by-id (most reliable)
  if [[ -d /dev/serial/by-id ]]; then
    for link in /dev/serial/by-id/*Pico* /dev/serial/by-id/*pico*; do
      [[ -e "$link" ]] && candidates+=("$(readlink -f "$link")")
    done
  fi

  # Method 2: fall back to ttyACM devices not used by the Arduino Nano
  if [[ ${#candidates[@]} -eq 0 ]]; then
    for dev in /dev/ttyACM*; do
      [[ -e "$dev" ]] || continue
      # Skip devices identified as Arduino
      local id_path="/dev/serial/by-id"
      local is_arduino=false
      if [[ -d "$id_path" ]]; then
        for link in "$id_path"/*Arduino* "$id_path"/*arduino*; do
          [[ -e "$link" ]] && [[ "$(readlink -f "$link")" == "$dev" ]] && is_arduino=true
        done
      fi
      $is_arduino || candidates+=("$dev")
    done
  fi

  if [[ ${#candidates[@]} -eq 0 ]]; then
    return 1
  fi

  echo "${candidates[0]}"
}

# ── Check for BOOTSEL (mass storage) mode ─────────────────────────────
find_pico_uf2_mount() {
  # When Pico is in BOOTSEL mode it appears as a USB mass storage device
  local mount=""
  mount=$(lsblk -o MOUNTPOINT,LABEL 2>/dev/null | grep -i "RPI-RP2" | awk '{print $1}')
  if [[ -n "$mount" && -d "$mount" ]]; then
    echo "$mount"
    return 0
  fi

  # Also check common mount points
  for mp in /media/*/RPI-RP2 /mnt/RPI-RP2 /run/media/*/RPI-RP2; do
    if [[ -d "$mp" ]]; then
      echo "$mp"
      return 0
    fi
  done

  return 1
}

# ── Flash ─────────────────────────────────────────────────────────────
do_flash() {
  UF2="${BUILD_DIR}/raspbot_bno055_bridge.ino.uf2"
  if [[ ! -f "$UF2" ]]; then
    error "No firmware found at ${UF2}. Run build first:  $0 --build"
    exit 1
  fi

  # ── Strategy 1: Pico already has CDC serial (was previously flashed) ──
  if [[ -n "$PORT" ]]; then
    PICO_PORT="$PORT"
  else
    PICO_PORT="$(find_pico_port)" || PICO_PORT=""
  fi

  if [[ -n "$PICO_PORT" ]]; then
    info "Flashing via arduino-cli upload on ${PICO_PORT}..."
    arduino-cli upload \
      --fqbn "$FQBN" \
      --input-dir "$BUILD_DIR" \
      --port "$PICO_PORT" \
      "$SKETCH_DIR"
    info "Flash complete! Pico will reboot automatically."
    return 0
  fi

  # ── Strategy 2: Pico is in BOOTSEL mode (mass storage) ──
  UF2_MOUNT="$(find_pico_uf2_mount)" || UF2_MOUNT=""
  if [[ -n "$UF2_MOUNT" ]]; then
    info "Pico in BOOTSEL mode — copying UF2 to ${UF2_MOUNT}..."
    cp "$UF2" "${UF2_MOUNT}/"
    sync
    info "Flash complete! Pico will reboot automatically."
    return 0
  fi

  # ── No Pico found ──
  error "No Pico detected."
  echo ""
  echo "  If this is the FIRST flash (no firmware on the Pico yet):"
  echo "    1. Unplug the Pico USB"
  echo "    2. Hold the BOOTSEL button on the Pico"
  echo "    3. While holding, plug the USB cable into the Pi"
  echo "    4. Release BOOTSEL — the Pico mounts as RPI-RP2"
  echo "    5. Run:  sudo mount /dev/sda1 /mnt  (if not auto-mounted)"
  echo "    6. Run:  $0 --flash"
  echo ""
  echo "  If the Pico was already flashed (has serial port):"
  echo "    1. Make sure USB is connected"
  echo "    2. Specify the port:  $0 --port /dev/ttyACM1"
  exit 1
}

# ── Main ──────────────────────────────────────────────────────────────
case "$ACTION" in
  setup) do_setup ;;
  build) do_build ;;
  flash) do_flash ;;
  all)   do_build; do_flash ;;
esac
