#!/usr/bin/env bash
# ──────────────────────────────────────────────────────────────
# flash.sh — Compile and flash the Raspbot Arduino Nano RP2040
#             Connect firmware directly from the Raspberry Pi.
#
# Usage:
#   ./flash.sh              # compile + upload to /dev/ttyACM0
#   ./flash.sh --port /dev/ttyACM1   # use a different port
#   ./flash.sh --compile-only        # compile without uploading
#   ./flash.sh --verify              # upload + verify flash
# ──────────────────────────────────────────────────────────────
set -euo pipefail

# ── Defaults ──────────────────────────────────────────────────
FQBN="arduino:mbed_nano:nanorp2040connect"
CORE="arduino:mbed_nano"
PORT="/dev/ttyACM0"
SKETCH_DIR="$(cd "$(dirname "$0")/raspbot_imu_bridge" && pwd)"
COMPILE_ONLY=false
VERIFY=false

# ── Colours ───────────────────────────────────────────────────
RED='\033[0;31m'
GRN='\033[0;32m'
YLW='\033[1;33m'
CYN='\033[0;36m'
RST='\033[0m'

info()  { echo -e "${CYN}[INFO]${RST}  $*"; }
warn()  { echo -e "${YLW}[WARN]${RST}  $*"; }
ok()    { echo -e "${GRN}[ OK ]${RST}  $*"; }
fail()  { echo -e "${RED}[FAIL]${RST}  $*"; exit 1; }

# ── Parse arguments ───────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --port|-p)
            PORT="$2"; shift 2 ;;
        --compile-only|-c)
            COMPILE_ONLY=true; shift ;;
        --verify|-v)
            VERIFY=true; shift ;;
        --help|-h)
            echo "Usage: $0 [--port PORT] [--compile-only] [--verify]"
            echo ""
            echo "Options:"
            echo "  --port, -p PORT     Serial port (default: /dev/ttyACM0)"
            echo "  --compile-only, -c  Compile without uploading"
            echo "  --verify, -v        Verify flash after upload"
            echo "  --help, -h          Show this help"
            exit 0 ;;
        *)
            fail "Unknown argument: $1 (use --help)" ;;
    esac
done

# ── Check prerequisites ──────────────────────────────────────
if ! command -v arduino-cli &>/dev/null; then
    fail "arduino-cli not found. Install it:\n  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh"
fi

info "arduino-cli $(arduino-cli version | head -1)"
info "Sketch:  $SKETCH_DIR"
info "FQBN:    $FQBN"

# ── Ensure the core is installed ──────────────────────────────
if ! arduino-cli core list 2>/dev/null | grep -q "$CORE"; then
    warn "Core '$CORE' not installed — installing now..."
    arduino-cli core install "$CORE"
    ok "Core installed"
else
    info "Core '$CORE' already installed"
fi

# ── Compile ───────────────────────────────────────────────────
info "Compiling..."
COMPILE_START=$(date +%s)

if ! arduino-cli compile --fqbn "$FQBN" "$SKETCH_DIR" 2>&1; then
    fail "Compilation failed"
fi

COMPILE_END=$(date +%s)
ok "Compilation succeeded ($((COMPILE_END - COMPILE_START))s)"

if $COMPILE_ONLY; then
    ok "Compile-only mode — skipping upload."
    exit 0
fi

# ── Check port exists ─────────────────────────────────────────
if [[ ! -e "$PORT" ]]; then
    fail "Serial port $PORT not found. Is the Arduino connected?\n  Available ports: $(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo 'none')"
fi

info "Port:    $PORT"

# ── Stop any service using the port (e.g. the ROS 2 node) ────
# The IMU serial node holds the port open; uploading will fail
# unless we release it first.  We try a gentle approach.
ROS_NODES_STOPPED=()

# Check if the imu_serial node is running and occupying the port
if command -v fuser &>/dev/null; then
    PIDS=$(fuser "$PORT" 2>/dev/null || true)
    if [[ -n "$PIDS" ]]; then
        warn "Port $PORT is in use by PID(s): $PIDS"
        # Try to find if any are ROS 2 nodes so we can give a useful message
        for pid in $PIDS; do
            CMDLINE=$(cat "/proc/$pid/cmdline" 2>/dev/null | tr '\0' ' ' || true)
            if echo "$CMDLINE" | grep -qi "ros\|imu_serial"; then
                warn "  PID $pid looks like a ROS 2 node: $CMDLINE"
            fi
        done
        echo ""
        read -r -p "Kill process(es) using $PORT to proceed? [Y/n] " REPLY
        REPLY=${REPLY:-Y}
        if [[ "$REPLY" =~ ^[Yy]$ ]]; then
            for pid in $PIDS; do
                info "Sending SIGTERM to $pid..."
                kill "$pid" 2>/dev/null || true
            done
            # Wait for port to be released
            for i in $(seq 1 10); do
                if ! fuser "$PORT" &>/dev/null; then
                    break
                fi
                sleep 0.5
            done
            if fuser "$PORT" &>/dev/null; then
                warn "Port still in use — trying SIGKILL..."
                fuser -k "$PORT" 2>/dev/null || true
                sleep 1
            fi
            ok "Port released"
        else
            fail "Cannot upload while port is in use. Stop the IMU node first."
        fi
    fi
fi

# ── Upload ────────────────────────────────────────────────────
UPLOAD_ARGS=(--fqbn "$FQBN" --port "$PORT")
if $VERIFY; then
    UPLOAD_ARGS+=(--verify)
fi

info "Uploading to $PORT..."

if ! arduino-cli upload "${UPLOAD_ARGS[@]}" "$SKETCH_DIR" 2>&1; then
    fail "Upload failed"
fi

ok "Firmware flashed successfully!"

# ── Wait for the board to re-enumerate ────────────────────────
info "Waiting for board to re-enumerate on $PORT..."
for i in $(seq 1 20); do
    if [[ -e "$PORT" ]]; then
        ok "Board is back on $PORT"
        break
    fi
    sleep 0.5
done

if [[ ! -e "$PORT" ]]; then
    warn "Board did not re-appear on $PORT within 10s."
    warn "Check: ls /dev/ttyACM*"
fi

# ── Quick sanity check: read a few lines ──────────────────────
info "Reading first few lines from $PORT (3s timeout)..."
echo ""
timeout 3 cat "$PORT" 2>/dev/null | head -5 || true
echo ""

ok "Done. You can restart the ROS 2 bringup now."
