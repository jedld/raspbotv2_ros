#!/usr/bin/env bash
# ──────────────────────────────────────────────────────────────────────
# sync_service.sh — Rebuild changed packages and update the systemd
#                   service so the running system matches the source.
#
# Usage:
#   ./sync_service.sh                  # rebuild all raspbot packages
#   ./sync_service.sh raspbot_hw       # rebuild only raspbot_hw
#   ./sync_service.sh --service-only   # only re-copy the unit file
# ──────────────────────────────────────────────────────────────────────
set -euo pipefail

WORKSPACE="$(cd "$(dirname "$0")/.." && pwd)"
SRC_SERVICE="$(dirname "$0")/raspbot_bringup/systemd/raspbot.service"
INSTALLED_SERVICE="/etc/systemd/system/raspbot.service"
ALL_PACKAGES=(
    raspbot_hw
    raspbot_bringup
    raspbot_teleop
    raspbot_web_video
    raspbot_hailo_tracking
)

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No colour

info()  { echo -e "${GREEN}[sync]${NC} $*"; }
warn()  { echo -e "${YELLOW}[sync]${NC} $*"; }
error() { echo -e "${RED}[sync]${NC} $*" >&2; }

# ── Parse arguments ──────────────────────────────────────────────────
SERVICE_ONLY=false
PACKAGES=()

for arg in "$@"; do
    case "$arg" in
        --service-only) SERVICE_ONLY=true ;;
        --help|-h)
            echo "Usage: $0 [--service-only] [package ...]"
            echo ""
            echo "  --service-only   Only sync the systemd unit file (skip build)"
            echo "  package ...      Rebuild only the listed packages"
            echo "                   (default: all raspbot packages)"
            exit 0
            ;;
        *) PACKAGES+=("$arg") ;;
    esac
done

if [ ${#PACKAGES[@]} -eq 0 ]; then
    PACKAGES=("${ALL_PACKAGES[@]}")
fi

# ── 1. Rebuild packages ─────────────────────────────────────────────
if [ "$SERVICE_ONLY" = false ]; then
    info "Rebuilding: ${PACKAGES[*]}"
    cd "$WORKSPACE"

    # Source the workspace so colcon can resolve dependencies
    set +u
    # shellcheck disable=SC1091
    source install/setup.bash 2>/dev/null || true
    set -u

    if ! colcon build \
            --packages-select "${PACKAGES[@]}" \
            --symlink-install \
            --allow-overriding "${PACKAGES[@]}" 2>&1; then
        error "Build failed — aborting."
        exit 1
    fi

    info "Build succeeded."
fi

# ── 2. Sync the systemd unit file if it changed ─────────────────────
if [ ! -f "$SRC_SERVICE" ]; then
    warn "Source service file not found: $SRC_SERVICE — skipping unit sync."
elif [ ! -f "$INSTALLED_SERVICE" ]; then
    warn "No installed service found — installing for the first time."
    sudo cp "$SRC_SERVICE" "$INSTALLED_SERVICE"
    sudo systemctl daemon-reload
    sudo systemctl enable raspbot.service
    info "Service installed and enabled."
elif ! diff -q "$SRC_SERVICE" "$INSTALLED_SERVICE" >/dev/null 2>&1; then
    warn "Service unit file differs — updating."
    diff --color=auto "$SRC_SERVICE" "$INSTALLED_SERVICE" || true
    sudo cp "$SRC_SERVICE" "$INSTALLED_SERVICE"
    sudo systemctl daemon-reload
    info "Service unit file updated and systemd reloaded."
else
    info "Service unit file is already in sync."
fi

# ── 3. Kill stale manual processes that might hold port 8080 ─────────
_kill_stale() {
    local pname="$1"
    # Find processes matching the name that are attached to a terminal (pts/*),
    # which means they were started manually, not by systemd.
    local pids
    pids=$(pgrep -f "$pname" -t pts 2>/dev/null || true)
    if [ -n "$pids" ]; then
        warn "Killing stale $pname processes (terminal): $pids"
        echo "$pids" | xargs kill 2>/dev/null || true
        sleep 1
    fi
}
_kill_stale "web_video_server"
_kill_stale "face_recognition"

# ── 4. Restart the service ──────────────────────────────────────────
info "Restarting raspbot.service …"
if sudo systemctl restart raspbot.service; then
    sleep 2
    if sudo systemctl is-active --quiet raspbot.service; then
        info "raspbot.service is running."
    else
        error "Service started but is not active. Check logs:"
        echo "  sudo journalctl -u raspbot -n 30 --no-pager"
        exit 1
    fi
else
    error "Failed to restart raspbot.service."
    echo "  sudo journalctl -u raspbot -n 30 --no-pager"
    exit 1
fi

info "Done ✓"
