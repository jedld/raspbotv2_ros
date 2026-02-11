#!/usr/bin/env bash
# ──────────────────────────────────────────────────────────────────────
# Raspbot ROS 2 bringup launcher (called by systemd)
#
# Sources the colcon workspace and runs the full bringup launch file.
# Logs go to journald (systemd handles stdout/stderr capture).
# ──────────────────────────────────────────────────────────────────────
set -eo pipefail

WORKSPACE="/home/jedld/ros2_foxy"

# Wait for networking (OLED needs an IP to display).
# systemd After=network-online.target helps, but DHCP on WiFi
# may still be in progress; give it a few extra seconds.
for i in $(seq 1 15); do
    IP=$(hostname -I 2>/dev/null | awk '{print $1}')
    if [ -n "$IP" ] && [ "$IP" != "127.0.0.1" ]; then
        echo "[raspbot] Network ready: $IP"
        break
    fi
    echo "[raspbot] Waiting for network… ($i/15)"
    sleep 2
done

# Source the workspace (sets ROS_DISTRO, AMENT_PREFIX_PATH, etc.)
# Note: colcon-generated scripts reference unset variables,
# so we temporarily allow that during sourcing.
set +u
# shellcheck disable=SC1091
source "${WORKSPACE}/install/local_setup.bash"
set -u

echo "[raspbot] Launching bringup (ROS_DISTRO=${ROS_DISTRO:-unknown})…"

exec ros2 launch raspbot_bringup bringup.launch.py
