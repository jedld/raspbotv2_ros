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

# Ensure Bluetooth controller is available and powered.
if command -v bluetoothctl >/dev/null 2>&1; then
    if command -v rfkill >/dev/null 2>&1; then
        rfkill unblock bluetooth >/dev/null 2>&1 || true
    fi

    for i in $(seq 1 8); do
        bluetoothctl power on >/dev/null 2>&1 || true
        bluetoothctl agent NoInputNoOutput >/dev/null 2>&1 || true
        bluetoothctl default-agent >/dev/null 2>&1 || true

        if bluetoothctl show 2>/dev/null | grep -q "Powered: yes"; then
            echo "[raspbot] Bluetooth ready (powered=yes)"
            break
        fi

        echo "[raspbot] Waiting for Bluetooth adapter… ($i/8)"
        sleep 1
    done

    if ! bluetoothctl show 2>/dev/null | grep -q "Powered: yes"; then
        echo "[raspbot] WARNING: Bluetooth adapter not powered yet; continuing bringup"
    fi

    CARDPUTER_BT_NAME="${CARDPUTER_BT_NAME:-RaspbotCardputer}"
    CARDPUTER_BT_ADDRESS="${CARDPUTER_BT_ADDRESS:-}"
    CARDPUTER_BT_PRECONNECT="${CARDPUTER_BT_PRECONNECT:-0}"

    if [ -z "${CARDPUTER_BT_ADDRESS}" ] && [ -n "${CARDPUTER_BT_NAME}" ]; then
        CARDPUTER_BT_ADDRESS="$({ bluetoothctl devices Paired 2>/dev/null || true; } | awk -v name="${CARDPUTER_BT_NAME}" '$1=="Device"{mac=$2; $1=""; $2=""; sub(/^  */, "", $0); if ($0==name) { print mac; exit }}')"
    fi

    if [ -n "${CARDPUTER_BT_ADDRESS}" ]; then
        echo "[raspbot] Attempting Cardputer reconnect: ${CARDPUTER_BT_ADDRESS}"
        bluetoothctl trust "${CARDPUTER_BT_ADDRESS}" >/dev/null 2>&1 || true
        if [ "${CARDPUTER_BT_PRECONNECT}" = "1" ]; then
            for i in $(seq 1 6); do
                bluetoothctl connect "${CARDPUTER_BT_ADDRESS}" >/dev/null 2>&1 || true
                if bluetoothctl info "${CARDPUTER_BT_ADDRESS}" 2>/dev/null | grep -q "Connected: yes"; then
                    echo "[raspbot] Cardputer connected: ${CARDPUTER_BT_ADDRESS}"
                    break
                fi
                echo "[raspbot] Waiting for Cardputer reconnect… ($i/6)"
                sleep 1
            done

            if ! bluetoothctl info "${CARDPUTER_BT_ADDRESS}" 2>/dev/null | grep -q "Connected: yes"; then
                echo "[raspbot] Cardputer not connected yet; BLE teleop node will keep scanning"
            fi
        else
            echo "[raspbot] Cardputer trust ready; BLE teleop node will perform GATT connection"
        fi
    fi
fi

exec ros2 launch raspbot_bringup bringup.launch.py
