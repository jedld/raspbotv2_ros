#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT="${1:-}"

if [[ -n "${PORT}" ]]; then
  "${SCRIPT_DIR}/flash_cardputer.sh" "${PORT}"
else
  "${SCRIPT_DIR}/flash_cardputer.sh"
fi

# Give USB/BLE stack a moment after reboot caused by flash.
sleep 3

"${SCRIPT_DIR}/test_cardputer.sh" "RaspbotCardputer"

echo "[flash+test] Completed successfully."
