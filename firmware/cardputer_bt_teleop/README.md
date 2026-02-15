# Cardputer Bluetooth Teleop Firmware

Firmware for **M5Stack Cardputer** that controls Raspbot over BLE and displays live telemetry.

## Features

- BLE enabled at boot via sketch startup advertising.
- Sends teleop commands:
  - Movement (`cmd_vel`): hold W/S, A/D, J/L
  - Gimbal pan/tilt: `Fn` + `,` `/` `;` `.`
  - Toggle auto-follow: `F`
  - Toggle gimbal control mode: `G`
- Displays telemetry from Pi bridge:
 - Displays telemetry from Pi bridge:
  - Cardputer local BLE MAC
  - Speed
  - Gyroscope Z
  - Compass (from magnetometer)
  - Heading (from IMU yaw)
  - Follow/gimbal mode states

## Pairing and trust (one-time)

Pair the Cardputer and Pi once, then keep them bonded.

On Pi:

```bash
bluetoothctl
power on
agent NoInputNoOutput
default-agent
scan on
# find RaspbotCardputer MAC, then:
pair AA:BB:CC:DD:EE:FF
trust AA:BB:CC:DD:EE:FF
connect AA:BB:CC:DD:EE:FF
quit
```

## Flash

1. Open `cardputer_bt_teleop.ino` in Arduino IDE.
2. Board: `M5Stack Cardputer (ESP32-S3)`.
3. Install required libraries:
   - `M5Cardputer`
  - Use the bundled ESP32 BLE stack from the `esp32` core
4. Flash via USB (the Cardputer already connected to Pi is fine).

## Command-line flash and test (on Pi)

From this folder:

```bash
cd ~/ros2_foxy/src/raspbot_ros2/firmware/cardputer_bt_teleop
./flash_and_test_cardputer.sh
```

Scripts:

- `flash_cardputer.sh [PORT]`: installs required Arduino core/libs, compiles, uploads
- `test_cardputer.sh [BLE_NAME]`: scans BLE to verify advertising (default `RaspbotCardputer`)
- `flash_and_test_cardputer.sh [PORT]`: runs flash then test

Notes:

- Default flash uses `esp32:esp32:esp32s3` for faster setup.
- BLE compatibility default pins ESP32 core to `2.0.17` (override with `ESP32_CORE_VERSION=...`).
- If a global `~/Arduino/libraries/ESP32_BLE_Arduino` exists, flash script temporarily moves it out of the way to avoid build conflicts.
- Optional M5Stack board package mode:
  - `USE_M5STACK_CORE=1 ./flash_cardputer.sh`
- Test script prefers `bluetoothctl`, then `python bleak`, then USB-serial smoke test fallback.

## Pi-side ROS node

Run alongside bringup (enabled by default in bringup launch):

```bash
ros2 launch raspbot_bringup bringup.launch.py
```

Optional explicit launch:

```bash
ros2 launch raspbot_teleop bt_cardputer_teleop.launch.py device_name:=RaspbotCardputer
```

If you prefer fixed MAC targeting:

```bash
ros2 launch raspbot_teleop bt_cardputer_teleop.launch.py device_address:=AA:BB:CC:DD:EE:FF
```
