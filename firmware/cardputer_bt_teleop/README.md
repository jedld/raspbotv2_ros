# Cardputer Bluetooth Teleop Firmware

Firmware for **M5Stack Cardputer** (ESP32-S3) that controls Raspbot over BLE,
displays live telemetry, and supports remote photo capture from the robot's camera.

## Features

### Three-Panel UI (240×135 ST7789 landscape display)

| Panel | Key | Description |
|-------|-----|-------------|
| **1 – Teleop** | `1` | Drive telemetry: speed, velocity, gyro, compass, heading, roll/pitch, follow/gimbal states |
| **2 – Orient** | `2` | Graphical compass with heading needle, cardinal labels, roll/pitch/gyro readouts |
| **3 – Camera** | `3` | Remote photo capture, JPEG thumbnail display, SD card save |

Press `Tab` to cycle through panels (most reliable on the compact keyboard).

### Controls

| Key | Action |
|-----|--------|
| `W` / `S` | Drive forward / backward (hold) |
| `A` / `D` | Yaw left / right (hold) |
| `J` / `L` | Strafe left / right (hold) |
| `X` / `Space` / `Del` | Emergency stop |
| `F` | Toggle auto-follow mode |
| `G` | Toggle gimbal control mode |
| `Fn` + `,` `/` `;` `.` | Gimbal pan / tilt fine adjust |
| `Fn` + `Tab` | Center gimbal (90°, 45°) |
| `P` or `Enter` (Camera panel) | Capture photo |
| `1` / `2` / `3` | Jump to panel directly |
| `Tab` | Cycle to next panel |

### BLE Protocol

The firmware communicates with the Pi via Nordic UART Service (NUS) at MTU 185.

**Cardputer → Pi (TX notify):**

| Message | Format |
|---------|--------|
| Velocity | `CMD\|VEL\|vx\|vy\|wz\n` |
| Gimbal | `CMD\|GMB\|pan\|tilt\n` |
| Follow toggle | `CMD\|FOL\|0/1\n` |
| Gimbal mode | `CMD\|GMODE\|0/1\n` |
| Photo request | `CMD\|PHOTO\n` |
| Keepalive | `CMD\|PING\n` |

**Pi → Cardputer (RX write):**

| Message | Format |
|---------|--------|
| Telemetry | `TEL\|vx\|vy\|wz\|speed\|gyro_z\|compass\|heading\|follow\|gmode\|roll\|pitch\n` |
| Image header | `IMG\|BEGIN\|<total_bytes>\n` |
| Image data | Raw JPEG bytes in 180-byte chunks |
| Image error | `IMG\|ERROR\|<message>\n` |

### Photo Capture Flow

1. User presses `P` (or `Enter` on Camera panel)
2. Firmware sends `CMD|PHOTO\n` to Pi
3. Pi grabs latest camera JPEG, resizes to 240×135 (quality 40)
4. Pi sends `IMG|BEGIN|<size>\n`, then raw JPEG chunks (180 bytes each)
5. Firmware receives chunks into PSRAM buffer (max 64 KB)
6. On completion, displays JPEG thumbnail and saves to SD card (if available)

## Hardware

- **MCU**: ESP32-S3 (M5Stack Cardputer)
- **Display**: 240×135 ST7789V (rotation 1 = landscape)
- **Keyboard**: 56-key matrix (4×14)
- **BLE MAC**: `30:ED:A0:CA:17:B5` (this unit)
- **SD Card**: SPI on GPIO12 CS (optional, for photo save)
- **Image buffer**: 64 KB via PSRAM (`ps_malloc`)

## Pairing and Trust (one-time)

On Pi:

```bash
bluetoothctl
power on
agent NoInputNoOutput
default-agent
scan on
# find RaspbotCardputer MAC, then:
pair 30:ED:A0:CA:17:B5
trust 30:ED:A0:CA:17:B5
quit
```

> **Note:** Do NOT run `bluetoothctl connect` — the BLE connection is managed
> by the Python `bleak` bridge node. Manual connects create lingering connections
> that prevent the Cardputer from advertising.

## Flash

### Command-line (recommended)

```bash
cd ~/ros2_foxy/src/raspbot_ros2/firmware/cardputer_bt_teleop
ESP32_CORE_VERSION=2.0.17 USE_M5STACK_CORE=0 ./flash_cardputer.sh /dev/ttyACM1
```

### Arduino IDE

1. Board: `esp32 > ESP32S3 Dev Module` (core 2.0.17)
2. Libraries: `M5Cardputer` 1.1.1, `M5Unified`, `M5GFX`
3. **Remove** any `ESP32_BLE_Arduino` from `~/Arduino/libraries/` (conflicts with
   the BLE stack bundled in the ESP32 core)
4. Flash via USB

### Build notes

- ESP32 Arduino Core pinned to **2.0.17** for BLE compatibility
- `flash_cardputer.sh` automatically moves conflicting `ESP32_BLE_Arduino` during build
- Firmware uses ~85% program storage

## Pi-side ROS Node

Run alongside bringup (enabled by default):

```bash
ros2 launch raspbot_bringup bringup.launch.py
```

Or explicit launch:

```bash
ros2 launch raspbot_teleop bt_cardputer_teleop.launch.py \
  device_name:=RaspbotCardputer \
  camera_topic:=image_raw/compressed
```

Fixed MAC targeting (recommended once paired):

```bash
ros2 launch raspbot_teleop bt_cardputer_teleop.launch.py \
  device_address:=30:ED:A0:CA:17:B5
```

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| BLE won't connect | `bluetoothctl remove 30:ED:A0:CA:17:B5` then restart node |
| "failed to discover services" | Stale bonding — node auto-clears and retries |
| Cardputer not advertising | `bluetoothctl disconnect 30:ED:A0:CA:17:B5` to release lingering connection |
| Photo shows error | Check camera topic is publishing: `ros2 topic hz image_raw/compressed` |
| Panel won't switch | Use `Tab` key to cycle panels |
