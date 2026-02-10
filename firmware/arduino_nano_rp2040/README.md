# Arduino Nano RP2040 Connect — Raspbot Sensor Bridge

Firmware that turns the Arduino Nano RP2040 Connect into a USB sensor module
for the Raspbot V2, streaming IMU + microphone data to the Raspberry Pi.

## What it uses from the board

| Feature | Chip | Used for |
|---|---|---|
| **6-axis IMU** | LSM6DSOX | Accelerometer (±16g) + gyroscope (±2000°/s) → `sensor_msgs/Imu` |
| **Temperature** | LSM6DSOX (embedded) | Die temperature (~board ambient) |
| **Microphone** | MP34DT06JTR (PDM) | Sound level (RMS) for clap detection / noise monitoring |
| **RGB LED** | via NINA-W102 | Status indicator (calibrating = orange, host can set colour) |

### Features NOT used (and why)

| Feature | Why not |
|---|---|
| **WiFi** (u-blox NINA-W102) | Pi already has WiFi; USB serial is lower-latency and simpler |
| **Bluetooth/BLE** | Same — Pi has BT; USB is more reliable for real-time data |
| **Crypto chip** (ATECC608A) | No security need for an internal sensor bus |
| **GPIO / ADC / PWM pins** | Available if you want to add extra sensors later |

## Wiring

Just a USB cable:

```
Arduino Nano RP2040 Connect  ←── USB Micro-B cable ──→  Raspberry Pi USB port
```

No other wires needed. The Pi sees it as `/dev/ttyACM0` (or `/dev/ttyACM1`).

## Building & flashing

### Option A: Arduino IDE

1. **Board Manager** → install **"Arduino Mbed OS Nano Boards"**
2. **Board** → select **Arduino Nano RP2040 Connect**
3. The required libraries are bundled with the board package:
   - `Arduino_LSM6DSOX`
   - `PDM`
   - `WiFiNINA`
4. Open `raspbot_imu_bridge.ino`
5. Upload

### Option B: Arduino CLI

```bash
# Install core
arduino-cli core install arduino:mbed_nano

# Compile
arduino-cli compile --fqbn arduino:mbed_nano:nanorp2040connect \
    firmware/arduino_nano_rp2040/raspbot_imu_bridge/

# Upload (adjust port)
arduino-cli upload --fqbn arduino:mbed_nano:nanorp2040connect \
    --port /dev/ttyACM0 \
    firmware/arduino_nano_rp2040/raspbot_imu_bridge/
```

## Serial protocol

Baud rate: **115200**

### Output lines (Arduino → Pi)

| Prefix | Format | When |
|---|---|---|
| `$IMU` | `$IMU,ax,ay,az,gx,gy,gz,temp,mic,ms` | Every cycle (default 100 Hz) |
| `$INFO` | `$INFO,nano_rp2040,<hz>,<version>` | On startup and on `?` command |
| `$CAL` | `$CAL,start` / `$CAL,done,<n>,<ox>,<oy>,<oz>` | During gyro calibration |
| `$ERR` | `$ERR,<message>` | On init failure |

#### `$IMU` field details

| Field | Unit | Type | Description |
|---|---|---|---|
| ax, ay, az | g | float | Accelerometer (1 g ≈ 9.81 m/s²) |
| gx, gy, gz | °/s | float | Gyroscope (calibration offsets subtracted) |
| temp | °C | float | LSM6DSOX die temperature |
| mic | — | int | Microphone RMS level (0–32767) |
| ms | ms | int | Arduino `millis()` timestamp |

### Input commands (Pi → Arduino)

| Char | Action |
|---|---|
| `?` | Request `$INFO` |
| `C` | Calibrate gyro (hold robot still for 2 s) |
| `R` | RGB LED → red |
| `G` | RGB LED → green |
| `B` | RGB LED → blue |
| `W` | RGB LED → white |
| `O` | RGB LED → off |
| `1` | Set output rate to 50 Hz |
| `2` | Set output rate to 100 Hz (default) |
| `3` | Set output rate to 200 Hz |

## Quick test from Pi

```bash
# Check the device appeared
ls -l /dev/ttyACM*

# Read raw data
cat /dev/ttyACM0

# Or with screen
screen /dev/ttyACM0 115200

# Send a command (e.g. request info)
echo -n '?' > /dev/ttyACM0
```

## Mounting

Mount the Arduino **rigidly** on the robot chassis — any flex or vibration
isolation will corrupt the gyroscope readings. The axes orientation matters
for the ROS 2 node; note which way the board faces when you mount it.

Typical mounting: board flat on the chassis, USB connector facing rear.
In that orientation with the components facing up:
- X = forward
- Y = left  
- Z = up

If you mount it differently, the ROS 2 driver node has axis remap parameters.
