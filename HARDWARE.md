# Raspbot V2 hardware notes

Hardware reference for the Yahboom Raspbot V2 as used by this ROS 2 workspace.

## Platform overview

| Component | Details |
|---|---|
| Compute | Raspberry Pi 5 (BCM2712), 8 GB RAM, Ubuntu 24.04, kernel 6.8.0-raspi |
| Drive base | 4 × DC motors + mecanum wheels (holonomic motion) |
| Controller board | Yahboom motor/sensor controller on I2C bus 1 (address `0x2B`) |
| AI accelerator | Hailo-8 (full, not H8L) on PCIe (`0000:01:00.0`), HailoRT 4.23.0 |
| Front camera | Pi Camera Module 3 (IMX708 wide) on CSI CAM1 |
| Rear camera | USB webcam (auto-detected, typically `/dev/video8`) |
| IMU + audio | Arduino Nano RP2040 Connect on USB serial (`/dev/ttyACM0`, 115200 baud) |
| Display | SSD1306 I2C OLED (128×32, address `0x3C`) |
| LEDs | WS2812 RGB light bar (14 LEDs), controlled via I2C controller |
| Gimbal | 2DOF pan/tilt servo mount (servo IDs 1=pan, 2=tilt) |

## Cameras

### Pi Camera Module 3 (front-facing, CSI)

- **Sensor**: Sony IMX708 (wide variant)
- **Connection**: CSI ribbon cable on CAM1 port
- **Config**: `dtoverlay=imx708` in `/boot/firmware/config.txt` (or `camera_auto_detect=1`)
- **V4L2 devices**: Claims `/dev/video0`–`/dev/video7` (rpivid, pispbe, rp1-cfe devices)
- **ROS topic**: `front_camera/compressed` (JPEG, default 640×480 @ 30 Hz)
- **Capture pipeline** (priority order):
  1. libcamera Python bindings (best quality, direct ISP)
  2. OpenCV + GStreamer `libcamerasrc` pipeline
  3. OpenCV + V4L2 (last resort)
- **ISP**: PiSP variant BCM2712_D0, requires `libpisp` and RPi `libcamera` (built from source)
- **Tuning file**: `/usr/local/share/libcamera/ipa/rpi/pisp/imx708_wide.json`

### USB webcam (rear, gimbal-mounted)

- **Connection**: USB, auto-detected when `device_index=-1`
- **Auto-detection**: Scans `/sys/class/video4linux/video*`, filters out platform/CSI devices
  (pispbe, rp1-cfe, rpivid, bcm2835, unicam, isp, codec, scaler), follows
  symlinks to verify `usb` in the device path
- **V4L2 device**: Typically `/dev/video8` (after CSI claims 0–7)
- **Backend**: V4L2 (`CAP_V4L2`) with MJPG fourcc (avoids GStreamer conflicts)
- **ROS topic**: `image_raw/compressed` (JPEG, default 640×480 @ 30 Hz)

## Hailo-8 AI accelerator

- **Device**: Full Hailo-8 (26 TOPS), not H8L
- **Interface**: PCIe Gen 3 ×1 at `0000:01:00.0`
- **Firmware**: 4.23.0
- **Runtime**: HailoRT 4.23.0 (C++ API; no Python SDK / DFC / Model Zoo installed)
- **Driver**: `/dev/hailo0`

### Loaded models

Both models share a single `VDevice` in-process (the HailoRT scheduler
multiplexes them on the hardware):

| Model | File | Size | Input | Output | Throughput |
|---|---|---|---|---|---|
| YOLOv5s person+face | `yolov5s_personface.hef` | 14 MB | 640×640×3 UINT8 NHWC | NMS_BY_SCORE (2 classes) | ~100 FPS |
| fast_depth (NYU Depth V2) | `fast_depth.hef` | 3.1 MB | 224×224×3 UINT8 NHWC | 224×224×1 FLOAT32 depth | ~2500 FPS |

**Storage**: `~/.local/share/raspbot/models/hailo8/`

**HailoRT C++ API pattern used**:
```
VDevice::create_shared() → create_infer_model(hef_path)
  → input().set_format_type(UINT8).set_format_order(NHWC)
  → output().set_format_type(FLOAT32)
  → configure() → Buffer::create() → create_bindings() → run()
```

**Important**: Only one process may claim the Hailo-8 `VDevice` at a time.
Detection and depth run in the same `hailo_detector` node to share the device.

## Arduino Nano RP2040 Connect (IMU + microphone)

- **MCU**: RP2040 + NINA-W102 (WiFi/BLE, RGB LED)
- **Connection**: USB serial at `/dev/ttyACM0`, 115200 baud
- **Firmware**: `firmware/arduino_nano_rp2040/raspbot_imu_bridge.ino` (v1.1.0)
- **Flash script**: `firmware/arduino_nano_rp2040/flash.sh` (requires `arduino-cli`)

### Sensors

| Sensor | Chip | Details |
|---|---|---|
| 6-axis IMU | LSM6DSOX | Accelerometer + gyroscope, 100 Hz output |
| PDM microphone | MP34DT06JTR | 8 kHz, unsigned 8-bit PCM via `$AUD` protocol |
| RGB LED | NINA-W102 | Status indicator (R/G/B/W/O commands) |

### Serial protocol

**Output lines** (Arduino → Pi):

| Prefix | Format | Rate |
|---|---|---|
| `$IMU` | `ax,ay,az,gx,gy,gz` (float, g / °/s) | 100 Hz |
| `$AUD` | `base64-encoded PCM chunk` | When enabled |
| `$CAL` | `start` or `done,samples,ax,ay,az,gx,gy,gz` | On calibration |
| `$INFO` | `key=value` (firmware version, etc.) | On connect |
| `$ERR` | Error description | On error |

**Input commands** (Pi → Arduino):

| Command | Action |
|---|---|
| `?` | Request `$INFO` |
| `C` | Start gyro calibration |
| `A` | Enable audio streaming |
| `a` | Disable audio streaming |
| `R` / `G` / `B` / `W` / `O` | Set RGB LED colour |
| `1` / `2` / `3` | Set IMU output rate (100/50/10 Hz) |

## Yahboom controller board (I2C)

### I2C address & protocol

| Protocol | Address | Notes |
|---|---|---|
| **pi5** (current) | `0x2B` | Per-motor commands, ultrasonics, servos, LEDs |
| legacy | `0x16` | Older kits, left/right drive only |

The workspace defaults to `auto` protocol detection (resolves to `pi5` when
address is `0x2B`).

### Register map (pi5 protocol)

| Register | Payload | Description |
|---|---|---|
| `0x01` | `[motor_id, dir, speed]` | Motor command (id: 0=FL, 1=RL, 2=FR, 3=RR; dir: 0=fwd, 1=rev; speed: 0–255) |
| `0x02` | `[servo_id, angle_deg]` | Servo command (id: 1=pan, 2=tilt) |
| `0x03` | `[state, color]` | WS2812 all LEDs on/off |
| `0x04` | `[index, state, color]` | WS2812 single LED |
| `0x05` | `[state]` | IR receiver enable |
| `0x06` | `[state]` | Buzzer enable |
| `0x07` | `[state]` | Ultrasonic enable (1=on, 0=off) |
| `0x08` | `[R, G, B]` | WS2812 all LEDs brightness/colour |
| `0x09` | `[index, R, G, B]` | WS2812 single LED brightness/colour |
| `0x1A` | (read) | Ultrasonic distance low byte (mm) |
| `0x1B` | (read) | Ultrasonic distance high byte (mm → `(H<<8)\|L`) |

## GPIO pin mapping (BOARD numbering)

These are the defaults used by this workspace. They are **BOARD** numbering, not BCM.

| Function | Pin(s) |
|---|---|
| Ultrasonic Trig | 16 |
| Ultrasonic Echo | 18 |
| IR Avoid ON | 22 |
| IR Avoid Left | 21 |
| IR Avoid Right | 19 |
| Line Track L1 | 13 |
| Line Track L2 | 15 |
| Line Track R1 | 11 |
| Line Track R2 | 7 |

> **Note**: GPIO is only used as a fallback for ultrasonic. The preferred
> backend reads distance via I2C registers `0x1A`/`0x1B`.

## ROS 2 nodes

Bringup: `ros2 launch raspbot_bringup bringup.launch.py`

| Node | Package | Description |
|---|---|---|
| `motor_driver` | `raspbot_hw` | Mecanum drive with PID heading-hold and adaptive trim |
| `ultrasonic` | `raspbot_hw` | Ultrasonic distance (I2C preferred, GPIO fallback) |
| `gpio_sensors` | `raspbot_hw` | IR avoid + line tracking |
| `opencv_camera` | `raspbot_hw` | USB webcam with auto-detection |
| `pi_camera` | `raspbot_hw` | Pi Camera Module 3 via libcamera/GStreamer |
| `camera_gimbal` | `raspbot_hw` | 2DOF servo gimbal with IMU tilt compensation |
| `lightbar` | `raspbot_hw` | WS2812 LED bar effects |
| `oled` | `raspbot_hw` | I2C OLED text display |
| `imu_serial` | `raspbot_hw` | Arduino IMU bridge + audio streaming |
| `startup_sound` | `raspbot_bringup` | Boot chime (waits for hardware ready) |
| `web_video` | `raspbot_web_video` | Web UI (port 8080) with MJPEG, controls, dashboards |
| `hailo_detector` | `raspbot_hailo_tracking` | Object detection + depth estimation + tracking + auto-follow |

## OS / device prerequisites

| Device | Typical path | Group | Notes |
|---|---|---|---|
| I2C bus | `/dev/i2c-1` | `i2c` | Controller board + OLED |
| GPIO (Pi 5) | `/dev/gpiochip*` | varies | `lgpio` backend; `RPi.GPIO` may fail |
| USB webcam | `/dev/video8` (auto) | `video` | After CSI claims video0–7 |
| Pi Camera | `/dev/video0`–`/dev/video7` | `video` | CSI devices (rpivid, pispbe, rp1-cfe) |
| Hailo-8 | `/dev/hailo0` | — | PCIe, HailoRT 4.23.0 |
| Arduino | `/dev/ttyACM0` | `dialout` | USB serial, 115200 baud |
| Audio | — | — | No ALSA needed; audio streams over serial |

## Quick sanity checks

```bash
# I2C controller present?
sudo i2cdetect -y 1
# Expect: 0x2b (controller), 0x3c (OLED)

# Hailo-8 device?
hailortcli fw-control identify
# Expect: Board Name: Hailo-8, Firmware Version: 4.23.0

# Pi Camera?
libcamera-hello --list-cameras
# Expect: imx708_wide on /base/axi/pcie@120000/rp1/i2c@80000/imx708@1a

# Arduino?
ls -l /dev/ttyACM0
# Expect: crw-rw---- ... dialout

# USB webcam?
v4l2-ctl --list-devices
# Look for USB camera entry (not pispbe/rp1-cfe/rpivid)
```

## References

- Vendor course/download hub: https://www.yahboom.net/study/RASPBOT-V2
- Vendor Python I2C driver: `Python driver library/py_install/Raspbot_Lib/Raspbot_Lib.py`
- ROS 2 hardware config: `raspbot_hw/config/raspbot_hw.yaml`
- Arduino firmware: `firmware/arduino_nano_rp2040/`
- Hailo Model Zoo (pre-compiled HEFs): https://github.com/hailo-ai/hailo_model_zoo
