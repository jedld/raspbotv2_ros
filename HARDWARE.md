# Raspbot V2 hardware notes

Hardware reference for the Yahboom Raspbot V2 as used by this ROS 2 workspace.

## Platform overview

| Component | Details |
|---|---|
| Compute | Raspberry Pi 5 (BCM2712), 4 GB RAM, Ubuntu 24.04 Noble, kernel 6.8.0-raspi |
| Drive base | 4 × DC motors + mecanum wheels (holonomic motion) |
| Controller board | Yahboom motor/sensor controller on I2C bus 1 (address `0x2B`) |
| AI accelerator | Hailo-8 (full, not H8L) on PCIe (`0000:01:00.0`), HailoRT 4.23.0 |
| Front camera | Pi Camera Module 3 (IMX708 wide, PDAF autofocus) on CSI CAM1 |
| Rear camera | USB webcam (auto-detected, typically `/dev/video8`) |
| IMU (primary) | Arduino Nano RP2040 Connect on USB serial (`/dev/ttyACM0`, 115200 baud) — LSM6DSOX 6-axis + PDM mic |
| IMU (9-DOF, optional) | BNO055 via Pico RP2040 bridge on USB serial (`/dev/ttyACM1`, 115200 baud) — fused quaternion, magnetometer |
| LiDAR | YDLidar T-mini Plus (360°, 230400 baud, 0.05–12 m, 10 Hz) |
| Display | SSD1306 I2C OLED (128×32, address `0x3C`) |
| LEDs | WS2812 RGB light bar (14 LEDs), controlled via I2C controller |
| Gimbal | 2DOF pan/tilt servo mount (servo IDs 1=pan, 2=tilt), IMU tilt compensation |
| Remote controller | M5Stack Cardputer (ESP32-S3, 240×135 TFT, WiFi, 56-key keyboard) — optional |

## Cameras

### Pi Camera Module 3 (front-facing, CSI)

- **Sensor**: Sony IMX708 (wide variant)
- **Connection**: CSI ribbon cable on CAM1 port
- **Config**: `dtoverlay=imx708` in `/boot/firmware/config.txt` (or `camera_auto_detect=1`)
- **V4L2 devices**: Claims `/dev/video0`–`/dev/video7` (rpivid, pispbe, rp1-cfe devices)
- **ROS topic**: `front_camera/compressed` (JPEG, default 640×480 @ 30 Hz)
- **Autofocus (PDAF)**:
  - `af_mode`: 0=manual, 1=auto (single-shot), 2=continuous (default)
  - `af_range`: 0=normal, 1=macro, 2=full (default)
  - `af_speed`: 0=normal, 1=fast (default)
- **Capture pipeline** (priority order):
  1. libcamera Python bindings (best quality, direct ISP)
  2. OpenCV + GStreamer `libcamerasrc` pipeline (AF properties auto-set)
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
- **BNO055 support**: When a BNO055 is wired to A4/A5, the firmware additionally
  outputs `$BNO` and `$GRV` lines with fused 9-DOF orientation data

### Sensors

| Sensor | Chip | Details |
|---|---|---|
| 6-axis IMU | LSM6DSOX | Accelerometer + gyroscope, 100 Hz output |
| 9-DOF IMU (optional) | BNO055 | Fused quaternion, magnetometer, gravity (via A4/A5 I2C) |
| PDM microphone | MP34DT06JTR | 8 kHz, unsigned 8-bit PCM via `$AUD` protocol |
| RGB LED | NINA-W102 | Status indicator (R/G/B/W/O commands) |

### Serial protocol

**Output lines** (Arduino → Pi):

| Prefix | Format | Rate |
|---|---|---|
| `$IMU` | `ax,ay,az,gx,gy,gz` (float, g / °/s) | 100 Hz |
| `$BNO` | `qw,qx,qy,qz,ax,ay,az,gx,gy,gz,mx,my,mz,yaw,pitch,roll,temp,sys,gyro,accel,mag,dt,seq` | When BNO055 connected |
| `$GRV` | `gx,gy,gz` (gravity vector) | When BNO055 connected |
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
| `S` | Save BNO055 calibration to flash |
| `L` | Load BNO055 calibration from flash |
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

## BNO055 9-DOF IMU (Pico RP2040 bridge)

- **Sensor**: Bosch BNO055 (accelerometer, gyroscope, magnetometer, fused orientation)
- **Bridge MCU**: Raspberry Pi Pico (RP2040)
- **Connection**: USB serial at `/dev/ttyACM1`, 115200 baud
- **Firmware**: `firmware/pico_rp2040_bno055/raspbot_bno055_bridge/`
- **Flash script**: `firmware/pico_rp2040_bno055/build_and_flash_pico.sh`
- **Wiring**: See `firmware/pico_rp2040_bno055/WIRING.md` — GP4=SDA, GP5=SCL, 3V3 power
- **Status LED**: Optional on GP13

The BNO055 provides drift-free magnetometer-stabilised heading, fused quaternion
orientation, and gravity-compensated linear acceleration. This significantly
improves heading-hold, odometry, and gimbal tilt compensation compared to the
gyro-only LSM6DSOX.

### Serial protocol

**Output lines** (Pico → Pi):

| Prefix | Format | Rate |
|---|---|---|
| `$BNO` | 23 fields: qw,qx,qy,qz,ax,ay,az,gx,gy,gz,mx,my,mz,yaw,pitch,roll,temp,sys_cal,gyro_cal,accel_cal,mag_cal,dt_ms,seq | ~100 Hz |
| `$GRV` | `gx,gy,gz` (gravity vector in body frame) | With `$BNO` |
| `$INFO` | Firmware version and device info | On connect |

**Input commands** (Pi → Pico):

| Command | Action |
|---|---|
| `?` | Request `$INFO` |
| `C` | Reset BNO055 calibration |
| `S` | Save calibration to Pico flash |
| `L` | Load calibration from Pico flash |

### Calibration

The BNO055 auto-calibrates over time. Calibration status is reported as
sys/gyro/accel/mag (0–3 each). Full calibration (all 3's) can be saved to
Pico flash and auto-loaded on next boot.

## YDLidar T-mini Plus

- **Type**: 360° time-of-flight laser scanner
- **Range**: 0.05–12 m
- **Scan rate**: 10 Hz (360° per scan)
- **Baud rate**: 230,400
- **ROS topic**: `/scan` (`sensor_msgs/LaserScan`)
- **TF frame**: `laser_frame` (static transform from `base_link`, +0.05 m Z)
- **Config**: `ydlidar_ros2_driver/params/tmini_plus.yaml`
- **Note**: The `ydlidar_ros2_driver` package must be built separately.
  If not installed, the launch file gracefully skips the lidar nodes.

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

## M5Stack Cardputer (optional remote controller)

| Property | Value |
|---|---|
| SoC | ESP32-S3FN8 (240 MHz dual-core, 8 MB flash, WiFi 2.4 GHz, BLE 5.0) |
| Display | 1.14″ 240×135 ST7789V2 IPS TFT |
| Input | 56-key matrix keyboard |
| Battery | 120 mAh (internal) + 1400 mAh (StampS3 base) |
| Connection | WiFi (same network as Raspberry Pi) — **no Bluetooth pairing** |
| Firmware | PlatformIO (ESP32-S3 Arduino), see `firmware/m5stack_cardputer/` |

The Cardputer connects to the Pi's web server over HTTP to:
- **Stream camera** via `/stream_front_lo.mjpg` or `/stream_lo.mjpg` (240×180, JPEG q=50)
- **Send teleop** via `POST /api/cmd_vel` (WASD keyboard → linear/angular velocities)
- **Read status** via `GET /status` (ultrasonic distance, collision failsafe state)

No new software is needed on the Pi — the Cardputer reuses the existing web
server endpoints.

## ROS 2 nodes

Bringup: `ros2 launch raspbot_bringup bringup.launch.py`

| Node | Package | Description |
|---|---|---|
| `motor_driver` | `raspbot_hw` | Mecanum drive with PID heading-hold, adaptive trim, collision/cliff failsafe |
| `ultrasonic` | `raspbot_hw` | Ultrasonic distance (I2C preferred, GPIO fallback) |
| `gpio_sensors` | `raspbot_hw` | IR avoid + line tracking |
| `opencv_camera` | `raspbot_hw` | USB webcam with auto-detection |
| `pi_camera` | `raspbot_hw` | Pi Camera Module 3 via libcamera/GStreamer (PDAF autofocus) |
| `camera_gimbal` | `raspbot_hw` | 2DOF servo gimbal with IMU tilt compensation |
| `lightbar` | `raspbot_hw` | WS2812 LED bar effects (rainbow, breathing, chase) |
| `oled` | `raspbot_hw` | I2C OLED display (IP, range, velocity, custom text) |
| `imu_serial` | `raspbot_hw` | Arduino IMU bridge (LSM6DSOX + optional BNO055) + audio streaming |
| `bno055_serial` | `raspbot_hw` | Standalone BNO055 9-DOF via Pico RP2040 bridge |
| `odometry` | `raspbot_hw` | Dead-reckoning odometry + ZUPT + path recording + return-to-origin |
| `motor_id_test` | `raspbot_hw` | Utility: spin each motor individually to identify motor ID mapping |
| `ydlidar_ros2_driver_node` | `ydlidar_ros2_driver` | YDLidar T-mini Plus 360° laser scanner |
| `startup_sound` | `raspbot_bringup` | Boot chime (waits for hardware ready) |
| `keyboard_teleop` | `raspbot_teleop` | Terminal WASD teleop with mecanum strafe (J/L keys) |
| `gimbal_teleop` | `raspbot_teleop` | Terminal gimbal pan/tilt control (arrow keys / HJKL) |
| `web_video` | `raspbot_web_video` | Web UI (port 8080) with MJPEG, controls, dashboards, LiDAR, odometry |
| `hailo_detector` | `raspbot_hailo_tracking` | Object detection + depth estimation + tracking + auto-follow |

## OS / device prerequisites

| Device | Typical path | Group | Notes |
|---|---|---|---|
| I2C bus | `/dev/i2c-1` | `i2c` | Controller board + OLED |
| GPIO (Pi 5) | `/dev/gpiochip*` | varies | `lgpio` backend; auto-detects RP1 gpiochip; `RPi.GPIO` fallback |
| USB webcam | `/dev/video8` (auto) | `video` | After CSI claims video0–7 |
| Pi Camera | `/dev/video0`–`/dev/video7` | `video` | CSI devices (rpivid, pispbe, rp1-cfe) |
| Hailo-8 | `/dev/hailo0` | — | PCIe, HailoRT 4.23.0 |
| Arduino | `/dev/ttyACM0` | `dialout` | USB serial, 115200 baud (LSM6DSOX + mic) |
| Pico (BNO055) | `/dev/ttyACM1` | `dialout` | USB serial, 115200 baud (optional 9-DOF) |
| YDLidar | `/dev/ydlidar` | `dialout` | USB serial, 230400 baud (optional) |
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

# Pico BNO055 bridge? (optional)
ls -l /dev/ttyACM1
# Expect: crw-rw---- ... dialout

# YDLidar? (optional)
ls -l /dev/ydlidar 2>/dev/null || ls -l /dev/ttyUSB0
# Expect: crw-rw---- ... dialout

# USB webcam?
v4l2-ctl --list-devices
# Look for USB camera entry (not pispbe/rp1-cfe/rpivid)

# GPIO chip (Pi 5)?
ls /dev/gpiochip*
# Expect: gpiochip0 gpiochip4 (RP1 is typically gpiochip4)

# Firmware version?
sudo rpi-eeprom-update
# Check bootloader is up to date

# Thermal status?
vcgencmd get_throttled
# Expect: throttled=0x0 (0xe0008 means thermal throttling!)
vcgencmd measure_temp
# Pi 5 throttles at 85°C — active cooling is essential
```

## Motor driver features

The `motor_driver` node provides a full-featured mecanum drive controller:

| Feature | Description |
|---|---|
| Mecanum holonomic drive | 4-wheel inverse kinematics (linear.x, linear.y, angular.z) |
| PID heading-hold | Full Kp/Ki/Kd with anti-windup, uses BNO055 magnetometer-stabilised yaw (preferred) or gyro integration |
| Adaptive motor trim | Learns L/R asymmetry from sustained PID corrections (~10 s convergence) |
| Per-motor manual trim | `trim_fl`/`trim_fr`/`trim_rl`/`trim_rr` for known hardware bias |
| Lateral drift correction | Accelerometer-based sideways drift compensation, BNO055-aware |
| Collision failsafe | Ultrasonic-based forward stop/slowdown zones, runtime toggle, only blocks forward |
| Cliff failsafe | IR line-tracker edge detection, configurable sensor bitmask, only blocks forward |
| Startup kick | Brief PWM boost to overcome static friction on all wheels |
| Strafe stabilisation | Correction deadband + PWM slew rate limiting during pure strafe |
| Differential mode | Alternative 2-wheel drive mode (`drive_mode: "differential"`) |
| Per-motor ID mapping | Configurable motor ID assignment for wiring variations |
| Per-motor inversion | Flip any wheel direction without rewiring |

All parameters are runtime-tunable (175+ total across all nodes).
See `raspbot_hw/config/raspbot_hw.yaml` for the full parameter reference.

## Odometry

The `odometry` node provides dead-reckoning pose estimation with navigation features:

| Feature | Description |
|---|---|
| Dead reckoning | Integrates `cmd_vel` + IMU yaw heading |
| IMU fusion | BNO055 fused heading (preferred) or gyro-integrated yaw |
| Zero-Velocity Update (ZUPT) | Uses gravity-compensated accel + gyro EMA to suppress drift when stationary |
| Velocity scaling | Calibratable `odom_vx_scale`/`odom_vy_scale` for wheel slip compensation |
| TF broadcast | Publishes `odom` → `base_link` transform |
| Path recording | Records waypoints at configurable distance/angle intervals (max 5000) |
| Return-to-origin | Autonomously retraces recorded path in reverse with rotate-in-place + proportional steering |
| Live path publishing | Publishes recorded path for real-time visualisation |

Services: `odom/set_origin`, `odom/start_recording`, `odom/stop_recording`,
`odom/return_to_origin`, `odom/cancel_return` (all `std_srvs/Trigger`).

## Thermal considerations

The Pi 5 throttles at 85°C. Running 12+ ROS nodes, Hailo-8 inference, camera
streams, and the LiDAR generates significant heat. **Active cooling is essential**
(official Pi 5 Active Cooler or case with fan). Check thermal status with:

```bash
vcgencmd get_throttled   # 0x0 = OK, 0xe0008 = throttling
vcgencmd measure_temp    # target: <75°C under load
```

## References

- Vendor course/download hub: https://www.yahboom.net/study/RASPBOT-V2
- Vendor Python I2C driver: `Python driver library/py_install/Raspbot_Lib/Raspbot_Lib.py`
- ROS 2 hardware config: `raspbot_hw/config/raspbot_hw.yaml`
- Arduino firmware: `firmware/arduino_nano_rp2040/`
- BNO055 Pico firmware: `firmware/pico_rp2040_bno055/`
- M5Stack Cardputer firmware: `firmware/m5stack_cardputer/`
- YDLidar params: `ydlidar_ros2_driver/params/tmini_plus.yaml`
- Hailo Model Zoo (pre-compiled HEFs): https://github.com/hailo-ai/hailo_model_zoo
