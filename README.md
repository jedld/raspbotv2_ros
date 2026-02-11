# Raspbot ROS 2 packages

ROS 2 workspace for the **Yahboom Raspbot V2** on a **Raspberry Pi 5** (Ubuntu 24.04).

## Packages

| Package | Description |
|---|---|
| `raspbot_hw` | I2C motor driver, GPIO sensors, USB camera (auto-detect), Pi Camera Module 3, 2DOF gimbal, WS2812 light-bar, OLED, IMU serial bridge (Arduino Nano RP2040 Connect) |
| `raspbot_bringup` | Unified bringup launch + startup sound |
| `raspbot_web_video` | Web UI on port 8080 — MJPEG streams, drive (WASD), gimbal, detection overlay, auto-follow controls, snapshots, IMU dashboard, microphone streaming, depth map |
| `raspbot_hailo_tracking` | Hailo-8 object detection (YOLOv5s person+face), gimbal person tracking, PID auto-follow with mecanum strafing, IMU feedback & obstacle avoidance, monocular depth estimation (fast_depth) |

## Quick start

```bash
# Build
colcon build --packages-select raspbot_hw raspbot_bringup raspbot_web_video raspbot_hailo_tracking

# Full bringup (all features enabled by default)
ros2 launch raspbot_bringup bringup.launch.py
```

Then open **http://\<pi-ip\>:8080/** in a browser.

## Auto-start on boot

A systemd service launches the full bringup automatically when the Pi powers on.
The OLED will display the IP address as soon as the network is ready.

```bash
# Install the service (already done if you followed initial setup)
sudo cp src/raspbot_ros2/raspbot_bringup/systemd/raspbot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable raspbot.service

# Manual control
sudo systemctl start raspbot          # start now
sudo systemctl stop raspbot           # stop
sudo systemctl restart raspbot        # restart
sudo journalctl -u raspbot -f         # live logs
```

## Launch arguments

All arguments have sensible defaults and can be toggled individually:

```bash
# Disable specific subsystems
ros2 launch raspbot_bringup bringup.launch.py enable_hailo:=false
ros2 launch raspbot_bringup bringup.launch.py enable_web_video:=false
ros2 launch raspbot_bringup bringup.launch.py enable_lightbar:=false
ros2 launch raspbot_bringup bringup.launch.py enable_front_camera:=false
ros2 launch raspbot_bringup bringup.launch.py enable_imu:=false

# Disable depth estimation (empty path)
ros2 launch raspbot_bringup bringup.launch.py depth_hef_path:=

# Custom Hailo model
ros2 launch raspbot_bringup bringup.launch.py \
  hailo_hef_path:=/path/to/model.hef \
  hailo_labels_path:=/path/to/labels.txt
```

<details>
<summary>Full argument list</summary>

| Argument | Default | Description |
|---|---|---|
| `enable_motors` | `true` | Mecanum motor driver |
| `enable_ultrasonic` | `true` | Ultrasonic range sensor |
| `enable_gpio_sensors` | `true` | IR avoid + line tracking |
| `enable_camera` | `true` | USB webcam |
| `enable_front_camera` | `true` | Pi Camera Module 3 (CSI) |
| `enable_gimbal` | `true` | 2DOF pan/tilt servos |
| `enable_lightbar` | `true` | WS2812 RGB LED bar |
| `enable_oled` | `true` | I2C OLED display |
| `enable_imu` | `true` | Arduino IMU + audio bridge |
| `enable_web_video` | `true` | Web UI (port 8080) |
| `enable_hailo` | `true` | Hailo-8 detection + depth |
| `play_startup_sound` | `true` | Play sound on boot |
| `hailo_hef_path` | `~/.local/share/raspbot/models/hailo8/yolov5s_personface.hef` | Detection model |
| `hailo_labels_path` | `~/.local/share/raspbot/models/hailo8/personface.labels` | Detection labels |
| `depth_hef_path` | `~/.local/share/raspbot/models/hailo8/fast_depth.hef` | Depth model (empty = disabled) |
| `hailo_pan_sign` | `-1` | Gimbal pan direction |
| `hailo_tilt_sign` | `-1` | Gimbal tilt direction |

</details>

## ROS 2 topics

### Sensors & cameras

| Topic | Type | Source |
|---|---|---|
| `image_raw/compressed` | `CompressedImage` | USB webcam (auto-detected) |
| `front_camera/compressed` | `CompressedImage` | Pi Camera Module 3 (IMX708) |
| `ultrasonic/range` | `Range` | Ultrasonic distance |
| `ir_avoid/state` | `Int32` | IR obstacle (bit0=L, bit1=R) |
| `tracking/state` | `Int32` | Line tracking (4-bit) |
| `imu/data` | `Imu` | 6-axis accel+gyro (LSM6DSOX) |
| `imu/yaw_deg` | `Float64` | Integrated yaw heading |
| `imu/calibrated` | `Bool` | Gyro calibration status |
| `imu/temperature` | `Float32` | IMU die temperature |
| `imu/mic_level` | `Int32` | Microphone RMS level |
| `imu/audio` | `UInt8MultiArray` | Audio PCM stream (8 kHz, 8-bit) |

### AI / Hailo-8

| Topic | Type | Source |
|---|---|---|
| `detections/json` | `String` | Detection results (JSON) |
| `depth/image` | `Image` (32FC1) | Raw metric depth map |
| `depth/colorized/compressed` | `CompressedImage` | Colorized depth (TURBO colormap) |

### Actuators & control

| Topic | Type | Direction |
|---|---|---|
| `cmd_vel` | `Twist` | → motor driver |
| `camera_gimbal/command_deg` | `Vector3` | → gimbal (x=pan, y=tilt) |
| `lightbar/command` | `String` (JSON) | → WS2812 LED bar |
| `tracking/enable` | `Bool` | → enable person tracking |
| `tracking/config` | `Int32MultiArray` | → [pan_sign, tilt_sign] |
| `follow/enable` | `Bool` | → enable auto-follow |
| `follow/strafe_gain` | `Float64` | → tune strafe intensity |
| `follow/gyro_damping` | `Float64` | → tune gyro damping |
| `depth/enable` | `Bool` | → enable/disable depth |
| `imu/audio_enable` | `Bool` | → enable mic streaming |
| `imu/calibrate` | `Empty` | → trigger gyro calibration |

## Web UI features (port 8080)

- **Live MJPEG streams**: rear camera (`/stream.mjpg`), front camera (`/stream_front.mjpg`), depth map (`/stream_depth.mjpg`)
- **Gimbal control**: pan/tilt sliders with center button
- **Drive (WASD)**: keyboard teleop with shift-for-speed, auto-stop on keyup
- **Detection overlay**: bounding boxes drawn on the live stream canvas
- **Person tracking**: toggle gimbal tracking with pan/tilt invert options
- **Auto-follow**: PID-based robot following with mecanum strafing, IMU gyro damping, heading hold, configurable target area & speed
- **Light bar**: colour picker, per-LED control, breathing/rainbow/chase effects
- **Snapshots**: capture full-resolution stills to `~/Pictures/raspbot/`
- **IMU dashboard**: live accelerometer/gyroscope, 3D orientation cube, yaw heading, temperature
- **Microphone**: browser audio playback from Arduino PDM mic (8 kHz)
- **Depth map**: toggleable Hailo-8 monocular depth visualization with TURBO colormap

## Arduino firmware

The Arduino Nano RP2040 Connect provides the IMU (LSM6DSOX) and PDM microphone
over USB serial at 115200 baud. The firmware and a flash script are in
`firmware/arduino_nano_rp2040/`.

```bash
# Flash from the Pi (requires arduino-cli)
cd firmware/arduino_nano_rp2040
./flash.sh --port /dev/ttyACM0
```

See `firmware/arduino_nano_rp2040/README.md` for the full serial protocol
(`$IMU`, `$AUD`, `$CAL`, `$INFO`, `$ERR` output lines; `?`, `C`, `A`/`a`,
`R`/`G`/`B`/`W`/`O` input commands).

## M5Stack Cardputer controller

The M5Stack Cardputer (ESP32-S3, 240×135 TFT, 56-key keyboard) works as a
handheld teleop controller — live camera view + WASD keyboard driving, all
over WiFi (no Bluetooth needed).

```bash
# Build & flash (requires PlatformIO)
cd firmware/m5stack_cardputer
# Edit src/config.h with your WiFi credentials and Pi IP address
pio run -t upload
```

Key features: camera toggle (front/gimbal), speed control (+/-), ultrasonic
distance HUD, collision failsafe indicator, emergency stop (Space).
See `firmware/m5stack_cardputer/README.md` for the full keyboard map and setup.

## Hailo-8 models

Both models run on a single Hailo-8 device (shared `VDevice` in one process):

| Model | HEF | Input | Output | Throughput (Hailo-8) |
|---|---|---|---|---|
| YOLOv5s person+face | `yolov5s_personface.hef` (14 MB) | 640×640×3 | NMS detections (2 classes) | ~100 FPS |
| fast_depth | `fast_depth.hef` (3.1 MB) | 224×224×3 | 224×224×1 depth map | ~2500 FPS |

Models are stored in `~/.local/share/raspbot/models/hailo8/`. Use the download
script to fetch them:

```bash
ros2 run raspbot_hailo_tracking download_hailo_model
```

## Hardware notes

See `HARDWARE.md` for a consolidated hardware reference including:

- Pin mapping (BOARD numbering)
- I2C controller register map
- Pi Camera Module 3 (IMX708) setup
- Hailo-8 PCIe accelerator
- Arduino Nano RP2040 Connect wiring
- USB webcam auto-detection

## Troubleshooting

- **GPIO errors on Pi 5**: `RPi.GPIO` may fail with "Cannot determine SOC
  peripheral base address". The nodes auto-fallback to `lgpio`. Ensure
  `/dev/gpiochip*` permissions are correct.
- **USB camera at wrong `/dev/video`**: CSI cameras claim video0–7. The
  `opencv_camera` node auto-detects USB cameras when `device_index:=-1`
  (default) by scanning `/sys/class/video4linux` and filtering out
  platform/CSI devices.
- **Hailo `OUT_OF_PHYSICAL_DEVICES`**: Detection and depth share one `VDevice`
  in-process. Do not run a second Hailo process simultaneously.
- **Pi Camera timeouts**: Check the CSI ribbon cable seating. Try
  `libcamera-hello` to verify hardware before launching.
- **IMU calibration**: The Arduino performs gyro calibration on first connect
  (hold the robot still for ~2 seconds). Re-trigger with
  `ros2 topic pub --once imu/calibrate std_msgs/msg/Empty`.
