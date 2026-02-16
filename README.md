# Raspbot ROS 2 packages

ROS 2 workspace for the **Yahboom Raspbot V2** on a **Raspberry Pi 5** (Ubuntu 24.04).

## Packages

| Package | Description |
|---|---|
| `raspbot_hw` | Mecanum motor driver (PID heading-hold, adaptive trim, collision/cliff failsafe), GPIO sensors, USB camera (auto-detect), Pi Camera Module 3 (PDAF autofocus), 2DOF gimbal (IMU tilt compensation), WS2812 light-bar, OLED, IMU (Arduino + BNO055 9-DOF), odometry with path recording & return-to-origin, YDLidar T-mini Plus |
| `raspbot_bringup` | Unified bringup launch + startup sound + systemd service |
| `raspbot_teleop` | Terminal-based keyboard teleop (mecanum strafe) + gimbal teleop |
| `raspbot_web_video` | Web UI on port 8080 — MJPEG streams, drive (WASD), gimbal, detection overlay, auto-follow, LiDAR visualisation, odometry path canvas, collision/cliff failsafe controls, snapshots, IMU dashboard, microphone, depth map |
| `raspbot_hailo_tracking` | Hailo-8 object detection (YOLOv5s person+face), gimbal person tracking, PID auto-follow with mecanum strafing, IMU feedback & obstacle avoidance, lost-target scanning, monocular depth estimation (FastDepth) |

## Quick start

```bash
# Build (include ydlidar if you have the T-mini Plus attached)
colcon build --packages-select \
  raspbot_hw raspbot_bringup raspbot_teleop \
  raspbot_web_video raspbot_hailo_tracking \
  ydlidar_ros2_driver

# Full bringup (all features enabled by default)
ros2 launch raspbot_bringup bringup.launch.py
```

Then open **http://\<pi-ip\>:8080/** in a browser.

## Auto-start on boot

A systemd service launches the full bringup automatically when the Pi powers on.
The OLED displays the IP address as soon as the network is ready.

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

The service runs as the `jedld` user with I2C, GPIO, video, dialout, and
plugdev group access. It waits for network (up to 30 s) before launching so the
OLED can display the IP, and depends on `hailort.service` for the Hailo-8.

### Updating the service after code changes

The systemd service loads nodes from the **install/** overlay, so any source
code or config changes require a rebuild + service restart to take effect.

The easiest way is the **`sync_service.sh`** script at the repo root:

```bash
# Rebuild ALL raspbot packages, sync the unit file if changed, and restart
./sync_service.sh

# Rebuild only specific package(s)
./sync_service.sh raspbot_hw
./sync_service.sh raspbot_hw raspbot_teleop

# Only re-copy the systemd unit file (skip build)
./sync_service.sh --service-only
```

The script will:
1. `colcon build --symlink-install` the requested packages
2. Compare the source service unit file against `/etc/systemd/system/raspbot.service` — if they differ, copy and `daemon-reload` automatically
3. Restart the service and verify it's running

<details>
<summary>Manual steps (if you prefer not to use the script)</summary>

```bash
# 1. Rebuild the changed package(s)
cd ~/ros2_foxy
source install/setup.bash
colcon build --packages-select raspbot_hw          # ← replace with the package(s) you changed
# Tip: add --symlink-install so that Python files & config YAML are
#       symlinked instead of copied — future edits take effect on restart
#       without a rebuild.

# 2. Restart the service to pick up changes
sudo systemctl restart raspbot

# 3. Verify it's running
sudo systemctl status raspbot --no-pager
sudo journalctl -u raspbot -f          # live logs
```

If the **service unit file itself** (`raspbot_bringup/systemd/raspbot.service`)
was changed, you must also re-copy it and reload systemd:

```bash
sudo cp src/raspbot_ros2/raspbot_bringup/systemd/raspbot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl restart raspbot
```

> **Common pitfall:** forgetting `daemon-reload` after copying a new unit file.
> systemd will keep running the old version until you reload.

</details>

## Cardputer Bluetooth teleop (default-enabled)

`raspbot_bringup` starts BLE Cardputer teleop by default:

- `enable_bt_cardputer_teleop:=true`
- `bt_cardputer_name:=RaspbotCardputer`

Bluetooth defaults:

- Pi side: bringup powers Bluetooth on automatically at startup.
- Cardputer side: firmware advertises BLE automatically at boot.

Recommended one-time pairing/trusting on Pi (use your Cardputer MAC):

```bash
bluetoothctl
power on
scan on
pair AA:BB:CC:DD:EE:FF
trust AA:BB:CC:DD:EE:FF
connect AA:BB:CC:DD:EE:FF
quit
```

Then pin the paired device in launch/systemd bringup:

```bash
ros2 launch raspbot_bringup bringup.launch.py bt_cardputer_address:=AA:BB:CC:DD:EE:FF
```

## Launch arguments

All arguments have sensible defaults and can be toggled individually:

```bash
# Disable specific subsystems
ros2 launch raspbot_bringup bringup.launch.py enable_hailo:=false
ros2 launch raspbot_bringup bringup.launch.py enable_web_video:=false
ros2 launch raspbot_bringup bringup.launch.py enable_lidar:=false
ros2 launch raspbot_bringup bringup.launch.py enable_front_camera:=false
ros2 launch raspbot_bringup bringup.launch.py enable_imu:=false

# Disable SLAM and obstacle avoidance
ros2 launch raspbot_bringup bringup.launch.py enable_slam:=false enable_lidar_obstacle:=false

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
| `enable_motors` | `true` | Mecanum motor driver (PID heading-hold, failsafes) |
| `enable_ultrasonic` | `true` | Ultrasonic range sensor (I2C or GPIO) |
| `enable_gpio_sensors` | `true` | IR avoid + line tracking |
| `enable_camera` | `true` | USB webcam (auto-detect) |
| `enable_front_camera` | `true` | Pi Camera Module 3 (CSI, PDAF autofocus) |
| `enable_gimbal` | `true` | 2DOF pan/tilt servos (IMU tilt compensation) |
| `enable_lightbar` | `true` | WS2812 RGB LED bar (effects) |
| `enable_oled` | `true` | I2C OLED display (IP, range, velocity) |
| `enable_imu` | `true` | Arduino IMU bridge (LSM6DSOX + optional BNO055) |
| `enable_bno055` | `false` | Standalone BNO055 9-DOF via Pico RP2040 bridge |
| `enable_odometry` | `true` | Dead-reckoning odometry + path recording + return-to-origin |
| `enable_lidar` | `true` | YDLidar T-mini Plus (360° laser scan) |
| `enable_slam` | `true` | LiDAR occupancy grid SLAM (map + TF `map→odom`) |
| `enable_lidar_obstacle` | `true` | 360° LiDAR obstacle avoidance (4-zone Range) |
| `enable_web_video` | `true` | Web UI (port 8080) |
| `enable_hailo` | `true` | Hailo-8 detection + depth |
| `play_startup_sound` | `true` | Play sound on boot |
| `hailo_hef_path` | `~/.local/share/raspbot/models/hailo8/yolov5s_personface.hef` | Detection model |
| `hailo_labels_path` | `~/.local/share/raspbot/models/hailo8/personface.labels` | Detection labels |
| `depth_hef_path` | `~/.local/share/raspbot/models/hailo8/fast_depth.hef` | Depth model (empty = disabled) |
| `hailo_pan_sign` | `-1` | Gimbal pan direction |
| `hailo_tilt_sign` | `-1` | Gimbal tilt direction |
| `tracking_config_topic` | `tracking/config` | Detection config topic |
| `follow_enable_topic` | `follow/enable` | Auto-follow enable topic |
| `enable_bt_cardputer_teleop` | `true` | Start BLE Cardputer teleop bridge |
| `bt_cardputer_name` | `RaspbotCardputer` | BLE Cardputer advertised name |
| `bt_cardputer_address` | `` | Optional paired Cardputer MAC address |
| `params_file` | (auto) | Path to `raspbot_hw.yaml` parameter file |

</details>

## ROS 2 topics

### Sensors & cameras

| Topic | Type | Source |
|---|---|---|
| `image_raw/compressed` | `CompressedImage` | USB webcam (auto-detected) |
| `front_camera/compressed` | `CompressedImage` | Pi Camera Module 3 (IMX708, PDAF autofocus) |
| `ultrasonic/range` | `Range` | Ultrasonic distance (I2C preferred, GPIO fallback) |
| `ir_avoid/state` | `Int32` | IR obstacle (bit0=L, bit1=R) |
| `tracking/state` | `Int32` | Line tracking (4-bit bitmask) |
| `/scan` | `LaserScan` | YDLidar T-mini Plus (360°, 0.05–12 m, 10 Hz) |
| `imu/data` | `Imu` | 6-axis accel+gyro (LSM6DSOX) or fused 9-DOF quaternion (BNO055) |
| `imu/yaw_deg` | `Float64` | Yaw heading (gyro-integrated or magnetometer-stabilised) |
| `imu/calibrated` | `Bool` | IMU calibration status |
| `imu/calibration` | `String` | BNO055 calibration JSON: `{"sys":3,"gyro":3,"accel":2,"mag":1}` |
| `imu/temperature` | `Float32` | IMU die temperature (°C) |
| `imu/gravity` | `Vector3` | BNO055 gravity vector (body frame) |
| `imu/mag` | `Vector3` | BNO055 raw magnetometer (µT) |
| `imu/mic_level` | `Int32` | Microphone RMS level |
| `imu/audio` | `UInt8MultiArray` | Audio PCM stream (8 kHz, 8-bit) |

### Odometry & navigation

| Topic | Type | Source |
|---|---|---|
| `odom` | `Odometry` | Dead-reckoned pose (cmd_vel + IMU yaw) |
| `odom/path` | `Path` | Recorded waypoints (live during recording) |
| `odom/recording` | `Bool` | True while path recording is active |
| `odom/returning` | `Bool` | True while return-to-origin is in progress |
| TF: `odom` → `base_link` | `TransformStamped` | Odometry transform |
| TF: `map` → `odom` | `TransformStamped` | SLAM map-to-odom correction (10 Hz) |
| TF: `base_link` → `laser_frame` | `TransformStamped` | Static LiDAR offset (0.05 m Z) |
| `map` | `OccupancyGrid` | Occupancy grid (200×200 @ 0.05 m = 10 m²) |
| `slam/active` | `Bool` | True while SLAM node is running |
| `lidar/front_range` | `Range` | LiDAR min distance — front zone (±30°) |
| `lidar/left_range` | `Range` | LiDAR min distance — left zone (30°–150°) |
| `lidar/right_range` | `Range` | LiDAR min distance — right zone |
| `lidar/rear_range` | `Range` | LiDAR min distance — rear zone |
| `lidar_obstacle/active` | `Bool` | True while obstacle avoidance is running |

### AI / Hailo-8

| Topic | Type | Source |
|---|---|---|
| `detections/json` | `String` | Detection results (JSON) |
| `depth/image` | `Image` (32FC1) | Raw metric depth map |
| `depth/colorized/compressed` | `CompressedImage` | Colorized depth (TURBO colormap) |

### Actuators & control

| Topic | Type | Direction |
|---|---|---|
| `cmd_vel` | `Twist` | → motor driver (mecanum: x=fwd, y=strafe, z=yaw) |
| `camera_gimbal/command_deg` | `Vector3` | → gimbal (x=pan, y=tilt) |
| `lightbar/command` | `String` (JSON) | → WS2812 LED bar |
| `tracking/enable` | `Bool` | → enable person tracking |
| `tracking/config` | `Int32MultiArray` | → [pan_sign, tilt_sign] |
| `follow/enable` | `Bool` | → enable auto-follow |
| `follow/strafe_gain` | `Float64` | → tune strafe intensity |
| `follow/gyro_damping` | `Float64` | → tune gyro damping |
| `follow/target_area` | `Float64` | → target bbox area for follow distance |
| `follow/max_linear` | `Float64` | → max forward speed during follow |
| `collision_failsafe/enable` | `Bool` | → toggle collision failsafe |
| `cliff_failsafe/enable` | `Bool` | → toggle cliff/edge failsafe |
| `depth/enable` | `Bool` | → enable/disable depth estimation |
| `imu/audio_enable` | `Bool` | → enable mic streaming |
| `imu/calibrate` | `Empty` | → trigger gyro calibration |
| `imu/save_cal` | `Empty` | → save BNO055 calibration to flash |
| `imu/load_cal` | `Empty` | → load BNO055 calibration from flash |

### Safety

| Topic | Type | Direction |
|---|---|---|
| `collision_failsafe/active` | `Bool` | ← true when ultrasonic detects obstacle |
| `cliff_failsafe/active` | `Bool` | ← true when IR sensors detect edge/cliff |

### Services

| Service | Type | Node | Description |
|---|---|---|---|
| `odom/set_origin` | `Trigger` | odometry | Reset pose to (0,0,0) |
| `odom/start_recording` | `Trigger` | odometry | Begin recording waypoints |
| `odom/stop_recording` | `Trigger` | odometry | Stop recording |
| `odom/return_to_origin` | `Trigger` | odometry | Autonomously retrace path in reverse |
| `odom/cancel_return` | `Trigger` | odometry | Abort return-to-origin |
| `slam/reset` | `Trigger` | lidar_slam | Clear occupancy grid and reset map |

## Motor driver features

The mecanum motor driver (`motor_driver` node) includes:

- **Mecanum holonomic drive** — 4-wheel inverse kinematics (`linear.x`, `linear.y`, `angular.z`)
- **PID heading-hold** — uses IMU yaw (BNO055 magnetometer-stabilised preferred) to correct drift when driving straight; full Kp/Ki/Kd with anti-windup
- **Adaptive motor trim** — learns L/R asymmetry from sustained PID corrections, reducing correction load over time
- **Per-motor manual trim** — `trim_fl`/`trim_fr`/`trim_rl`/`trim_rr` for known hardware bias
- **Lateral drift correction** — accelerometer-based sideways drift compensation (mecanum-specific)
- **Collision failsafe** — ultrasonic + LiDAR front range fusion: uses `min(ultrasonic, lidar_front)` for forward stop/slowdown zone (configurable distances, runtime toggle)
- **Cliff failsafe** — IR line-tracker-based edge detection blocks forward motion (configurable sensor mask)
- **Startup kick** — brief PWM boost to overcome static friction
- **Strafe stabilisation** — correction deadband + PWM slew rate limiting during pure strafe
- **Differential mode** — alternative 2-wheel drive mode (`drive_mode: "differential"`)

All parameters are runtime-tunable via `ros2 param set`.

## LiDAR SLAM & obstacle avoidance

### Occupancy grid SLAM (`lidar_slam` node)

A lightweight pure-Python SLAM node designed for the Pi 5's constrained resources.
Builds an occupancy grid from `/scan` + `/odom` using log-odds Bresenham ray-casting.

- **Grid**: 200×200 cells at 0.05 m resolution (10 m × 10 m)
- **Update rate**: 2 Hz scan processing, 1 Hz map publish, 10 Hz TF broadcast
- **Scan downsample**: 3× (every 3rd ray) to reduce CPU
- **Optional scan matching**: Correlative scan-to-map matching for drift correction (disabled by default — `slam_scan_match_enable: true` to enable)
- Publishes: `/map` (OccupancyGrid), TF `map→odom`, `/slam/active` (Bool)
- Service: `slam/reset` — clears the grid

Key parameters (in `raspbot_hw.yaml` under `lidar_slam:`):

| Parameter | Default | Description |
|---|---|---|
| `slam_resolution` | `0.05` | Cell size in metres |
| `slam_width` / `slam_height` | `200` | Grid dimensions (cells) |
| `slam_update_hz` | `2.0` | Scan processing rate |
| `slam_map_publish_hz` | `1.0` | OccupancyGrid publish rate |
| `slam_min_range` / `slam_max_range` | `0.1` / `8.0` | Valid scan range |
| `slam_scan_downsample` | `3` | Use every Nth ray |
| `slam_scan_match_enable` | `false` | Enable correlative scan matching |
| `slam_scan_match_search_xy` | `0.3` | Search window (metres) |
| `slam_scan_match_search_yaw` | `0.15` | Search window (radians) |

### LiDAR obstacle avoidance (`lidar_obstacle` node)

Divides the 360° scan into four angular zones and publishes the minimum distance
for each as `sensor_msgs/Range`:

| Zone | Angular range | Topic |
|---|---|---|
| Front | ±30° from 0° | `lidar/front_range` |
| Left | 30°–150° | `lidar/left_range` |
| Rear | 150°–210° | `lidar/rear_range` |
| Right | 210°–330° | `lidar/right_range` |

- **Stop distance**: 0.20 m (robot halts)
- **Slow distance**: 0.50 m (speed reduced)
- The motor driver fuses `lidar/front_range` with the ultrasonic sensor for collision failsafe: `effective_distance = min(ultrasonic, lidar_front)`

## Web UI features (port 8080)

- **Live MJPEG streams**: rear camera (`/stream.mjpg`), front camera (`/stream_front.mjpg`), depth map (`/stream_depth.mjpg`)
- **Gimbal control**: pan/tilt sliders with center button
- **Drive (WASD)**: keyboard teleop with Shift for speed, auto-stop on keyup/tab switch
- **Detection overlay**: bounding boxes drawn on the live stream canvas
- **Person tracking**: toggle gimbal tracking with pan/tilt invert options
- **Auto-follow**: PID-based robot following with mecanum strafing, IMU gyro damping, heading hold, configurable target area & speed, lost-target scanning
- **Collision failsafe**: toggle + live ultrasonic distance readout
- **Cliff/edge failsafe**: toggle + per-sensor status display
- **Light bar**: colour picker, per-LED control, breathing/rainbow/chase effects
- **Snapshots**: capture full-resolution stills to `~/Pictures/raspbot/`
- **Odometry & navigation**: canvas path visualisation, X/Y/yaw readouts, distance/speed, record/stop/return-to-origin/cancel controls, **SLAM map overlay** (toggleable occupancy grid rendered on the path canvas with free/wall legend, cell count, and Reset Map button)
- **LiDAR scan**: live 360° point cloud canvas with zoom, range/FOV/point count readouts
- **LiDAR obstacle zones**: live front/left/right/rear distance table with colour-coded proximity (red < 0.2 m, orange < 0.5 m, green otherwise)
- **IMU dashboard**: live accelerometer/gyroscope, 3D orientation cube, yaw heading, calibration status, temperature
- **Microphone**: browser audio playback from Arduino PDM mic (8 kHz)
- **Depth map**: toggleable Hailo-8 monocular depth visualization with TURBO colormap

## Terminal teleop

The `raspbot_teleop` package provides terminal-based control (useful over SSH):

```bash
# Keyboard drive (WASD + J/L for strafe)
ros2 launch raspbot_teleop keyboard_teleop.launch.py

# Gimbal control (arrow keys / H/J/K/L)
ros2 launch raspbot_teleop gimbal_teleop.launch.py
```

## Bluetooth Cardputer teleop

BLE teleop bridge starts in bringup by default and powers on Bluetooth at startup.

```bash
# Default bringup (BLE teleop enabled)
ros2 launch raspbot_bringup bringup.launch.py

# Target paired Cardputer by MAC (recommended)
ros2 launch raspbot_bringup bringup.launch.py bt_cardputer_address:=AA:BB:CC:DD:EE:FF

# Disable BLE teleop bridge
ros2 launch raspbot_bringup bringup.launch.py enable_bt_cardputer_teleop:=false
```

Cardputer firmware and instructions:

- `firmware/cardputer_bt_teleop/cardputer_bt_teleop.ino`
- `firmware/cardputer_bt_teleop/README.md`

## Arduino firmware

The Arduino Nano RP2040 Connect provides the IMU (LSM6DSOX) and PDM microphone
over USB serial at 115200 baud. When a BNO055 is wired to A4/A5, it also
streams fused 9-DOF orientation data. The firmware is in
`firmware/arduino_nano_rp2040/`.

```bash
# Flash from the Pi (requires arduino-cli)
cd firmware/arduino_nano_rp2040
./flash.sh --port /dev/ttyACM0
```

See `firmware/arduino_nano_rp2040/README.md` for the full serial protocol
(`$IMU`, `$BNO`, `$GRV`, `$AUD`, `$CAL`, `$INFO`, `$ERR` output lines;
`?`, `C`, `A`/`a`, `S`, `L`, `R`/`G`/`B`/`W`/`O`, `1`/`2`/`3` input commands).

## BNO055 9-DOF IMU (Pico RP2040 bridge)

An optional Raspberry Pi Pico (RP2040) bridges a BNO055 9-DOF absolute
orientation sensor over USB serial. This provides drift-free magnetometer-
stabilised heading, fused quaternion orientation, and gravity-compensated
linear acceleration — significantly improving heading-hold, odometry, and
tilt compensation.

```bash
# Build & flash
cd firmware/pico_rp2040_bno055
./build_and_flash_pico.sh
```

See `firmware/pico_rp2040_bno055/WIRING.md` for the I2C wiring diagram
(GP4=SDA, GP5=SCL). Enable with `enable_bno055:=true` in the launch file.

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
| FastDepth | `fast_depth.hef` (3.1 MB) | 224×224×3 | 224×224×1 depth map | ~2500 FPS |

Models are stored in `~/.local/share/raspbot/models/hailo8/`. Use the download
script to fetch them:

```bash
ros2 run raspbot_hailo_tracking download_hailo_model
```

## Utilities

```bash
# Identify which motor ID maps to which physical wheel
ros2 run raspbot_hw motor_id_test
```

## Hardware notes

See `HARDWARE.md` for a consolidated hardware reference including:

- Pin mapping (BOARD numbering)
- I2C controller register map
- BNO055 wiring & calibration
- LiDAR configuration
- Full parameter reference (~175+ tunable parameters)
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
