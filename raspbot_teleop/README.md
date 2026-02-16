# raspbot_teleop

Keyboard teleop node that publishes `geometry_msgs/Twist` to `cmd_vel`.

## Build

- `cd ~/ros2_foxy`
- `colcon build --packages-select raspbot_teleop`
- `source install/local_setup.bash`

## Run

In one terminal:

- `source ~/ros2_foxy/install/local_setup.bash`
- `ros2 launch raspbot_bringup bringup.launch.py`

In a second terminal (interactive):

- `source ~/ros2_foxy/install/local_setup.bash`
- `ros2 run raspbot_teleop keyboard_teleop`

Keys:

- `w/s` linear +/−
- `a/d` angular +/−
- `j/l` strafe left/right
- `x` or `space` stop
- `q` quit

## Params

- `topic` (default: `cmd_vel`)
- `publish_hz` (default: 20)
- `deadman_timeout_sec` (default: 0.5)
- `linear_step`, `angular_step`
- `lateral_step`
- `linear_max`, `angular_max`
- `lateral_max`

## Gimbal Teleop

Requires the gimbal node to be running (it’s part of bringup by default):

- `source ~/ros2_foxy/install/local_setup.bash`
- `ros2 launch raspbot_bringup bringup.launch.py enable_gimbal:=true`

In a second terminal (interactive):

- `source ~/ros2_foxy/install/local_setup.bash`
- `ros2 run raspbot_teleop gimbal_teleop`

Keys:

- Arrow keys (or `h/j/k/l`) nudge pan/tilt
- `[` / `]` decrease/increase step
- `space` or `x` reset to neutral
- `q` quit

Params:

- `topic` (default: `camera_gimbal/command_deg`)
- `step_deg` (default: 5)
- `pan_*` / `tilt_*` range + neutral degrees

## Bluetooth Cardputer Teleop

Bridge node for M5Stack Cardputer BLE control + telemetry HUD + camera capture.

- `source ~/ros2_foxy/install/local_setup.bash`
- `ros2 launch raspbot_teleop bt_cardputer_teleop.launch.py device_name:=RaspbotCardputer`

Optional fixed-MAC mode (recommended once paired/trusted):

- `ros2 launch raspbot_teleop bt_cardputer_teleop.launch.py device_address:=30:ED:A0:CA:17:B5`

Default behavior:

- Pi Bluetooth is powered on automatically (`bluetoothctl power on`) by bringup startup scripts.
- Cardputer BLE is enabled automatically on boot by firmware (`BLEDevice::init(...)` + advertising).
- If `device_address` is set, the bridge auto-runs `bluetoothctl trust <MAC>`.
- GATT control connection is owned by the BLE bridge (`bleak`), so avoid forcing `bluetoothctl connect` unless explicitly testing low-level links.

### BLE Protocol (Cardputer → Pi)

| Command | Format |
|---------|--------|
| Movement | `CMD\|VEL\|vx\|vy\|wz\n` |
| Gimbal | `CMD\|GMB\|pan\|tilt\n` |
| Follow toggle | `CMD\|FOL\|0/1\n` |
| Gimbal mode | `CMD\|GMODE\|0/1\n` |
| Photo request | `CMD\|PHOTO\n` |
| Keepalive | `CMD\|PING\n` |

### BLE Protocol (Pi → Cardputer)

| Message | Format |
|---------|--------|
| Telemetry | `TEL\|vx\|vy\|wz\|speed\|gyro_z\|compass\|heading\|follow\|gmode\|roll\|pitch\n` |
| Image header | `IMG\|BEGIN\|<total_bytes>\n` |
| Image data | Raw JPEG bytes in 180-byte chunks |
| Image error | `IMG\|ERROR\|<message>\n` |

### Photo Capture

The node subscribes to multiple CompressedImage topics:
* `image_raw/compressed` (Main camera)
* `front_camera/compressed` (Front camera)

On `CMD|PHOTO` request from the Cardputer:

1. Grabs the latest frames from BOTH cameras
2. Resizes to 240×135 via Pillow (JPEG quality 40)
3. Sends Main Photo header + data
4. Waits 3 seconds (for Cardputer SD write)
5. Sends Front Photo header + data
6. Cardputer displays the thumbnails and saves them sequentially

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `device_name` | `RaspbotCardputer` | BLE peripheral name to scan for |
| `device_address` | (empty) | Fixed BLE MAC address |
| `scan_timeout` | `8.0` | BLE scan timeout (seconds) |
| `reconnect_delay` | `3.0` | Delay between reconnection attempts |
| `publish_hz` | `20.0` | Drive command publish rate |
| `telemetry_hz` | `5.0` | Telemetry send rate to Cardputer |
| `deadman_timeout` | `0.5` | Stop if no command received (seconds) |
| `camera_topic` | `image_raw/compressed` | Main Camera topic |
| `camera_front_topic` | `front_camera/compressed` | Front Camera topic |
| `photo_width` | `240` | Photo resize width |
| `photo_height` | `135` | Photo resize height |
| `photo_jpeg_quality` | `40` | JPEG quality for resized photos |
| `auto_power_on_bt` | `true` | Run `bluetoothctl power on` at startup |

Cardputer control keys:

- `W/S`: Drive forward/backward
- `A/D`: Turn left/right
- `J/L`: Strafe left/right
- `X` or `Space`: Stop
- `F`: toggle auto-follow (`follow/enable`)
- `G`: toggle gimbal control mode
- `Fn + , / . / ;`: gimbal pan/tilt
- `P` or `Enter` (Camera panel): capture photo
- `1/2/3`: switch panel directly
- `Tab`: cycle to next panel

Cardputer display panels:

| Panel | Content |
|-------|---------|
| 1 – Teleop | Speed, velocity, gyro, compass, heading, roll/pitch, follow/gimbal states |
| 2 – Orient | Graphical compass with heading needle, cardinal labels, roll/pitch/gyro |
| 3 – Camera | Photo capture, JPEG thumbnail, SD save status |
