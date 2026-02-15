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

Bridge node for M5Stack Cardputer BLE control + telemetry HUD.

- `source ~/ros2_foxy/install/local_setup.bash`
- `ros2 launch raspbot_teleop bt_cardputer_teleop.launch.py device_name:=RaspbotCardputer`

Optional fixed-MAC mode (recommended once paired/trusted):

- `ros2 launch raspbot_teleop bt_cardputer_teleop.launch.py device_address:=AA:BB:CC:DD:EE:FF`

Default behavior:

- Pi Bluetooth is powered on automatically (`bluetoothctl power on`) by bringup startup scripts.
- Cardputer BLE is enabled automatically on boot by firmware (`BLEDevice::init(...)` + advertising).
- If `device_address` is set, the bridge auto-runs `bluetoothctl trust <MAC>`.
- GATT control connection is owned by the BLE bridge (`bleak`), so avoid forcing `bluetoothctl connect` unless explicitly testing low-level links.

Command channels over BLE:

- Movement (`cmd_vel`): `CMD|VEL|vx|vy|wz`
- Gimbal (`camera_gimbal/command_deg`): `CMD|GMB|pan|tilt`
- Auto-follow toggle (`follow/enable`): `CMD|FOL|0/1`
- Gimbal control mode: `CMD|GMODE|0/1`

Telemetry back to Cardputer:

- `TEL|vx|vy|wz|speed|gyro_z|compass|heading|follow|gimbal_mode`

Cardputer control keys:

- Hold `W/S`: forward/backward
- Hold `A/D`: yaw left/right
- Hold `J/L`: strafe left/right
- `X` or `Space`: stop
- `F`: toggle auto-follow (`follow/enable`)
- `G`: toggle gimbal control mode
- `Fn + , / . / ;`: gimbal pan/tilt

Cardputer display data:

- Bluetooth link state
- Cardputer local MAC address
- Follow and gimbal-mode state
- Speed + velocity vector
- Gyroscope (`gyro_z`)
- Compass
- Heading (`imu/yaw_deg`)
