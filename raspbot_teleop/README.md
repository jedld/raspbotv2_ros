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
- `x` or `space` stop
- `q` quit

## Params

- `topic` (default: `cmd_vel`)
- `publish_hz` (default: 20)
- `deadman_timeout_sec` (default: 0.5)
- `linear_step`, `angular_step`
- `linear_max`, `angular_max`
