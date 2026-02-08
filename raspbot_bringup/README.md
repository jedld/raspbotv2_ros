# raspbot_bringup

One-command bringup for Yahboom Raspbot V2 on ROS 2 Foxy.

## Build

From your workspace root:

- `cd ~/ros2_foxy`
- `colcon build --packages-select raspbot_hw raspbot_bringup`
- `source install/local_setup.bash`

## Run (bringup)

- `ros2 launch raspbot_bringup bringup.launch.py`

## Test driving (keyboard teleop)

Run bringup in one terminal, then in a second terminal run:

- `ros2 run raspbot_teleop keyboard_teleop`

This publishes to `cmd_vel`. Keep the teleop terminal focused while driving.

Optional flags:

- Disable motors: `enable_motors:=false`
- Disable camera: `enable_camera:=false` (default)
- Enable camera: `enable_camera:=true`
- Disable ultrasonic: `enable_ultrasonic:=false`
- Disable GPIO sensors: `enable_gpio_sensors:=false`

Custom params file:

- `ros2 launch raspbot_bringup bringup.launch.py params_file:=/path/to/your.yaml`

## Topics

- Motors: subscribes `cmd_vel` (`geometry_msgs/Twist`)
- Ultrasonic: publishes `ultrasonic/range` (`sensor_msgs/Range`)
- IR avoid: publishes `ir_avoid/state` (`std_msgs/Int32`, bit0=left, bit1=right)
- Line tracking: publishes `tracking/state` (`std_msgs/Int32`, bit0=L1, bit1=L2, bit2=R1, bit3=R2)
- Camera (optional): publishes `image_raw/compressed` (`sensor_msgs/CompressedImage`)

## Permissions / prerequisites

You typically need access to:

- I2C: `/dev/i2c-1` (often group `i2c`)
- GPIO: depends on distro; `RPi.GPIO` may require root or the right device permissions
- Raspberry Pi 5 note: `RPi.GPIO` often fails; nodes auto-fallback to `lgpio` (uses `/dev/gpiochip*`)
- Camera: `/dev/video0` (group `video`)

Useful checks:

- `ls -l /dev/i2c-1 /dev/video0`
- `i2cdetect -y 1` (Pi 5 often shows motor controller at `2b`)

If the motor driver exits with an I2C error:

- Confirm the controller is powered and visible on the bus (`i2cdetect -y 1`)
- If the address isn’t `0x2B`, update `motor_driver.ros__parameters.i2c_addr` in your params file
- If you’re on an older controller (address `0x16`), set `motor_driver.ros__parameters.i2c_protocol: legacy`
