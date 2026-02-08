# Raspbot ROS 2 packages

This folder contains a minimal ROS 2 Foxy bringup for the Yahboom Raspbot V2 hardware.

Packages:

- `raspbot_hw`: I2C motor driver + GPIO sensors + optional camera publisher
- `raspbot_bringup`: unified bringup launch

Start here:

- Build: `colcon build --packages-select raspbot_hw raspbot_bringup`
- Bringup: `ros2 launch raspbot_bringup bringup.launch.py enable_camera:=true`

Quick component testing:

- Camera only: `ros2 launch raspbot_bringup bringup.launch.py enable_motors:=false enable_ultrasonic:=false enable_gpio_sensors:=false enable_camera:=true`
- Sensors only (no motors): `ros2 launch raspbot_bringup bringup.launch.py enable_motors:=false`

If your Yahboom scripts work, but these nodes can’t access hardware, it’s almost always permissions (I2C/GPIO/video) or different pin numbering (BOARD vs BCM). The default pins match Yahboom’s course notebooks:

- Ultrasonic: Trig=16, Echo=18 (BOARD)
- Avoid: L=21, R=19, ON=22 (BOARD)
- Tracking: L1=13, L2=15, R1=11, R2=7 (BOARD)

Raspberry Pi 5 GPIO:

- `RPi.GPIO` may error with “Cannot determine SOC peripheral base address”. The nodes will automatically fall back to `lgpio`.
- `lgpio` needs access to `/dev/gpiochip*`. If your devices are `crw------- root root`, fix udev permissions or run as root.
