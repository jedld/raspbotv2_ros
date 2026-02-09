# Raspbot ROS 2 packages

This folder contains a minimal ROS 2 Foxy bringup for the Yahboom Raspbot V2 hardware.

Packages:

- `raspbot_hw`: I2C motor driver + GPIO sensors + optional camera publisher
- `raspbot_bringup`: unified bringup launch
- `raspbot_web_video`: tiny web MJPEG viewer for the camera topic (includes drive, gimbal, tracking, and follow controls)
- `raspbot_hailo_tracking`: Hailo-8 object detection, gimbal person tracking, and auto-follow

Start here:

- Build: `colcon build --packages-select raspbot_hw raspbot_bringup raspbot_web_video raspbot_hailo_tracking`
- Bringup: `ros2 launch raspbot_bringup bringup.launch.py enable_camera:=true`

The standard bringup now includes the web server (port 8080), Hailo detection,
gimbal tracking, and auto-follow by default. You can disable them individually:

```bash
# Disable Hailo detection/follow
ros2 launch raspbot_bringup bringup.launch.py enable_hailo:=false

# Disable web UI
ros2 launch raspbot_bringup bringup.launch.py enable_web_video:=false
```

The Hailo node needs a model HEF and labels file. If `hailo_hef_path` is empty
(the default), inference is disabled until you set it:

```bash
ros2 launch raspbot_bringup bringup.launch.py \
  hailo_hef_path:=/path/to/model.hef \
  hailo_labels_path:=/path/to/labels.txt
```

Quick component testing:

- Camera only: `ros2 launch raspbot_bringup bringup.launch.py enable_motors:=false enable_ultrasonic:=false enable_gpio_sensors:=false enable_camera:=true`
- Sensors only (no motors): `ros2 launch raspbot_bringup bringup.launch.py enable_motors:=false`

Hardware overview / board notes:

- See `HARDWARE.md` for a consolidated feature list, pin map, and I2C register notes.

If your Yahboom scripts work, but these nodes can’t access hardware, it’s almost always permissions (I2C/GPIO/video) or different pin numbering (BOARD vs BCM). The default pins match Yahboom’s course notebooks:

- Ultrasonic: Trig=16, Echo=18 (BOARD)
- Avoid: L=21, R=19, ON=22 (BOARD)
- Tracking: L1=13, L2=15, R1=11, R2=7 (BOARD)

Raspberry Pi 5 GPIO:

- `RPi.GPIO` may error with “Cannot determine SOC peripheral base address”. The nodes will automatically fall back to `lgpio`.
- `lgpio` needs access to `/dev/gpiochip*`. If your devices are `crw------- root root`, fix udev permissions or run as root.
