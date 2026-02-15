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
- Disable startup sound: `play_startup_sound:=false`
- Disable OLED: `enable_oled:=false`
- Enable web camera UI: `enable_web_video:=true`
- Enable Hailo detection/tracking: `enable_hailo:=true hailo_hef_path:=/path/to/model.hef`
	- Optional labels: `hailo_labels_path:=~/.local/share/raspbot/models/hailo8/coco80.labels`

## Bluetooth Cardputer teleop

BLE Cardputer teleop is enabled by default in bringup.

- `enable_bt_cardputer_teleop:=true`
- `bt_cardputer_name:=RaspbotCardputer`
- optional `bt_cardputer_address:=AA:BB:CC:DD:EE:FF` (recommended for paired fixed-MAC setup)

Bringup startup also powers on Bluetooth automatically on Pi.

### Make Bluetooth start reliably after reboot

Install/refresh the service unit and enable both services:

- `sudo cp ~/ros2_foxy/src/raspbot_ros2/raspbot_bringup/systemd/raspbot.service /etc/systemd/system/`
- `sudo systemctl daemon-reload`
- `sudo systemctl enable bluetooth.service raspbot.service`
- `sudo systemctl restart bluetooth.service raspbot.service`

After reboot, verify:

- `systemctl is-active bluetooth raspbot`
- `bluetoothctl show | grep Powered`

Expected state is `active` for both services and `Powered: yes` from `bluetoothctl`.

Optional: force reconnect to a specific paired Cardputer MAC on boot:

- `sudo tee /etc/default/raspbot >/dev/null <<'EOF'`
- `CARDPUTER_BT_ADDRESS=AA:BB:CC:DD:EE:FF`
- `CARDPUTER_BT_NAME=RaspbotCardputer`
- `CARDPUTER_BT_PRECONNECT=0`
- `EOF`
- `sudo systemctl restart raspbot.service`

If `CARDPUTER_BT_ADDRESS` is omitted, startup tries to find a paired device matching `CARDPUTER_BT_NAME`.
For BLE teleop, keep `CARDPUTER_BT_PRECONNECT=0` so the ROS BLE bridge can own the GATT session.

## OLED screen

Many Raspbot controller stacks include a small I2C OLED screen (often address `0x3C`).

This bringup starts an `oled` node by default (disable with `enable_oled:=false`). The node displays basic status (IP, ultrasonic distance, cmd_vel) and accepts custom text:

- Publish custom text: `ros2 topic pub -1 /oled/text std_msgs/msg/String "{data: 'Hello\nRaspbot'}"`

If the OLED is blank or the text looks garbled, run a one-time probe cycle that tries common panel configs (128x64/128x32 + rotation) so you can see which one is correct:

- `ros2 run raspbot_hw oled --ros-args -p startup_probe:=true`

Then set `oled.ros__parameters.height` to `32` or `64` and/or `oled.ros__parameters.rotate_180` to `true` in your params YAML.

Probe hint: if the **3rd** screen looks best, that corresponds to `height:=32` and `rotate_180:=false`.

Custom params file:

- `ros2 launch raspbot_bringup bringup.launch.py params_file:=/path/to/your.yaml`

## Camera in a browser (MJPEG)

If you enabled the camera (`enable_camera:=true`), you can view the feed in a browser using the small web streamer package:

- Build once: `colcon build --packages-select raspbot_web_video`
- Run: `ros2 run raspbot_web_video web_video`
- Open: `http://<robot-ip>:8080/`

You can also run it directly from bringup:

- `ros2 launch raspbot_bringup bringup.launch.py enable_camera:=true enable_web_video:=true`

By default it subscribes to `image_raw/compressed` (JPEG). If your camera topic differs:

- `ros2 run raspbot_web_video web_video --ros-args -p topic:=/image_raw/compressed`

## Hailo-8 object detection + person tracking

If you have a Hailo HEF model on the robot, you can start the detector node from bringup. It publishes detection JSON for the web UI overlay and listens for a tracking enable toggle.

To download a known-good open-source COCO detector (pre-compiled HEF for Hailo-8) into `~/.local/share/raspbot/models/hailo8`:

- `ros2 run raspbot_hailo_tracking download_hailo_model --model yolox_s_leaky`

- Start everything: `ros2 launch raspbot_bringup bringup.launch.py enable_camera:=true enable_web_video:=true enable_hailo:=true hailo_hef_path:=/path/to/model.hef hailo_labels_path:=~/.local/share/raspbot/models/hailo8/coco80.labels`

If tracking moves the camera the wrong direction, flip the signs (common when the gimbal axes are wired opposite or the camera is mounted mirrored):

- Flip pan: `hailo_pan_sign:=-1`
- Flip tilt: `hailo_tilt_sign:=-1`

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
