# raspbot_web_video

Tiny web app to view the robot camera feed in a browser.

It subscribes to a ROS 2 `sensor_msgs/CompressedImage` topic (JPEG) and serves it as an **MJPEG** stream.

## Build

From your workspace root:

```bash
cd ~/ros2_foxy
colcon build --packages-select raspbot_web_video
source install/local_setup.bash
```

## Run

Start the camera (bringup):

```bash
ros2 launch raspbot_bringup bringup.launch.py enable_camera:=true
```

In a second terminal:

```bash
source ~/ros2_foxy/install/local_setup.bash
ros2 run raspbot_web_video web_video
```

Then open:

- `http://<robot-ip>:8080/`

### Common options

If your camera topic is different:

```bash
ros2 run raspbot_web_video web_video --ros-args -p topic:=/image_raw/compressed
```

Change bind/port:

```bash
ros2 run raspbot_web_video web_video --ros-args -p bind:=0.0.0.0 -p port:=8080
```

Limit FPS (server-side):

```bash
ros2 run raspbot_web_video web_video --ros-args -p fps_limit:=10.0
```

## Gimbal controls

If `raspbot_hw` gimbal is running (part of bringup by default), the page includes pan/tilt sliders.

Defaults:

- Topic: `camera_gimbal/command_deg` (`geometry_msgs/Vector3`, x=pan_deg, y=tilt_deg)
- Ranges: pan 0..180, tilt 0..110

Override topic/limits:

```bash
ros2 run raspbot_web_video web_video --ros-args \
	-p gimbal_topic:=/camera_gimbal/command_deg \
	-p pan_min_deg:=0.0 -p pan_max_deg:=180.0 \
	-p tilt_min_deg:=0.0 -p tilt_max_deg:=110.0
```

## Drive controls (WASD)

The page also supports driving the robot using **WASD** and publishes `cmd_vel` (`geometry_msgs/Twist`).

Safety behaviors:

- Auto-stop on key release
- Auto-stop if the tab loses focus / becomes hidden
- Server-side watchdog publishes a stop if commands stop arriving (`cmd_timeout_sec`)

Defaults:

- Topic: `cmd_vel`
- Max speed: `max_linear_mps=0.25`, `max_angular_rps=1.2`

Tune limits:

```bash
ros2 run raspbot_web_video web_video --ros-args \
	-p cmd_vel_topic:=/cmd_vel \
	-p max_linear_mps:=0.20 \
	-p max_angular_rps:=1.00 \
	-p cmd_timeout_sec:=0.5

## Detection + person tracking

If you run the Hailo detector/tracker node (see `raspbot_hailo_tracking`), the web UI can:

- Draw bounding boxes (pulled from `detections_topic`, JSON on `std_msgs/String`)
- Toggle person tracking (publishes `std_msgs/Bool` to `tracking_enable_topic`)

Defaults:

- `detections_topic`: `detections/json`
- `tracking_enable_topic`: `tracking/enable`

Override topics:

```bash
ros2 run raspbot_web_video web_video --ros-args \
	-p detections_topic:=detections/json \
	-p tracking_enable_topic:=tracking/enable
```
```

## Notes

- This uses only Python stdlib for HTTP (no Flask), so it’s easy to run on stock images.
- If you see a black frame or no updates, verify the ROS camera topic is publishing:

```bash
ros2 topic list | grep image
ros2 topic info /image_raw/compressed
```
