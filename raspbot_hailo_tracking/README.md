# raspbot_hailo_tracking

Runs a **local** Hailo object-detection HEF on the Hailo-8 accelerator and publishes:

- `detections/json` (`std_msgs/String`): JSON bounding boxes in *pixel* coordinates
- `camera_gimbal/command_deg` (`geometry_msgs/Vector3`): optional gimbal commands to keep a person centered
- `cmd_vel` (`geometry_msgs/Twist`): optional robot base velocity commands for auto-follow (subject following)

## Prereqs

- HailoRT runtime + driver installed (Debian packages): `hailort`, `hailort-pcie-driver`
- A Hailo **HEF** file for an object-detection network that outputs **NMS** (this node expects `HAILO_NMS_BY_SCORE`).

## Recommended open-source model

For person tracking + general object boxes, a good default is a small COCO detector:

- **YOLOX-S (COCO)** (`yolox_s_leaky`): fast and accurate enough for tracking, includes the `person` class (COCO class id `0`).
  - Upstream license: Apache-2.0 (recommended default for permissive licensing)
  - Source: https://github.com/Megvii-BaseDetection/YOLOX

Alternative (very fast, lower accuracy):

- **SSD MobileNet v1 (COCO)** (`ssd_mobilenet_v1`)
  - Upstream license: Apache-2.0
  - Source: https://github.com/tensorflow/models

The HEF is downloaded from the Hailo Model Zoo public Hailo-8 compiled models list.

### Download

This package ships a helper that downloads the precompiled HEF into a standard local directory (`~/.local/share/raspbot/models/hailo8` by default):

```bash
ros2 run raspbot_hailo_tracking download_hailo_model --model yolox_s_leaky
```

It prints the resulting `hef_path` and `labels_path` you can paste into the launch command.

## Build

```bash
cd ~/ros2_foxy
colcon build --packages-select raspbot_hailo_tracking
source install/local_setup.bash
```

## Run

```bash
source ~/ros2_foxy/install/local_setup.bash
ros2 launch raspbot_hailo_tracking hailo_tracking.launch.py \
  hef_path:=/path/to/your_model.hef \
  labels_path:=/path/to/labels.txt \
  input_topic:=/image_raw/compressed

# (labels_path is optional but recommended for nicer JSON labels + auto person-id)
```

### Tracking toggle

Tracking is controlled via a topic so the web UI can enable/disable it:

- `tracking/enable` (`std_msgs/Bool`)

Example:

```bash
ros2 topic pub --once /tracking/enable std_msgs/msg/Bool "{data: true}"
```

### Auto-follow toggle

Auto-follow makes the **robot base** move to keep the subject at a set distance:

- `follow/enable` (`std_msgs/Bool`)

Example:

```bash
# Enable follow (robot will start moving toward detected person)
ros2 topic pub --once /follow/enable std_msgs/msg/Bool "{data: true}"

# Disable follow (robot stops immediately)
ros2 topic pub --once /follow/enable std_msgs/msg/Bool "{data: false}"
```

You can enable both gimbal tracking and auto-follow simultaneously.  When both
are active, the gimbal keeps the subject centered in the camera while the robot
base rotates to bring the gimbal back to its neutral position (extending
effective yaw range beyond the gimbal's physical limits).  Forward/backward
movement is controlled by comparing the subject's bounding-box area to the
configured target area.

### Useful parameters

- `hef_path` (string, required)
- `labels_path` (string, optional): newline-delimited labels; used for `label` in JSON and to auto-detect the `person` class id
- `score_threshold` (double): filters detections + sets NMS score threshold
- `iou_threshold` (double): sets NMS IoU threshold
- `inference_fps` (double): throttles inference
- `person_class_id` (int): set if your model uses a known class id for person
- `kp_pan_deg`, `kp_tilt_deg`, `max_step_deg`, `deadband_norm`: tracking controller tuning
- `pan_sign`, `tilt_sign`: flip if tracking moves the wrong direction

**Auto-follow parameters:**

- `follow_enabled` (bool, default `false`): start with follow active
- `follow_cmd_vel_topic` (string, default `cmd_vel`): Twist topic for robot base
- `follow_enable_topic` (string, default `follow/enable`): Bool topic to toggle follow
- `follow_target_bbox_area` (double, default `0.12`): desired bounding-box area (normalised 0–1); the robot drives forward/backward to maintain this size
- `follow_kp_linear`, `follow_ki_linear`, `follow_kd_linear`: PID gains for forward/backward speed
- `follow_kp_angular`, `follow_ki_angular`, `follow_kd_angular`: PID gains for yaw rotation
- `follow_max_linear` (double, default `0.3` m/s): linear velocity safety limit
- `follow_max_angular` (double, default `0.8` rad/s): angular velocity safety limit
- `follow_linear_deadband` (double, default `0.02`): area-error deadband (ignore small distance errors)
- `follow_angular_deadband` (double, default `0.05`): centre-error deadband (ignore small angular errors)
- `follow_lost_timeout_sec` (double, default `1.0`): seconds without a detection before the robot enters scan-to-reacquire
- `follow_use_gimbal_feedback` (bool, default `true`): when gimbal tracking is also active, blend gimbal pan offset with camera error for coordinated base+gimbal control
- `follow_gimbal_recenter_kp` (double, default `0.10`): spring gain that pulls the gimbal toward neutral when follow is active — this biases yaw work to the robot base so the gimbal stays near centre for fast corrections (0 = no spring, 1 = immediate snap to neutral)
- `follow_angular_blend` (double, default `0.5`): blend factor between camera error and gimbal offset for base yaw (0.0 = pure camera, 1.0 = pure gimbal offset, 0.5 = equal mix)
- `follow_scan_speed_deg` (double, default `30.0`): gimbal sweep speed in degrees/sec during scan-to-reacquire
- `follow_scan_pause_sec` (double, default `0.4`): (reserved) pause at each sweep edge
- `follow_scan_max_sweeps` (int, default `3`): number of left/right sweeps before giving up and returning gimbal to neutral

**Ultrasonic obstacle avoidance:**

- `follow_ultrasonic_topic` (string, default `ultrasonic/range`): topic name for `sensor_msgs/Range` from the ultrasonic sensor
- `follow_obstacle_stop_m` (double, default `0.20`): distance below which forward motion is completely blocked
- `follow_obstacle_slow_m` (double, default `0.50`): distance below which forward speed is linearly scaled down (between stop and slow distances)

### Coordinated gimbal + base movement

When both gimbal tracking and auto-follow are enabled, the system uses a coordinated control strategy:

1. **Gimbal** (fast): Tracks the person to keep the bounding box roughly centred in the camera, but a *re-centering spring* (`follow_gimbal_recenter_kp`) biases the gimbal back toward its neutral position each frame. This intentionally leaves a small residual camera error.

2. **Base yaw** (slower): Driven by a *blend* of the residual camera error and the gimbal offset from neutral (`follow_angular_blend`). The camera error gives fast initial reaction; the gimbal offset provides a steady-state signal that "unwinds" the gimbal back to centre.

3. **Net effect**: The robot base handles the heavy yaw rotation while the gimbal makes quick fine corrections. The gimbal stays near neutral and available for fast transient tracking, and the camera stays pointed at the subject even during turns.

### Scan-to-reacquire behaviour

When the follow target is lost for longer than `follow_lost_timeout_sec`, the robot stops and the gimbal begins sweeping left ↔ right to search for the person. If the target is re-detected during a sweep, normal follow-tracking resumes immediately. After `follow_scan_max_sweeps` full sweeps without success the gimbal returns to its neutral position and the scan ends.

### Ultrasonic obstacle avoidance

The robot subscribes to the ultrasonic range sensor. When an obstacle is detected:
- Closer than `follow_obstacle_stop_m` (default 20 cm): **forward motion is blocked** (angular rotation still allowed so the robot can turn away)
- Between `follow_obstacle_slow_m` and `follow_obstacle_stop_m`: forward speed is **linearly scaled down**
- Backward motion is never gated (the robot can always reverse away from obstacles)

### Tuning the follow controller

1. Start with conservative gains: `follow_kp_linear:=0.3 follow_kp_angular:=0.5`
2. Increase Kp until the robot tracks smoothly without overshooting.
3. Add Ki (start ≈ 0.02) if there is steady-state offset (robot stops too far/close).
4. Add Kd (start ≈ 0.05) if the response oscillates.
5. Adjust `follow_target_bbox_area` by standing at the desired distance and reading the bounding-box area from `detections/json`.
6. If the robot turns the wrong way, check the sign of `follow_kp_angular` or flip `pan_sign`.
7. If the gimbal overshoots its neutral and the base lags behind, increase `follow_gimbal_recenter_kp` (e.g. 0.15–0.25).
8. If the base yaw is sluggish, decrease `follow_angular_blend` toward 0.0 to weight camera error more heavily.

## JSON schema

Published on `detections_topic` (default `detections/json`):

```json
{
  "image_width": 640,
  "image_height": 480,
  "stamp": {"sec": 0, "nanosec": 0},
  "detections": [
    {"class_id": 0, "label": "person", "score": 0.93, "x": 123, "y": 45, "w": 80, "h": 160}
  ]
}
```
