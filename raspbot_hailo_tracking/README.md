# raspbot_hailo_tracking

Runs a **local** Hailo object-detection HEF on the Hailo-8 accelerator and publishes:

- `detections/json` (`std_msgs/String`): JSON bounding boxes in *pixel* coordinates
- `camera_gimbal/command_deg` (`geometry_msgs/Vector3`): optional gimbal commands to keep a person centered

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

### Useful parameters

- `hef_path` (string, required)
- `labels_path` (string, optional): newline-delimited labels; used for `label` in JSON and to auto-detect the `person` class id
- `score_threshold` (double): filters detections + sets NMS score threshold
- `iou_threshold` (double): sets NMS IoU threshold
- `inference_fps` (double): throttles inference
- `person_class_id` (int): set if your model uses a known class id for person
- `kp_pan_deg`, `kp_tilt_deg`, `max_step_deg`, `deadband_norm`: tracking controller tuning
- `pan_sign`, `tilt_sign`: flip if tracking moves the wrong direction

Tip: if the camera consistently turns away from the person, set `pan_sign:=-1` (and/or `tilt_sign:=-1`).

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
