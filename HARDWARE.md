# Raspbot V2 hardware notes (known-good)

This document consolidates **known** hardware details for the Yahboom Raspbot V2 as used by this ROS 2 Foxy workspace.

It’s intentionally scoped to what we can confirm from:
- the ROS 2 packages in this repo (`raspbot_hw`, `raspbot_bringup`)
- the vendor Python driver library shipped in this workspace (`Python driver library/py_install`)

If your kit differs (older controller, different pinout, different add-ons), treat these as starting points.

## Platform overview

- Compute: Raspberry Pi (commonly Raspberry Pi 5 for “V2” kits)
- Drive base: 4 independent motors + mecanum wheels (holonomic motion)
- Expansion/controller board: Yahboom motor/sensor controller accessed via I2C

## Available features (confirmed)

From ROS 2 nodes and vendor I2C protocol:

- **Mecanum chassis control** (4 motors)
  - ROS: `motor_driver` node subscribes to `cmd_vel` (`geometry_msgs/Twist`)
  - I2C: per-motor commands supported on the “pi5” protocol

- **Ultrasonic distance**
  - ROS: publishes `ultrasonic/range` (`sensor_msgs/Range`)
  - Supported via two backends:
    - I2C distance registers on “pi5” controller (preferred)
    - GPIO trig/echo (legacy fallback)

- **GPIO sensor module**
  - IR avoid / obstacle: publishes `ir_avoid/state` (`std_msgs/Int32`)
  - Line tracking (4-channel): publishes `tracking/state` (`std_msgs/Int32`)

- **USB camera (optional)**
  - ROS: publishes `image_raw/compressed` (`sensor_msgs/CompressedImage`)
  - Device: typically `/dev/video0`

- **2DOF camera gimbal (pan/tilt servos)**
  - ROS: `camera_gimbal` node subscribes to `camera_gimbal/command_deg`
  - I2C: servo commands on the controller board

- **WS2812 RGB light-bar (14 LEDs)**
  - ROS: `lightbar` node subscribes to `lightbar/command` (`std_msgs/String`, JSON)
  - I2C: registers `0x03`, `0x04`, `0x08`, `0x09` on the pi5 controller
  - Supports: solid colour, per-LED colour, breathing, rainbow, chase effects
  - Also controllable from the web UI (port 8080)

## OS / device prerequisites

Typical device nodes:

- I2C: `/dev/i2c-1`
- GPIO (Pi 5): `/dev/gpiochip*`
- Camera: `/dev/video0`

Typical groups/permissions:

- I2C: group `i2c`
- Camera: group `video`
- GPIO: varies by distro; on Pi 5 the `lgpio` backend needs access to `/dev/gpiochip*`

Pi 5 GPIO note:
- `RPi.GPIO` often fails on Pi 5; these nodes auto-fallback to `lgpio`.

## GPIO pin mapping (BOARD numbering)

These are the defaults used by this workspace (and match the repo README). They are **BOARD** numbering, not BCM.

- Ultrasonic (GPIO fallback):
  - Trig: 16
  - Echo: 18

- IR avoid:
  - ON: 22
  - Left: 21
  - Right: 19

- Line tracking (4-channel):
  - L1: 13
  - L2: 15
  - R1: 11
  - R2: 7

If your sensors behave “dead”, double-check BOARD vs BCM numbering.

## Controller board (I2C) information

### I2C address & protocol

This workspace supports two controller protocols:

- **pi5 protocol** (newer Yahboom Pi 5 kits)
  - I2C address: `0x2B` (decimal `43`)
  - Per-motor commands (4 motors) + ultrasonic registers

Many kits also include a small I2C OLED screen. If you see `0x3C` (decimal `60`) on `i2cdetect -y 1`, it is typically an SSD1306-compatible OLED.

- **legacy protocol** (older kits / older notebooks)
  - I2C address: often `0x16`
  - Left/right drive only (no per-wheel mecanum commands)

The ROS 2 config defaults to:
- motor controller I2C required: `true`
- address: `0x2B`
- protocol: `auto` (resolves to `pi5` when addr is `0x2B`)

### Known register map (pi5 protocol)

The following are derived from the vendor Python library in `Python driver library/py_install`.

- Motor command: register `0x01`
  - payload: `[motor_id, motor_dir, motor_speed]`
  - motor_id: `0=L1, 1=L2, 2=R1, 3=R2`
  - motor_dir: `0=forward, 1=backward`
  - motor_speed: `0..255`

- Servo command: register `0x02`
  - payload: `[servo_id, angle_deg]`
  - Typical IDs: `1=pan`, `2=tilt` (tilt often limited by firmware)

- WS2812 / RGB light bar:
  - All on/off: register `0x03` payload `[state, color]`
  - Single LED: register `0x04` payload `[index, state, color]`
  - Brightness (all): register `0x08` payload `[R, G, B]`
  - Brightness (single): register `0x09` payload `[index, R, G, B]`

- IR receiver enable: register `0x05` payload `[state]`
- Buzzer enable: register `0x06` payload `[state]`

- Ultrasonic enable: register `0x07` payload `[state]`
  - `state=1` enable measurement, `state=0` disable

- Ultrasonic distance (millimeters): registers `0x1A` (low byte) and `0x1B` (high byte)
  - `mm = (H << 8) | L`

## ROS 2 interfaces (this workspace)

Bringup: `ros2 launch raspbot_bringup bringup.launch.py`

Nodes:
- `/motor_driver`
- `/ultrasonic`
- `/gpio_sensors`
- `/opencv_camera` (optional)
- `/camera_gimbal`
- `/lightbar`
- `/oled` (optional)

Topics:
- `/cmd_vel` (subscribed by motor driver)
- `/ultrasonic/range` (`sensor_msgs/Range`)
- `/ir_avoid/state` (`std_msgs/Int32`, bit0=left, bit1=right)
- `/tracking/state` (`std_msgs/Int32`, bit0=L1, bit1=L2, bit2=R1, bit3=R2)
- `/image_raw/compressed` (`sensor_msgs/CompressedImage`, optional)
- `/camera_gimbal/command_deg` (`geometry_msgs/Vector3`, degrees)
- `/lightbar/command` (`std_msgs/String`, JSON – see lightbar_node.py for schema)
- `/oled/text` (`std_msgs/String`, optional)

## Quick board sanity checks

I2C controller presence:

```bash
ls -l /dev/i2c-1
sudo i2cdetect -y 1  # expect to see 2b for pi5 protocol boards

# If you have an OLED screen, you will often also see 3c on the same bus.
# Example: ... 2b ... 3c ...
```

Read ultrasonic registers directly (bypasses ROS 2):

```bash
python3 - <<'PY'
import time
import smbus
bus = smbus.SMBus(1)
addr = 0x2B
for i in range(20):
    l = bus.read_byte_data(addr, 0x1A)
    h = bus.read_byte_data(addr, 0x1B)
    mm = (h << 8) | l
    print(i, mm)
    time.sleep(0.1)
PY
```

If ROS 2 ultrasonic is publishing `.inf`, it usually means the driver returned `NaN` (GPIO echo timeout or invalid I2C reading).

## References

- Vendor course/download hub: https://www.yahboom.net/study/RASPBOT-V2
- Vendor Python I2C driver (in this workspace): `Python driver library/py_install/Raspbot_Lib/Raspbot_Lib.py`
- ROS 2 config used by this workspace: `raspbot_hw/config/raspbot_hw.yaml`
