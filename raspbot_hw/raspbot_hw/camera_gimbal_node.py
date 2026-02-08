import math
import time

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node

from .i2c_car import I2CCar


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class CameraGimbalNode(Node):
    def __init__(self):
        super().__init__("camera_gimbal")

        self._start_time = time.monotonic()

        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("i2c_addr", 0x2B)
        self.declare_parameter("i2c_protocol", "auto")
        self.declare_parameter("i2c_required", True)
        self.declare_parameter("dry_run", False)

        self.declare_parameter("pan_servo_id", 1)
        self.declare_parameter("tilt_servo_id", 2)

        self.declare_parameter("pan_neutral_deg", 90.0)
        self.declare_parameter("tilt_neutral_deg", 90.0)

        self.declare_parameter("pan_min_deg", 0.0)
        self.declare_parameter("pan_max_deg", 180.0)
        # Safe default for many Raspbot builds; increase if your tilt servo supports it.
        self.declare_parameter("tilt_min_deg", 0.0)
        self.declare_parameter("tilt_max_deg", 110.0)

        self.declare_parameter("command_topic", "camera_gimbal/command_deg")
        self.declare_parameter("startup_delay_sec", 0.5)
        self.declare_parameter("reset_on_startup", True)
        self.declare_parameter("reset_on_shutdown", True)
        self.declare_parameter("command_timeout_sec", 0.0)  # <=0 disables
        self.declare_parameter("publish_hz", 20.0)

        i2c_bus = int(self.get_parameter("i2c_bus").value)
        i2c_addr = int(self.get_parameter("i2c_addr").value)
        i2c_protocol = str(self.get_parameter("i2c_protocol").value)
        i2c_required = bool(self.get_parameter("i2c_required").value)
        dry_run = bool(self.get_parameter("dry_run").value)

        self._i2c_required = i2c_required

        self._pan_id = int(self.get_parameter("pan_servo_id").value)
        self._tilt_id = int(self.get_parameter("tilt_servo_id").value)

        self._pan_neutral = float(self.get_parameter("pan_neutral_deg").value)
        self._tilt_neutral = float(self.get_parameter("tilt_neutral_deg").value)

        self._pan_min = float(self.get_parameter("pan_min_deg").value)
        self._pan_max = float(self.get_parameter("pan_max_deg").value)
        self._tilt_min = float(self.get_parameter("tilt_min_deg").value)
        self._tilt_max = float(self.get_parameter("tilt_max_deg").value)

        self._startup_delay = float(self.get_parameter("startup_delay_sec").value)
        self._reset_on_startup = bool(self.get_parameter("reset_on_startup").value)
        self._reset_on_shutdown = bool(self.get_parameter("reset_on_shutdown").value)

        self._cmd_timeout = float(self.get_parameter("command_timeout_sec").value)
        publish_hz = float(self.get_parameter("publish_hz").value)

        topic = str(self.get_parameter("command_topic").value)

        self._car = I2CCar(i2c_bus=i2c_bus, i2c_addr=i2c_addr, dry_run=dry_run, protocol=i2c_protocol)

        if dry_run:
            self.get_logger().warn("Camera gimbal running in dry_run mode (no I2C writes).")

        self._pan_deg = self._pan_neutral
        self._tilt_deg = self._tilt_neutral
        self._last_cmd_time = None
        self._did_startup_reset = False

        if not self._reset_on_startup:
            self._did_startup_reset = True

        self.create_subscription(Vector3, topic, self._on_cmd, 10)

        period = 1.0 / max(publish_hz, 1e-3)
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f"Camera gimbal ready (addr=0x{i2c_addr:02x}, pan_id={self._pan_id}, tilt_id={self._tilt_id}). "
            f"Command topic: {topic} (x=pan_deg, y=tilt_deg)"
        )

    def _on_cmd(self, msg: Vector3) -> None:
        pan = float(msg.x)
        tilt = float(msg.y)

        if math.isnan(pan) or math.isnan(tilt):
            return

        self._pan_deg = _clamp(pan, self._pan_min, self._pan_max)
        self._tilt_deg = _clamp(tilt, self._tilt_min, self._tilt_max)
        self._last_cmd_time = time.monotonic()

    def _apply(self, pan_deg: float, tilt_deg: float) -> None:
        self._car.set_servo(self._pan_id, pan_deg)
        self._car.set_servo(self._tilt_id, tilt_deg)

    def _tick(self) -> None:
        now = time.monotonic()

        if self._reset_on_startup and not self._did_startup_reset:
            if (now - self._start_time) >= self._startup_delay:
                try:
                    self._apply(self._pan_neutral, self._tilt_neutral)
                    self._did_startup_reset = True
                    self.get_logger().info(
                        f"Gimbal reset to neutral (pan={self._pan_neutral:.0f}, tilt={self._tilt_neutral:.0f})"
                    )
                except Exception as e:
                    msg = f"Failed to reset gimbal to neutral: {e!r}"
                    if self._i2c_required:
                        self.get_logger().fatal(msg)
                        rclpy.shutdown()
                        return
                    self.get_logger().warn(msg)
            return

        if self._cmd_timeout > 0.0:
            if self._last_cmd_time is None or (now - self._last_cmd_time) > self._cmd_timeout:
                try:
                    self._apply(self._pan_neutral, self._tilt_neutral)
                except Exception as e:
                    self.get_logger().warn(f"Failed to hold neutral: {e!r}")
                return

        try:
            self._apply(self._pan_deg, self._tilt_deg)
        except Exception as e:
            msg = f"I2C servo write failed: {e!r}"
            if self._i2c_required:
                self.get_logger().fatal(msg)
                rclpy.shutdown()
                return
            self.get_logger().error(msg)

    def destroy_node(self):
        if self._reset_on_shutdown:
            try:
                self._apply(self._pan_neutral, self._tilt_neutral)
            except Exception:
                pass
        try:
            self._car.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = CameraGimbalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
