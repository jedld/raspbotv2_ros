import math
import time

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

from .i2c_car import I2CCar

RAD_TO_DEG = 180.0 / math.pi


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
        self.declare_parameter("tilt_neutral_deg", 45.0)

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

        # ── IMU tilt compensation ─────────────────────────────────────
        # When enabled, the robot's pitch (from accelerometer) is added to the
        # tilt servo command so the camera stays level on inclines/during accel.
        self.declare_parameter("imu_tilt_compensation", True)
        self.declare_parameter("imu_topic", "imu/data")
        # How much of the measured pitch to compensate (1.0 = full, 0.0 = off).
        self.declare_parameter("imu_tilt_gain", 1.0)
        # Low-pass filter coefficient for pitch (0..1). Closer to 1 = more smoothing.
        self.declare_parameter("imu_tilt_alpha", 0.85)
        # Sign: +1 or -1, depending on how IMU pitch maps to tilt direction.
        self.declare_parameter("imu_tilt_sign", -1.0)

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

        # ── IMU tilt compensation state ───────────────────────────────
        self._imu_tilt_comp = bool(self.get_parameter("imu_tilt_compensation").value)
        self._imu_tilt_gain = float(self.get_parameter("imu_tilt_gain").value)
        self._imu_tilt_alpha = float(self.get_parameter("imu_tilt_alpha").value)
        self._imu_tilt_sign = float(self.get_parameter("imu_tilt_sign").value)
        self._imu_pitch_deg = 0.0  # filtered pitch from accelerometer
        self._imu_calibrated = False  # gated until imu/calibrated is True

        self.create_subscription(Vector3, topic, self._on_cmd, 10)

        # ── IMU subscription ──────────────────────────────────────────
        if self._imu_tilt_comp:
            imu_topic = str(self.get_parameter("imu_topic").value)
            self.create_subscription(Imu, imu_topic, self._on_imu, 10)
            self.create_subscription(Bool, 'imu/calibrated', self._on_imu_calibrated, 10)
            self.get_logger().info(
                f"IMU tilt compensation ENABLED (gain={self._imu_tilt_gain}, "
                f"alpha={self._imu_tilt_alpha}, sign={self._imu_tilt_sign}). "
                f"Waiting for IMU calibration..."
            )

        period = 1.0 / max(publish_hz, 1e-3)
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f"Camera gimbal ready (addr=0x{i2c_addr:02x}, pan_id={self._pan_id}, tilt_id={self._tilt_id}). "
            f"Command topic: {topic} (x=pan_deg, y=tilt_deg)"
        )

    def _on_imu_calibrated(self, msg: Bool) -> None:
        was = self._imu_calibrated
        self._imu_calibrated = msg.data
        if msg.data and not was:
            self.get_logger().info('IMU calibrated — tilt compensation is now active')
        elif not msg.data and was:
            self.get_logger().warn('IMU calibration lost — tilt compensation suspended')
            self._imu_pitch_deg = 0.0

    def _on_imu(self, msg: Imu) -> None:
        """Derive pitch from accelerometer and low-pass filter it."""
        # Ignore data until gyro calibration is done (accel values may also
        # be noisy during the calibration hold period)
        if not self._imu_calibrated:
            return
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        # Pitch = atan2(ax, sqrt(ay² + az²)) — positive when nose is up
        denom = math.sqrt(ay * ay + az * az)
        if denom < 1e-6:
            return
        raw_pitch_deg = math.atan2(ax, denom) * RAD_TO_DEG

        # Exponential low-pass filter to smooth vibration
        alpha = self._imu_tilt_alpha
        self._imu_pitch_deg = alpha * self._imu_pitch_deg + (1.0 - alpha) * raw_pitch_deg

    def _on_cmd(self, msg: Vector3) -> None:
        pan = float(msg.x)
        tilt = float(msg.y)

        if math.isnan(pan) or math.isnan(tilt):
            return

        self._pan_deg = _clamp(pan, self._pan_min, self._pan_max)
        self._tilt_deg = _clamp(tilt, self._tilt_min, self._tilt_max)
        self._last_cmd_time = time.monotonic()

    def _apply(self, pan_deg: float, tilt_deg: float) -> None:
        # Apply IMU tilt compensation: offset the tilt servo to counteract robot pitch
        if self._imu_tilt_comp:
            compensation = self._imu_tilt_sign * self._imu_pitch_deg * self._imu_tilt_gain
            tilt_deg = _clamp(tilt_deg + compensation, self._tilt_min, self._tilt_max)
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
