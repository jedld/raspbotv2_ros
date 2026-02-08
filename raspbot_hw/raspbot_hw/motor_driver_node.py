import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from .i2c_car import I2CCar


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver')

        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_addr', 0x16)
        self.declare_parameter('i2c_protocol', 'auto')
        self.declare_parameter('i2c_required', True)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('max_pwm', 100)
        self.declare_parameter('min_pwm', 0)  # 0 disables; helps overcome static friction
        self.declare_parameter('max_linear_velocity', 0.35)
        self.declare_parameter('wheel_separation', 0.18)
        self.declare_parameter('write_hz', 20.0)  # I2C write rate (lower reduces bus load)
        self.declare_parameter('drive_mode', 'differential')  # differential|mecanum
        self.declare_parameter('wheelbase', 0.18)  # m (front-to-rear)
        self.declare_parameter('track_width', 0.18)  # m (left-to-right)

        # pi5 protocol motor IDs (defaults match vendor docs: 0=L1,1=L2,2=R1,3=R2)
        # We assume L1=front_left, L2=rear_left, R1=front_right, R2=rear_right.
        self.declare_parameter('motor_id_fl', 0)
        self.declare_parameter('motor_id_rl', 1)
        self.declare_parameter('motor_id_fr', 2)
        self.declare_parameter('motor_id_rr', 3)
        self.declare_parameter('invert_fl', False)
        self.declare_parameter('invert_fr', False)
        self.declare_parameter('invert_rl', False)
        self.declare_parameter('invert_rr', False)
        self.declare_parameter('cmd_vel_timeout_sec', 0.5)
        self.declare_parameter('idle_stop_period_sec', 1.0)

        i2c_bus = int(self.get_parameter('i2c_bus').value)
        i2c_addr = int(self.get_parameter('i2c_addr').value)
        i2c_protocol = str(self.get_parameter('i2c_protocol').value)
        i2c_required = bool(self.get_parameter('i2c_required').value)
        dry_run = bool(self.get_parameter('dry_run').value)

        self._max_pwm = int(self.get_parameter('max_pwm').value)
        self._min_pwm = int(self.get_parameter('min_pwm').value)
        self._max_linear = float(self.get_parameter('max_linear_velocity').value)
        self._wheel_sep = float(self.get_parameter('wheel_separation').value)

        self._write_hz = float(self.get_parameter('write_hz').value)
        self._min_write_period = 1.0 / max(self._write_hz, 1e-3)

        self._drive_mode = str(self.get_parameter('drive_mode').value).strip().lower()
        self._wheelbase = float(self.get_parameter('wheelbase').value)
        self._track_width = float(self.get_parameter('track_width').value)

        self._motor_id_fl = int(self.get_parameter('motor_id_fl').value)
        self._motor_id_fr = int(self.get_parameter('motor_id_fr').value)
        self._motor_id_rl = int(self.get_parameter('motor_id_rl').value)
        self._motor_id_rr = int(self.get_parameter('motor_id_rr').value)

        self._invert_fl = bool(self.get_parameter('invert_fl').value)
        self._invert_fr = bool(self.get_parameter('invert_fr').value)
        self._invert_rl = bool(self.get_parameter('invert_rl').value)
        self._invert_rr = bool(self.get_parameter('invert_rr').value)
        self._timeout = float(self.get_parameter('cmd_vel_timeout_sec').value)
        self._idle_stop_period = float(self.get_parameter('idle_stop_period_sec').value)

        self._car = I2CCar(i2c_bus=i2c_bus, i2c_addr=i2c_addr, dry_run=dry_run, protocol=i2c_protocol)

        self._last_cmd_time = None
        self._last_idle_stop_time = 0.0
        self._last_write_time = 0.0
        self._left_pwm = 0.0
        self._right_pwm = 0.0

        self._fl_pwm = 0.0
        self._fr_pwm = 0.0
        self._rl_pwm = 0.0
        self._rr_pwm = 0.0

        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)
        self._timer = self.create_timer(0.02, self._tick)  # 50Hz

        if dry_run:
            self.get_logger().warn('Motor driver running in dry_run mode (no I2C writes).')
        else:
            try:
                self._car.stop()
            except Exception as e:
                msg = f'I2C motor controller not reachable (bus={i2c_bus}, addr=0x{i2c_addr:02x}): {e!r}'
                if i2c_required:
                    raise RuntimeError(msg) from e
                self.get_logger().error(msg)

        self.get_logger().info(
            f'I2C motor driver ready (bus={i2c_bus}, addr=0x{i2c_addr:02x}, max_pwm={self._max_pwm})'
        )

        if self._car.protocol == 'pi5':
            self.get_logger().info(
                'Motor mapping (pi5): '
                f'FL=id{self._motor_id_fl} inv={self._invert_fl}, '
                f'FR=id{self._motor_id_fr} inv={self._invert_fr}, '
                f'RL=id{self._motor_id_rl} inv={self._invert_rl}, '
                f'RR=id{self._motor_id_rr} inv={self._invert_rr}'
            )

        if self._drive_mode not in {'differential', 'mecanum'}:
            self.get_logger().warn(
                f"Unknown drive_mode '{self._drive_mode}', falling back to differential."
            )
            self._drive_mode = 'differential'

        if self._drive_mode == 'mecanum' and self._car.protocol != 'pi5':
            self.get_logger().warn(
                'drive_mode=mecanum requested but I2C protocol is not pi5; '
                'strafe will be ignored and controller will behave like differential.'
            )

    def _on_cmd_vel(self, msg: Twist) -> None:
        vx = float(msg.linear.x)
        vy = float(getattr(msg.linear, 'y', 0.0))
        wz = float(msg.angular.z)

        if self._max_linear <= 0.0:
            self._left_pwm = 0.0
            self._right_pwm = 0.0
            self._fl_pwm = 0.0
            self._fr_pwm = 0.0
            self._rl_pwm = 0.0
            self._rr_pwm = 0.0
            self._last_cmd_time = time.time()
            return

        if self._drive_mode == 'mecanum' and self._car.protocol == 'pi5':
            # ROS base_link: x forward, y left, z up.
            lx = 0.5 * float(self._wheelbase if self._wheelbase > 0.0 else self._wheel_sep)
            ly = 0.5 * float(self._track_width if self._track_width > 0.0 else self._wheel_sep)
            k = (lx + ly)

            # Wheel "linear" commands (m/s at wheel perimeter), scaled to max_linear_velocity.
            v_fl = vx - vy - (k * wz)
            v_fr = vx + vy + (k * wz)
            v_rl = vx + vy - (k * wz)
            v_rr = vx - vy + (k * wz)

            max_abs = max(abs(v_fl), abs(v_fr), abs(v_rl), abs(v_rr), 1e-6)
            if max_abs > self._max_linear:
                scale = self._max_linear / max_abs
                v_fl *= scale
                v_fr *= scale
                v_rl *= scale
                v_rr *= scale

            fl_pwm = v_fl / self._max_linear * self._max_pwm
            fr_pwm = v_fr / self._max_linear * self._max_pwm
            rl_pwm = v_rl / self._max_linear * self._max_pwm
            rr_pwm = v_rr / self._max_linear * self._max_pwm

            if self._invert_fl:
                fl_pwm *= -1.0
            if self._invert_fr:
                fr_pwm *= -1.0
            if self._invert_rl:
                rl_pwm *= -1.0
            if self._invert_rr:
                rr_pwm *= -1.0

            self._fl_pwm = clamp(fl_pwm, -self._max_pwm, self._max_pwm)
            self._fr_pwm = clamp(fr_pwm, -self._max_pwm, self._max_pwm)
            self._rl_pwm = clamp(rl_pwm, -self._max_pwm, self._max_pwm)
            self._rr_pwm = clamp(rr_pwm, -self._max_pwm, self._max_pwm)

            if self._min_pwm > 0:
                self._fl_pwm = self._apply_min_pwm(self._fl_pwm)
                self._fr_pwm = self._apply_min_pwm(self._fr_pwm)
                self._rl_pwm = self._apply_min_pwm(self._rl_pwm)
                self._rr_pwm = self._apply_min_pwm(self._rr_pwm)
        else:
            # Differential drive kinematics (vy ignored)
            v_l = vx - (wz * self._wheel_sep / 2.0)
            v_r = vx + (wz * self._wheel_sep / 2.0)

            self._left_pwm = clamp(v_l / self._max_linear * self._max_pwm, -self._max_pwm, self._max_pwm)
            self._right_pwm = clamp(v_r / self._max_linear * self._max_pwm, -self._max_pwm, self._max_pwm)

            if self._min_pwm > 0:
                self._left_pwm = self._apply_min_pwm(self._left_pwm)
                self._right_pwm = self._apply_min_pwm(self._right_pwm)

        self._last_cmd_time = time.time()

    def _apply_min_pwm(self, pwm: float) -> float:
        if pwm == 0.0:
            return 0.0
        sign = 1.0 if pwm > 0.0 else -1.0
        mag = abs(pwm)
        if 0.0 < mag < float(self._min_pwm):
            return sign * float(self._min_pwm)
        return pwm

    def _tick(self) -> None:
        now = time.time()
        if self._last_cmd_time is None:
            if self._idle_stop_period > 0.0 and (now - self._last_idle_stop_time) > self._idle_stop_period:
                try:
                    self._car.stop()
                except Exception as e:
                    self.get_logger().warn(f'Failed to stop motors (idle): {e!r}')
                self._last_idle_stop_time = now
            return

        if (now - self._last_cmd_time) > self._timeout:
            if self._idle_stop_period > 0.0 and (now - self._last_idle_stop_time) > self._idle_stop_period:
                try:
                    self._car.stop()
                except Exception as e:
                    self.get_logger().warn(f'Failed to stop motors (timeout): {e!r}')
                self._last_idle_stop_time = now
            return

        # Rate-limit I2C writes to reduce bus load and keep wheel updates consistent.
        if (now - self._last_write_time) < self._min_write_period:
            return

        try:
            if self._drive_mode == 'mecanum' and self._car.protocol == 'pi5':
                self._car.set_motor_pwms(
                    self._fl_pwm,
                    self._fr_pwm,
                    self._rl_pwm,
                    self._rr_pwm,
                    max_pwm=self._max_pwm,
                    motor_id_fl=self._motor_id_fl,
                    motor_id_fr=self._motor_id_fr,
                    motor_id_rl=self._motor_id_rl,
                    motor_id_rr=self._motor_id_rr,
                )
            else:
                self._car.set_wheel_pwms(self._left_pwm, self._right_pwm, max_pwm=self._max_pwm)
            self._last_write_time = now
        except Exception as e:
            self.get_logger().error(f'I2C write failed: {e!r}')

    def destroy_node(self):
        try:
            self._car.stop()
            self._car.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
