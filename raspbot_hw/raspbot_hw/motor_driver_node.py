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
        self.declare_parameter('max_linear_velocity', 0.35)
        self.declare_parameter('wheel_separation', 0.18)
        self.declare_parameter('cmd_vel_timeout_sec', 0.5)
        self.declare_parameter('idle_stop_period_sec', 1.0)

        i2c_bus = int(self.get_parameter('i2c_bus').value)
        i2c_addr = int(self.get_parameter('i2c_addr').value)
        i2c_protocol = str(self.get_parameter('i2c_protocol').value)
        i2c_required = bool(self.get_parameter('i2c_required').value)
        dry_run = bool(self.get_parameter('dry_run').value)

        self._max_pwm = int(self.get_parameter('max_pwm').value)
        self._max_linear = float(self.get_parameter('max_linear_velocity').value)
        self._wheel_sep = float(self.get_parameter('wheel_separation').value)
        self._timeout = float(self.get_parameter('cmd_vel_timeout_sec').value)
        self._idle_stop_period = float(self.get_parameter('idle_stop_period_sec').value)

        self._car = I2CCar(i2c_bus=i2c_bus, i2c_addr=i2c_addr, dry_run=dry_run, protocol=i2c_protocol)

        self._last_cmd_time = None
        self._last_idle_stop_time = 0.0
        self._left_pwm = 0.0
        self._right_pwm = 0.0

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

    def _on_cmd_vel(self, msg: Twist) -> None:
        v = float(msg.linear.x)
        w = float(msg.angular.z)

        # Differential drive kinematics
        v_l = v - (w * self._wheel_sep / 2.0)
        v_r = v + (w * self._wheel_sep / 2.0)

        # Map m/s to PWM using max_linear_velocity
        if self._max_linear <= 0.0:
            self._left_pwm = 0.0
            self._right_pwm = 0.0
        else:
            self._left_pwm = clamp(v_l / self._max_linear * self._max_pwm, -self._max_pwm, self._max_pwm)
            self._right_pwm = clamp(v_r / self._max_linear * self._max_pwm, -self._max_pwm, self._max_pwm)

        self._last_cmd_time = time.time()

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

        try:
            self._car.set_wheel_pwms(self._left_pwm, self._right_pwm, max_pwm=self._max_pwm)
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
