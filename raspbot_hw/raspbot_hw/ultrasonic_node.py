import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

from .gpio_backend import GpioConfig, create_gpio


class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic')

        self.declare_parameter('gpio_backend', 'auto')
        self.declare_parameter('gpio_mode', 'BOARD')
        self.declare_parameter('lgpio_chip', -1)
        self.declare_parameter('trig_pin', 16)
        self.declare_parameter('echo_pin', 18)
        self.declare_parameter('frame_id', 'ultrasonic_link')
        self.declare_parameter('min_range_m', 0.02)
        self.declare_parameter('max_range_m', 4.0)
        self.declare_parameter('publish_hz', 10.0)

        self._frame_id = str(self.get_parameter('frame_id').value)
        self._min_range = float(self.get_parameter('min_range_m').value)
        self._max_range = float(self.get_parameter('max_range_m').value)
        publish_hz = float(self.get_parameter('publish_hz').value)

        self._trig = int(self.get_parameter('trig_pin').value)
        self._echo = int(self.get_parameter('echo_pin').value)
        gpio_mode = str(self.get_parameter('gpio_mode').value).upper()

        gpio_backend = str(self.get_parameter('gpio_backend').value)
        lgpio_chip = int(self.get_parameter('lgpio_chip').value)

        cfg = GpioConfig(backend=gpio_backend, mode=gpio_mode, lgpio_chip=lgpio_chip)

        self._gpio = create_gpio(cfg, logger=self.get_logger())
        try:
            self._gpio.setup_in(self._echo)
            self._gpio.setup_out(self._trig, initial=False)
        except Exception as e:
            if cfg.backend.lower().strip() == 'auto':
                self.get_logger().warn(f'GPIO init failed ({e!r}); retrying with lgpio backend')
                try:
                    self._gpio.cleanup()
                except Exception:
                    pass
                self._gpio = create_gpio(GpioConfig(backend='lgpio', mode=gpio_mode, lgpio_chip=lgpio_chip))
                self._gpio.setup_in(self._echo)
                self._gpio.setup_out(self._trig, initial=False)
            else:
                raise

        self._pub = self.create_publisher(Range, 'ultrasonic/range', 10)
        period = 1.0 / max(publish_hz, 1e-3)
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f'Ultrasonic ready (mode={gpio_mode}, trig={self._trig}, echo={self._echo}, hz={publish_hz})'
        )

    def _distance_m(self) -> float:
        # Based on Yahboom notebook implementation.
        gpio = self._gpio

        gpio.write(self._trig, False)
        time.sleep(0.000002)
        gpio.write(self._trig, True)
        time.sleep(0.000015)
        gpio.write(self._trig, False)

        timeout_s = 0.03
        t_start = time.monotonic()
        while not gpio.read(self._echo):
            if (time.monotonic() - t_start) > timeout_s:
                return float('nan')
        t1 = time.monotonic()

        while gpio.read(self._echo):
            if (time.monotonic() - t1) > timeout_s:
                return float('nan')
        t2 = time.monotonic()

        # Speed of sound ~340 m/s, the notebook returns centimeters
        distance_cm = ((t2 - t1) * 340.0 / 2.0) * 100.0
        return distance_cm / 100.0

    def _tick(self) -> None:
        d = self._distance_m()

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.5
        msg.min_range = float(self._min_range)
        msg.max_range = float(self._max_range)

        if d != d:  # NaN
            msg.range = float('inf')
        else:
            msg.range = float(d)

        self._pub.publish(msg)

    def destroy_node(self):
        try:
            self._gpio.cleanup()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
