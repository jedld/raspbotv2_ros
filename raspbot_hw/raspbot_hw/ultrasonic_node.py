import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

from .gpio_backend import GpioConfig, create_gpio
from .i2c_car import I2CCar


class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic')

        # mode: auto (pi5->i2c, legacy->gpio), i2c (force), gpio (force)
        self.declare_parameter('mode', 'auto')

        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_addr', 0x2B)
        self.declare_parameter('i2c_protocol', 'auto')
        self.declare_parameter('i2c_required', False)
        self.declare_parameter('dry_run', False)

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

        mode = str(self.get_parameter('mode').value).strip().lower()
        i2c_bus = int(self.get_parameter('i2c_bus').value)
        i2c_addr = int(self.get_parameter('i2c_addr').value)
        i2c_protocol = str(self.get_parameter('i2c_protocol').value)
        i2c_required = bool(self.get_parameter('i2c_required').value)
        dry_run = bool(self.get_parameter('dry_run').value)

        self._use_i2c = False
        self._car = None
        self._gpio = None

        # Decide mode
        if mode not in {'auto', 'i2c', 'gpio'}:
            self.get_logger().warn(f"Unknown ultrasonic mode '{mode}', falling back to auto")
            mode = 'auto'

        if mode in {'auto', 'i2c'}:
            try:
                car = I2CCar(i2c_bus=i2c_bus, i2c_addr=i2c_addr, dry_run=dry_run, protocol=i2c_protocol)
                if car.protocol == 'pi5' and mode in {'auto', 'i2c'}:
                    self._car = car
                    self._use_i2c = True
                    if not dry_run:
                        try:
                            self._car.set_ultrasonic_enabled(True)
                        except Exception as e:
                            if i2c_required or mode == 'i2c':
                                raise
                            self.get_logger().warn(f'Failed to enable I2C ultrasonic: {e!r}; falling back to GPIO')
                            self._use_i2c = False
                            self._car = None
            except Exception as e:
                if i2c_required or mode == 'i2c':
                    raise
                self.get_logger().warn(f'I2C ultrasonic init failed: {e!r}; falling back to GPIO')

        if not self._use_i2c:
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

        if self._use_i2c:
            self.get_logger().info(f'Ultrasonic ready (I2C mode, hz={publish_hz})')
        else:
            self.get_logger().info(
                f'Ultrasonic ready (GPIO mode={gpio_mode}, trig={self._trig}, echo={self._echo}, hz={publish_hz})'
            )

    def _distance_m(self) -> float:
        if self._use_i2c and self._car is not None:
            return self._car.read_ultrasonic_m()

        # GPIO trig/echo (legacy / notebook implementation).
        gpio = self._gpio
        if gpio is None:
            return float('nan')

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
            if self._car is not None:
                try:
                    self._car.set_ultrasonic_enabled(False)
                except Exception:
                    pass
                try:
                    self._car.close()
                except Exception:
                    pass
            if self._gpio is not None:
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
