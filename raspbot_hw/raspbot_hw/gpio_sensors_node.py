import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

from .gpio_backend import GpioConfig, create_gpio


class GpioSensorsNode(Node):
    def __init__(self):
        super().__init__('gpio_sensors')

        self.declare_parameter('gpio_backend', 'auto')
        self.declare_parameter('gpio_mode', 'BOARD')
        self.declare_parameter('lgpio_chip', -1)
        self.declare_parameter('avoid_on_pin', 22)
        self.declare_parameter('avoid_left_pin', 21)
        self.declare_parameter('avoid_right_pin', 19)
        self.declare_parameter('tracking_left1_pin', 13)
        self.declare_parameter('tracking_left2_pin', 15)
        self.declare_parameter('tracking_right1_pin', 11)
        self.declare_parameter('tracking_right2_pin', 7)
        self.declare_parameter('publish_hz', 20.0)

        gpio_mode = str(self.get_parameter('gpio_mode').value).upper()
        gpio_backend = str(self.get_parameter('gpio_backend').value)
        lgpio_chip = int(self.get_parameter('lgpio_chip').value)

        self._avoid_on = int(self.get_parameter('avoid_on_pin').value)
        self._avoid_l = int(self.get_parameter('avoid_left_pin').value)
        self._avoid_r = int(self.get_parameter('avoid_right_pin').value)

        self._trk_l1 = int(self.get_parameter('tracking_left1_pin').value)
        self._trk_l2 = int(self.get_parameter('tracking_left2_pin').value)
        self._trk_r1 = int(self.get_parameter('tracking_right1_pin').value)
        self._trk_r2 = int(self.get_parameter('tracking_right2_pin').value)

        publish_hz = float(self.get_parameter('publish_hz').value)

        cfg = GpioConfig(backend=gpio_backend, mode=gpio_mode, lgpio_chip=lgpio_chip)

        self._gpio = create_gpio(cfg, logger=self.get_logger())
        try:
            self._gpio.setup_in(self._avoid_l)
            self._gpio.setup_in(self._avoid_r)
            self._gpio.setup_out(self._avoid_on, initial=True)

            self._gpio.setup_in(self._trk_l1)
            self._gpio.setup_in(self._trk_l2)
            self._gpio.setup_in(self._trk_r1)
            self._gpio.setup_in(self._trk_r2)
        except Exception as e:
            if cfg.backend.lower().strip() == 'auto':
                self.get_logger().warn(f'GPIO init failed ({e!r}); retrying with lgpio backend')
                try:
                    self._gpio.cleanup()
                except Exception:
                    pass
                self._gpio = create_gpio(GpioConfig(backend='lgpio', mode=gpio_mode, lgpio_chip=lgpio_chip))
                self._gpio.setup_in(self._avoid_l)
                self._gpio.setup_in(self._avoid_r)
                self._gpio.setup_out(self._avoid_on, initial=True)

                self._gpio.setup_in(self._trk_l1)
                self._gpio.setup_in(self._trk_l2)
                self._gpio.setup_in(self._trk_r1)
                self._gpio.setup_in(self._trk_r2)
            else:
                raise

        self._avoid_pub = self.create_publisher(Int32, 'ir_avoid/state', 10)
        self._track_pub = self.create_publisher(Int32, 'tracking/state', 10)

        period = 1.0 / max(publish_hz, 1e-3)
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            'GPIO sensors ready. Topics: /ir_avoid/state (L bit0, R bit1), /tracking/state (L1 bit0, L2 bit1, R1 bit2, R2 bit3)'
        )

    def _tick(self) -> None:
        gpio = self._gpio

        avoid_l = 1 if gpio.read(self._avoid_l) else 0
        avoid_r = 1 if gpio.read(self._avoid_r) else 0
        avoid_bits = (avoid_l << 0) | (avoid_r << 1)

        trk_l1 = 1 if gpio.read(self._trk_l1) else 0
        trk_l2 = 1 if gpio.read(self._trk_l2) else 0
        trk_r1 = 1 if gpio.read(self._trk_r1) else 0
        trk_r2 = 1 if gpio.read(self._trk_r2) else 0
        track_bits = (trk_l1 << 0) | (trk_l2 << 1) | (trk_r1 << 2) | (trk_r2 << 3)

        a = Int32()
        a.data = int(avoid_bits)
        self._avoid_pub.publish(a)

        t = Int32()
        t.data = int(track_bits)
        self._track_pub.publish(t)

    def destroy_node(self):
        try:
            self._gpio.cleanup()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = GpioSensorsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
