import time

import rclpy
from rclpy.node import Node

from .i2c_car import I2CCar


class MotorIdTestNode(Node):
    def __init__(self):
        super().__init__("motor_id_test")

        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("i2c_addr", 0x2B)
        self.declare_parameter("i2c_protocol", "auto")
        self.declare_parameter("dry_run", False)
        self.declare_parameter("i2c_required", True)

        self.declare_parameter("pwm", 40)  # 0..100
        self.declare_parameter("motor_ids", [0, 1, 2, 3])
        self.declare_parameter("seconds_per_motor", 1.0)
        self.declare_parameter("pause_sec", 0.5)
        self.declare_parameter("repeat", True)

        i2c_bus = int(self.get_parameter("i2c_bus").value)
        i2c_addr = int(self.get_parameter("i2c_addr").value)
        i2c_protocol = str(self.get_parameter("i2c_protocol").value)
        dry_run = bool(self.get_parameter("dry_run").value)
        i2c_required = bool(self.get_parameter("i2c_required").value)

        self._pwm = int(self.get_parameter("pwm").value)
        self._motor_ids = [int(x) for x in self.get_parameter("motor_ids").value]
        self._seconds = float(self.get_parameter("seconds_per_motor").value)
        self._pause = float(self.get_parameter("pause_sec").value)
        self._repeat = bool(self.get_parameter("repeat").value)

        self._car = I2CCar(i2c_bus=i2c_bus, i2c_addr=i2c_addr, dry_run=dry_run, protocol=i2c_protocol)

        if self._car.protocol != "pi5":
            msg = f"motor_id_test requires pi5 protocol (addr 0x2B). Current protocol: {self._car.protocol}"
            if i2c_required:
                raise RuntimeError(msg)
            self.get_logger().error(msg)

        self.get_logger().info(
            f"Motor ID test starting (addr=0x{i2c_addr:02x}, pwm={self._pwm}, ids={self._motor_ids}).\n"
            "Observe which physical wheel spins for each motor_id."
        )

        self._state = "pause"
        self._index = 0
        self._state_start = time.monotonic()
        self._timer = self.create_timer(0.02, self._tick)

    def _set_only(self, motor_id: int, pwm: int) -> None:
        # Command only one motor, stop others.
        ids = [0, 1, 2, 3]
        pwms = {mid: 0.0 for mid in ids}
        pwms[int(motor_id)] = float(pwm)
        self._car.set_motor_pwms(
            pwms[0], pwms[2], pwms[1], pwms[3],  # (fl, fr, rl, rr) values don't matter; ids are fixed in I2C
            max_pwm=100,
            motor_id_fl=0,
            motor_id_fr=2,
            motor_id_rl=1,
            motor_id_rr=3,
        )

    def _stop(self) -> None:
        self._car.stop()

    def _tick(self) -> None:
        now = time.monotonic()
        dt = now - self._state_start

        if self._state == "pause":
            if dt >= self._pause:
                if self._index >= len(self._motor_ids):
                    if self._repeat:
                        self._index = 0
                    else:
                        self.get_logger().info("Done.")
                        raise SystemExit(0)

                motor_id = int(self._motor_ids[self._index])
                self.get_logger().info(f"Testing motor_id={motor_id} FORWARD")
                try:
                    self._set_only(motor_id, self._pwm)
                except Exception as e:
                    self.get_logger().error(f"I2C write failed: {e!r}")
                self._state = "run"
                self._state_start = now
            return

        if self._state == "run":
            if dt >= self._seconds:
                try:
                    self._stop()
                except Exception:
                    pass
                self._index += 1
                self._state = "pause"
                self._state_start = now

    def destroy_node(self):
        try:
            self._stop()
            self._car.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = MotorIdTestNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
