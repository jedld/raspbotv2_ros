import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


_HELP = """
Keyboard Teleop (publishes Twist)

Controls:
  w/s : linear velocity +/−
  a/d : angular velocity +/−
    j/l : strafe left/right (linear.y +/−)
  x or space : stop
  r/f : increase/decrease linear step
    u/o : increase/decrease strafe step
  t/g : increase/decrease angular step
  q : quit

Notes:
- Run this in a real terminal (needs stdin).
- Keep this terminal focused while driving.
""".strip()


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_teleop")

        self.declare_parameter("topic", "cmd_vel")
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("deadman_timeout_sec", 0.5)

        self.declare_parameter("linear_step", 0.05)
        self.declare_parameter("lateral_step", 0.05)
        self.declare_parameter("angular_step", 0.3)
        self.declare_parameter("linear_max", 0.5)
        self.declare_parameter("lateral_max", 0.5)
        self.declare_parameter("angular_max", 2.5)

        topic = str(self.get_parameter("topic").value)
        publish_hz = float(self.get_parameter("publish_hz").value)

        self._deadman = float(self.get_parameter("deadman_timeout_sec").value)
        self._linear_step = float(self.get_parameter("linear_step").value)
        self._lateral_step = float(self.get_parameter("lateral_step").value)
        self._angular_step = float(self.get_parameter("angular_step").value)
        self._linear_max = float(self.get_parameter("linear_max").value)
        self._lateral_max = float(self.get_parameter("lateral_max").value)
        self._angular_max = float(self.get_parameter("angular_max").value)

        self._v = 0.0
        self._vy = 0.0
        self._w = 0.0
        self._last_key_time = 0.0
        self._quit = False

        self._pub = self.create_publisher(Twist, topic, 10)
        period = 1.0 / max(publish_hz, 1e-3)
        self._timer = self.create_timer(period, self._tick)

        # Print (not logger) to avoid ROS log prefixes breaking multiline formatting.
        print(_HELP, flush=True)
        print(
            f"Publishing to {topic} @ {publish_hz:.1f} Hz (deadman {self._deadman:.2f}s)",
            flush=True,
        )

    @staticmethod
    def _clamp(x: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, x))

    def _read_key(self) -> str | None:
        # Non-blocking single-char read
        r, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not r:
            return None
        ch = sys.stdin.read(1)
        return ch

    def _apply_key(self, ch: str) -> None:
        # In raw/cbreak mode Ctrl-C may appear as ETX (\x03).
        if ch in ("q", "\x03"):
            self._quit = True
            return

        if ch in ("x", " "):
            self._v = 0.0
            self._vy = 0.0
            self._w = 0.0
            return

        if ch == "w":
            self._v += self._linear_step
        elif ch == "s":
            self._v -= self._linear_step
        elif ch == "a":
            self._w += self._angular_step
        elif ch == "d":
            self._w -= self._angular_step
        elif ch == "j":
            self._vy += self._lateral_step
        elif ch == "l":
            self._vy -= self._lateral_step
        elif ch == "r":
            self._linear_step *= 1.1
        elif ch == "f":
            self._linear_step /= 1.1
        elif ch == "u":
            self._lateral_step *= 1.1
        elif ch == "o":
            self._lateral_step /= 1.1
        elif ch == "t":
            self._angular_step *= 1.1
        elif ch == "g":
            self._angular_step /= 1.1
        else:
            return

        self._v = self._clamp(self._v, -self._linear_max, self._linear_max)
        self._vy = self._clamp(self._vy, -self._lateral_max, self._lateral_max)
        self._w = self._clamp(self._w, -self._angular_max, self._angular_max)

    def _publish(self, v: float, vy: float, w: float) -> None:
        msg = Twist()
        msg.linear.x = float(v)
        msg.linear.y = float(vy)
        msg.angular.z = float(w)
        self._pub.publish(msg)

    def _tick(self) -> None:
        ch = self._read_key()
        now = time.monotonic()

        if ch is not None:
            self._apply_key(ch)
            self._last_key_time = now

        if self._quit:
            print("Quit requested; stopping.", flush=True)
            self._publish(0.0, 0.0, 0.0)
            rclpy.shutdown()
            return

        # Deadman: if no key for a while, publish stop
        if self._deadman > 0.0 and (now - self._last_key_time) > self._deadman:
            self._publish(0.0, 0.0, 0.0)
        else:
            self._publish(self._v, self._vy, self._w)


class _RawTerminal:
    def __init__(self):
        self._old = None

    def __enter__(self):
        if not sys.stdin.isatty():
            raise RuntimeError("stdin is not a TTY; run in an interactive terminal")
        self._old = termios.tcgetattr(sys.stdin)
        # cbreak keeps signals (Ctrl-C works) but still gives immediate keypresses.
        tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, exc_type, exc, tb):
        if self._old is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old)


def main():
    rclpy.init()
    try:
        with _RawTerminal():
            node = KeyboardTeleop()
            try:
                rclpy.spin(node)
            except (KeyboardInterrupt, SystemExit):
                pass
            finally:
                try:
                    node._publish(0.0, 0.0, 0.0)
                except Exception:
                    pass
                node.destroy_node()
    finally:
        rclpy.shutdown()
