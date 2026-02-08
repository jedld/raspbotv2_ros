import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node


_HELP = """
Gimbal Teleop (publishes Vector3 degrees)

Publishes to a topic (default: /camera_gimbal/command_deg):
  msg.x = pan_deg
  msg.y = tilt_deg

Controls:
  Arrow Left/Right : pan -/+ (degrees)
  Arrow Up/Down    : tilt +/− (degrees)
  h/l              : pan -/+ (vim-style)
  k/j              : tilt +/− (vim-style)
  0                : reset to neutral
  [ / ]            : decrease/increase step
  space or x       : reset to neutral
  q                : quit

Notes:
- Run this in a real terminal (needs stdin).
- Keep this terminal focused while controlling the gimbal.
""".strip()


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class GimbalTeleop(Node):
    def __init__(self):
        super().__init__("gimbal_teleop")

        self.declare_parameter("topic", "camera_gimbal/command_deg")
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("step_deg", 5.0)

        self.declare_parameter("pan_neutral_deg", 90.0)
        self.declare_parameter("tilt_neutral_deg", 90.0)
        self.declare_parameter("pan_min_deg", 0.0)
        self.declare_parameter("pan_max_deg", 180.0)
        self.declare_parameter("tilt_min_deg", 0.0)
        self.declare_parameter("tilt_max_deg", 110.0)

        topic = str(self.get_parameter("topic").value)
        publish_hz = float(self.get_parameter("publish_hz").value)

        self._step = float(self.get_parameter("step_deg").value)

        self._pan_neutral = float(self.get_parameter("pan_neutral_deg").value)
        self._tilt_neutral = float(self.get_parameter("tilt_neutral_deg").value)

        self._pan_min = float(self.get_parameter("pan_min_deg").value)
        self._pan_max = float(self.get_parameter("pan_max_deg").value)
        self._tilt_min = float(self.get_parameter("tilt_min_deg").value)
        self._tilt_max = float(self.get_parameter("tilt_max_deg").value)

        self._pan = _clamp(self._pan_neutral, self._pan_min, self._pan_max)
        self._tilt = _clamp(self._tilt_neutral, self._tilt_min, self._tilt_max)

        self._last_key_time = 0.0
        self._quit = False

        self._pub = self.create_publisher(Vector3, topic, 10)

        period = 1.0 / max(publish_hz, 1e-3)
        self._timer = self.create_timer(period, self._tick)

        # Print (not logger) to avoid ROS log prefixes breaking multiline formatting.
        print(_HELP, flush=True)
        print(
            f"Publishing to {topic} @ {publish_hz:.1f} Hz (step {self._step:.1f} deg)",
            flush=True,
        )

    def _publish(self) -> None:
        msg = Vector3()
        msg.x = float(self._pan)
        msg.y = float(self._tilt)
        msg.z = 0.0
        self._pub.publish(msg)

    def _reset_neutral(self) -> None:
        self._pan = _clamp(self._pan_neutral, self._pan_min, self._pan_max)
        self._tilt = _clamp(self._tilt_neutral, self._tilt_min, self._tilt_max)

    def _read_key(self) -> str | None:
        r, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not r:
            return None
        return sys.stdin.read(1)

    def _read_escape_sequence(self) -> str | None:
        # Reads the rest of an escape sequence if available.
        # Arrow keys come in as: ESC [ A/B/C/D
        r, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not r:
            return None
        return sys.stdin.read(1)

    def _apply_key(self, ch: str) -> None:
        if ch in ("q", "\x03"):
            self._quit = True
            return

        if ch in ("x", " ", "0"):
            self._reset_neutral()
            return

        if ch == "[":
            self._step = max(1.0, self._step - 1.0)
            return
        if ch == "]":
            self._step = min(45.0, self._step + 1.0)
            return

        # Vim-style fallback
        if ch == "h":
            self._pan -= self._step
        elif ch == "l":
            self._pan += self._step
        elif ch == "k":
            self._tilt += self._step
        elif ch == "j":
            self._tilt -= self._step
        elif ch == "\x1b":
            # Escape sequence (arrow keys)
            nxt = self._read_escape_sequence()
            if nxt != "[":
                return
            code = self._read_escape_sequence()
            if code == "D":  # left
                self._pan -= self._step
            elif code == "C":  # right
                self._pan += self._step
            elif code == "A":  # up
                self._tilt += self._step
            elif code == "B":  # down
                self._tilt -= self._step
            else:
                return
        else:
            return

        self._pan = _clamp(self._pan, self._pan_min, self._pan_max)
        self._tilt = _clamp(self._tilt, self._tilt_min, self._tilt_max)

    def _tick(self) -> None:
        ch = self._read_key()
        now = time.monotonic()

        if ch is not None:
            self._apply_key(ch)
            self._last_key_time = now

        if self._quit:
            print("Quit requested.", flush=True)
            rclpy.shutdown()
            return

        # Always publish the latest command so the gimbal node can act on it.
        self._publish()


class _RawTerminal:
    def __init__(self):
        self._old = None

    def __enter__(self):
        if not sys.stdin.isatty():
            raise RuntimeError("stdin is not a TTY; run in an interactive terminal")
        self._old = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, exc_type, exc, tb):
        if self._old is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old)


def main():
    rclpy.init()
    try:
        with _RawTerminal():
            node = GimbalTeleop()
            try:
                rclpy.spin(node)
            except (KeyboardInterrupt, SystemExit):
                pass
            finally:
                node.destroy_node()
    finally:
        rclpy.shutdown()
