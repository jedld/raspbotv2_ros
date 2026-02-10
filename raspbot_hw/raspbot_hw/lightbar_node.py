"""ROS 2 node for Yahboom Raspbot V2 WS2812 light-bar (14 LEDs).

Subscribes to:
  lightbar/command  (std_msgs/String)  – JSON command  (see below)

JSON command formats:

  {"mode": "off"}
      Turn all LEDs off.

  {"mode": "solid", "r": 255, "g": 0, "b": 0}
      Set every LED to the given RGB colour.

  {"mode": "solid", "r": 0, "g": 255, "b": 128, "index": 3}
      Set a single LED (0-based index) to an RGB colour.

  {"mode": "preset", "color": 0}
      All LEDs on with a preset colour id (0=red … 6=white).

  {"mode": "preset", "color": 2, "index": 5}
      Single LED on with a preset colour.

  {"mode": "breathing", "r": 0, "g": 0, "b": 255, "period": 2.0}
      Breathing (pulse) effect for the given colour.

  {"mode": "rainbow", "speed": 1.0}
      Rotating rainbow across all 14 LEDs.

  {"mode": "chase", "r": 255, "g": 100, "b": 0, "speed": 1.0}
      Single-LED chase / running-light effect.
"""
import json
import math
import time
import colorsys
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .i2c_car import I2CCar


class LightbarNode(Node):
    NUM_LEDS = I2CCar.NUM_LEDS

    def __init__(self) -> None:
        super().__init__('lightbar')

        # ---- parameters ----
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_addr', 0x2B)
        self.declare_parameter('i2c_protocol', 'auto')
        self.declare_parameter('i2c_required', False)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('command_topic', 'lightbar/command')
        self.declare_parameter('startup_effect', 'rainbow')
        self.declare_parameter('startup_duration_sec', 3.0)
        self.declare_parameter('effect_fps', 30.0)

        bus = int(self.get_parameter('i2c_bus').value)
        addr = int(self.get_parameter('i2c_addr').value)
        proto = str(self.get_parameter('i2c_protocol').value)
        required = bool(self.get_parameter('i2c_required').value)
        dry_run = bool(self.get_parameter('dry_run').value)
        self._effect_fps = max(1.0, float(self.get_parameter('effect_fps').value))

        try:
            self._car = I2CCar(i2c_bus=bus, i2c_addr=addr, dry_run=dry_run, protocol=proto)
            self.get_logger().info(
                f'Lightbar ready on I2C bus={bus} addr=0x{addr:02X} proto={self._car.protocol}'
            )
        except Exception as exc:
            if required:
                self.get_logger().fatal(f'Cannot open I2C for lightbar: {exc}')
                raise
            self.get_logger().warn(f'Cannot open I2C for lightbar: {exc} – running in dry-run mode')
            self._car = I2CCar(i2c_bus=bus, i2c_addr=addr, dry_run=True, protocol=proto)

        # ---- effect state ----
        self._effect_lock = threading.Lock()
        self._current_mode = 'off'
        self._effect_params: dict = {}
        self._effect_running = False
        self._effect_thread: Optional[threading.Thread] = None

        # ---- subscriber ----
        topic = str(self.get_parameter('command_topic').value)
        self._sub = self.create_subscription(String, topic, self._on_command, 10)
        self.get_logger().info(f'Listening on {topic} (std_msgs/String JSON)')

        # ---- optional startup effect ----
        startup = str(self.get_parameter('startup_effect').value).strip().lower()
        duration = float(self.get_parameter('startup_duration_sec').value)
        if startup and startup != 'none' and duration > 0.0:
            self.get_logger().info(f'Playing startup effect: {startup} for {duration:.1f}s')
            self._apply_command({'mode': startup})
            self._startup_timer = self.create_timer(duration, self._end_startup)
        else:
            self._startup_timer = None
            self._car.lightbar_off()

    # ---- helpers ----

    def _end_startup(self) -> None:
        if self._startup_timer is not None:
            self._startup_timer.cancel()
            self._startup_timer = None
        self._stop_effect()
        self._car.lightbar_off()
        self._current_mode = 'off'
        self.get_logger().info('Startup effect finished – lightbar off')

    def _on_command(self, msg: String) -> None:
        if not msg.data:
            return
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Invalid JSON on lightbar/command: {exc}')
            return
        self._apply_command(cmd)

    def _apply_command(self, cmd: dict) -> None:
        mode = str(cmd.get('mode', 'off')).strip().lower()

        # Cancel any running animation first.
        self._stop_effect()

        if mode == 'off':
            self._car.lightbar_off()
            self._current_mode = 'off'
            self._effect_params = {}
            return

        if mode == 'solid':
            r = int(cmd.get('r', 0)) & 0xFF
            g = int(cmd.get('g', 0)) & 0xFF
            b = int(cmd.get('b', 0)) & 0xFF
            idx = cmd.get('index')
            if idx is not None:
                self._car.lightbar_set_one_rgb(int(idx), r, g, b)
            else:
                self._car.lightbar_set_all_rgb(r, g, b)
            self._current_mode = 'solid'
            self._effect_params = {'r': r, 'g': g, 'b': b}
            if idx is not None:
                self._effect_params['index'] = int(idx)
            return

        if mode == 'preset':
            color_id = int(cmd.get('color', 0))
            idx = cmd.get('index')
            if idx is not None:
                self._car.lightbar_set_one_preset(int(idx), True, color_id)
            else:
                self._car.lightbar_set_all_preset(True, color_id)
            self._current_mode = 'preset'
            self._effect_params = {'color': color_id}
            if idx is not None:
                self._effect_params['index'] = int(idx)
            return

        # Animated effects run on a background thread.
        if mode == 'breathing':
            r = int(cmd.get('r', 0)) & 0xFF
            g = int(cmd.get('g', 0)) & 0xFF
            b = int(cmd.get('b', 0)) & 0xFF
            period = max(0.5, float(cmd.get('period', 2.0)))
            self._current_mode = 'breathing'
            self._effect_params = {'r': r, 'g': g, 'b': b, 'period': period}
            self._start_effect(self._effect_breathing, r, g, b, period)
            return

        if mode == 'rainbow':
            speed = max(0.1, float(cmd.get('speed', 1.0)))
            self._current_mode = 'rainbow'
            self._effect_params = {'speed': speed}
            self._start_effect(self._effect_rainbow, speed)
            return

        if mode == 'chase':
            r = int(cmd.get('r', 255)) & 0xFF
            g = int(cmd.get('g', 255)) & 0xFF
            b = int(cmd.get('b', 255)) & 0xFF
            speed = max(0.1, float(cmd.get('speed', 1.0)))
            self._current_mode = 'chase'
            self._effect_params = {'r': r, 'g': g, 'b': b, 'speed': speed}
            self._start_effect(self._effect_chase, r, g, b, speed)
            return

        self.get_logger().warn(f'Unknown lightbar mode: {mode}')

    # ---- background effect helpers ----

    def _start_effect(self, target, *args) -> None:
        self._effect_running = True
        self._effect_thread = threading.Thread(target=target, args=args, daemon=True)
        self._effect_thread.start()

    def _stop_effect(self) -> None:
        self._effect_running = False
        t = self._effect_thread
        if t is not None and t.is_alive():
            t.join(timeout=1.0)
        self._effect_thread = None

    def _sleep_or_stop(self, dt: float) -> bool:
        """Sleep for *dt* seconds; return True if the effect should keep running."""
        end = time.monotonic() + dt
        while time.monotonic() < end:
            if not self._effect_running:
                return False
            time.sleep(min(0.02, end - time.monotonic()))
        return self._effect_running

    def _effect_breathing(self, r: int, g: int, b: int, period: float) -> None:
        dt = 1.0 / self._effect_fps
        t0 = time.monotonic()
        while self._effect_running:
            elapsed = time.monotonic() - t0
            # sine-based brightness (0..1)
            brightness = 0.5 * (1.0 + math.sin(2.0 * math.pi * elapsed / period - math.pi / 2.0))
            br = int(r * brightness)
            bg = int(g * brightness)
            bb = int(b * brightness)
            try:
                self._car.lightbar_set_all_rgb(br, bg, bb)
            except Exception:
                pass
            if not self._sleep_or_stop(dt):
                return

    def _effect_rainbow(self, speed: float) -> None:
        dt = 1.0 / self._effect_fps
        t0 = time.monotonic()
        while self._effect_running:
            elapsed = time.monotonic() - t0
            for i in range(self.NUM_LEDS):
                hue = (float(i) / self.NUM_LEDS + elapsed * speed * 0.2) % 1.0
                r, g, b = colorsys.hsv_to_rgb(hue, 1.0, 1.0)
                try:
                    self._car.lightbar_set_one_rgb(i, int(r * 255), int(g * 255), int(b * 255))
                except Exception:
                    pass
            if not self._sleep_or_stop(dt):
                return

    def _effect_chase(self, r: int, g: int, b: int, speed: float) -> None:
        interval = max(0.05, 1.0 / (speed * 10.0))
        pos = 0
        while self._effect_running:
            for i in range(self.NUM_LEDS):
                try:
                    if i == pos:
                        self._car.lightbar_set_one_rgb(i, r, g, b)
                    else:
                        self._car.lightbar_set_one_rgb(i, 0, 0, 0)
                except Exception:
                    pass
            pos = (pos + 1) % self.NUM_LEDS
            if not self._sleep_or_stop(interval):
                return

    # ---- public status (for web UI integration) ----

    def get_state(self) -> dict:
        return {
            'mode': str(self._current_mode),
            'params': dict(self._effect_params),
            'num_leds': self.NUM_LEDS,
        }

    def destroy_node(self) -> None:
        self._stop_effect()
        try:
            self._car.lightbar_off()
        except Exception:
            pass
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = LightbarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
