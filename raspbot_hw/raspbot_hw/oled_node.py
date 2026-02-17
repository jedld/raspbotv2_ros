import socket
import subprocess
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String


class SSD1306:
    def __init__(self, *, i2c_bus: int, i2c_addr: int, width: int, height: int, dry_run: bool = False):
        self._addr = int(i2c_addr)
        self._width = int(width)
        self._height = int(height)
        self._dry_run = bool(dry_run)

        if self._height not in (32, 64):
            raise ValueError("height must be 32 or 64 for this driver")
        if self._width != 128:
            raise ValueError("width must be 128 for this driver")

        self._bus = None
        if not self._dry_run:
            try:
                import smbus  # type: ignore

                self._bus = smbus.SMBus(int(i2c_bus))
            except Exception:
                import smbus2  # type: ignore

                self._bus = smbus2.SMBus(int(i2c_bus))

    def close(self) -> None:
        if self._dry_run or self._bus is None:
            return
        close_method = getattr(self._bus, "close", None)
        if callable(close_method):
            close_method()

    def _write_cmds(self, cmds: list[int]) -> None:
        if self._dry_run:
            return
        # Control byte 0x00 indicates command stream.
        # We chunk to avoid SMBus block size limitations.
        chunk = 16
        for i in range(0, len(cmds), chunk):
            part = cmds[i : i + chunk]
            self._bus.write_i2c_block_data(self._addr, 0x00, [int(x) & 0xFF for x in part])

    def _write_data(self, data: bytes) -> None:
        if self._dry_run:
            return
        chunk = 16
        for i in range(0, len(data), chunk):
            part = data[i : i + chunk]
            # Control byte 0x40 indicates data stream.
            self._bus.write_i2c_block_data(self._addr, 0x40, list(part))

    def init(self, *, rotate_180: bool = False, contrast: int = 0x7F) -> None:
        mux = 0x3F if self._height == 64 else 0x1F
        compins = 0x12 if self._height == 64 else 0x02

        seg_remap = 0xA0 if rotate_180 else 0xA1
        com_scan = 0xC0 if rotate_180 else 0xC8

        self._write_cmds(
            [
                0xAE,  # display off
                0xD5,
                0x80,  # clock
                0xA8,
                mux,  # multiplex
                0xD3,
                0x00,  # display offset
                0x40,  # start line
                seg_remap,
                com_scan,
                0xDA,
                compins,
                0x81,
                int(contrast) & 0xFF,
                0xD9,
                0xF1,  # precharge
                0xDB,
                0x40,  # vcomh
                0xA4,  # resume RAM
                0xA6,  # normal display
                0x20,
                0x00,  # horizontal addressing mode
                0x8D,
                0x14,  # charge pump on
                0xAF,  # display on
            ]
        )

    def clear(self) -> None:
        self.display_buffer(bytes([0x00]) * (self._width * (self._height // 8)))

    def display_buffer(self, buf: bytes) -> None:
        pages = self._height // 8
        expected = self._width * pages
        if len(buf) != expected:
            raise ValueError(f"buffer size must be {expected} bytes")

        for page in range(pages):
            self._write_cmds(
                [
                    0xB0 + page,  # page start
                    0x00,  # low column
                    0x10,  # high column
                ]
            )
            start = page * self._width
            self._write_data(buf[start : start + self._width])


def _first_non_local_ip() -> str:
    try:
        out = subprocess.check_output(["hostname", "-I"], text=True).strip()
        parts = [p.strip() for p in out.split() if p.strip()]
        for p in parts:
            if not p.startswith("127."):
                return p
        if parts:
            return parts[0]
    except Exception:
        pass
    try:
        return socket.gethostbyname(socket.gethostname())
    except Exception:
        return "?"


class OledNode(Node):
    def __init__(self) -> None:
        super().__init__("oled")

        self.declare_parameter("enabled", True)
        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("i2c_addr", 60)  # 0x3C
        self.declare_parameter("i2c_required", False)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("width", 128)
        self.declare_parameter("height", 64)
        self.declare_parameter("rotate_180", False)
        self.declare_parameter("contrast", 0x7F)
        self.declare_parameter("update_hz", 2.0)
        # Rendering
        self.declare_parameter("font_size", 8)
        self.declare_parameter("font_path", "")
        # If true, cycle through common OLED configs on startup so you can visually
        # identify the right height/rotation for your panel.
        self.declare_parameter("startup_probe", False)
        self.declare_parameter("startup_probe_hold_sec", 2.0)

        self.declare_parameter("text_topic", "oled/text")
        self.declare_parameter("ultrasonic_topic", "ultrasonic/range")
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("show_ip", True)

        self._custom_text: str = ""
        self._last_range_m: Optional[float] = None
        self._last_cmd: Optional[tuple[float, float]] = None
        self._last_cmd_time: float = 0.0

        enabled = bool(self.get_parameter("enabled").value)
        if not enabled:
            self.get_logger().info("OLED disabled by parameter")
            self._oled = None
            return

        i2c_bus = int(self.get_parameter("i2c_bus").value)
        i2c_addr = int(self.get_parameter("i2c_addr").value)
        i2c_required = bool(self.get_parameter("i2c_required").value)
        dry_run = bool(self.get_parameter("dry_run").value)

        width = int(self.get_parameter("width").value)
        height = int(self.get_parameter("height").value)
        rotate_180 = bool(self.get_parameter("rotate_180").value)
        contrast = int(self.get_parameter("contrast").value)

        self._i2c_bus = i2c_bus
        self._i2c_addr = i2c_addr
        self._dry_run = dry_run
        self._contrast = contrast

        self._oled = None
        try:
            self._init_oled(width=width, height=height, rotate_180=rotate_180)
        except Exception as e:
            msg = f"OLED not reachable (bus={i2c_bus}, addr=0x{i2c_addr:02x}): {e!r}"
            if i2c_required:
                raise RuntimeError(msg) from e
            self.get_logger().warn(msg)
            self._oled = None

        if self._oled is not None and bool(self.get_parameter("startup_probe").value):
            hold = float(self.get_parameter("startup_probe_hold_sec").value)
            self._run_startup_probe(width=width, hold_sec=hold)
            # Restore configured settings after probing.
            try:
                self._init_oled(width=width, height=height, rotate_180=rotate_180)
            except Exception:
                pass

        text_topic = str(self.get_parameter("text_topic").value)
        ultrasonic_topic = str(self.get_parameter("ultrasonic_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)

        self.create_subscription(String, text_topic, self._on_text, 10)
        self.create_subscription(Range, ultrasonic_topic, self._on_range, 10)
        self.create_subscription(Twist, cmd_vel_topic, self._on_cmd, 10)

        hz = float(self.get_parameter("update_hz").value)
        period = 1.0 / max(hz, 0.2)
        self._timer = self.create_timer(period, self._tick)

        self._show_ip = bool(self.get_parameter("show_ip").value)
        self._ip = _first_non_local_ip() if self._show_ip else ""
        # Retry interval for IP discovery when network isn't ready at boot.
        self._ip_retry_interval = 3.0  # seconds between retries
        self._ip_last_check = time.monotonic()
        self._ip_resolved = (self._ip not in ("", "?", "127.0.0.1"))
        self.get_logger().info(
            f"OLED node ready (i2c bus={i2c_bus}, addr=0x{i2c_addr:02x}, {width}x{height}). Text topic: {text_topic}"
        )
        if self._show_ip and not self._ip_resolved:
            self.get_logger().info("OLED: IP not yet available, will retry periodically")

    def _init_oled(self, *, width: int, height: int, rotate_180: bool) -> None:
        prev = getattr(self, "_oled", None)
        if prev is not None:
            try:
                prev.close()
            except Exception:
                pass

        oled = SSD1306(
            i2c_bus=self._i2c_bus,
            i2c_addr=self._i2c_addr,
            width=int(width),
            height=int(height),
            dry_run=self._dry_run,
        )
        oled.init(rotate_180=bool(rotate_180), contrast=int(self._contrast))
        oled.clear()
        self._oled = oled

    @staticmethod
    def _render_text_screen(*, width: int, height: int, lines: list[str]) -> bytes:
        from PIL import Image, ImageDraw, ImageFont

        img = Image.new("1", (int(width), int(height)), 0)
        draw = ImageDraw.Draw(img)
        font = ImageFont.load_default()

        y = 0
        line_h = 10
        for line in lines:
            if y >= height:
                break
            draw.text((0, y), str(line)[:32], fill=1, font=font)
            y += line_h

        pages = int(height) // 8
        pix = img.load()
        out = bytearray(int(width) * pages)
        for page in range(pages):
            for x in range(int(width)):
                b = 0
                for bit in range(8):
                    yy = page * 8 + bit
                    if yy >= int(height):
                        continue
                    if pix[x, yy]:
                        b |= (1 << bit)
                out[page * int(width) + x] = b
        return bytes(out)

    def _run_startup_probe(self, *, width: int, hold_sec: float) -> None:
        hold = max(float(hold_sec), 0.5)
        modes = [
            (64, False),
            (64, True),
            (32, False),
            (32, True),
        ]

        self.get_logger().info(
            f"OLED startup_probe enabled: cycling {len(modes)} modes (hold {hold:.1f}s each)"
        )

        for idx, (height, rot) in enumerate(modes, start=1):
            if not rclpy.ok():
                return

            try:
                self._init_oled(width=int(width), height=int(height), rotate_180=bool(rot))
            except Exception as e:
                self.get_logger().warn(f"OLED probe mode {idx}/{len(modes)} failed to init: {e!r}")
                continue

            label = f"{int(width)}x{int(height)} rot={'180' if rot else '0'}"
            lines = [
                "OLED PROBE",
                f"{idx}/{len(modes)} {label}",
                "If readable: use",
                f"height={height}",
                f"rotate_180={'true' if rot else 'false'}",
            ]
            try:
                buf = self._render_text_screen(width=int(width), height=int(height), lines=lines)
                self._oled.display_buffer(buf)
            except Exception as e:
                self.get_logger().warn(f"OLED probe mode {idx}/{len(modes)} render failed: {e!r}")

            end = time.monotonic() + hold
            while rclpy.ok() and time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.1)

    def _on_text(self, msg: String) -> None:
        self._custom_text = str(msg.data)

    def _on_range(self, msg: Range) -> None:
        try:
            self._last_range_m = float(msg.range)
        except Exception:
            self._last_range_m = None

    def _on_cmd(self, msg: Twist) -> None:
        self._last_cmd = (float(msg.linear.x), float(msg.angular.z))
        self._last_cmd_time = time.time()

    def _render(self) -> bytes:
        # Lazy-init cached rendering objects (font, image, draw)
        if not hasattr(self, '_cached_font'):
            from PIL import Image, ImageDraw, ImageFont
            self._PIL_Image = Image
            self._PIL_ImageDraw = ImageDraw
            self._cached_width = int(self.get_parameter("width").value)
            self._cached_height = int(self.get_parameter("height").value)
            self._cached_font = self._load_font(ImageFont)
            try:
                bbox = self._cached_font.getbbox("Ag")
                self._cached_line_h = max(7, int(bbox[3] - bbox[1]) + 1)
            except Exception:
                self._cached_line_h = 9
            self._cached_img = Image.new("1", (self._cached_width, self._cached_height), 0)
            self._cached_draw = ImageDraw.Draw(self._cached_img)
            try:
                import numpy as _np
                self._np = _np
            except ImportError:
                self._np = None

        width = self._cached_width
        height = self._cached_height
        font = self._cached_font
        line_h = self._cached_line_h
        img = self._cached_img
        draw = self._cached_draw

        # Clear image for reuse
        draw.rectangle((0, 0, width - 1, height - 1), fill=0)

        max_lines = max(1, int(height // max(line_h, 1)))
        lines: list[str] = []

        # Compact, line-based layout (works well for 128x32).
        lines.append("Raspbot")
        if self._ip:
            lines.append(f"IP {self._ip}")

        if self._last_range_m is None or self._last_range_m != self._last_range_m:
            lines.append("US ?")
        else:
            us_cm = self._last_range_m * 100.0
            if us_cm == float("inf"):
                lines.append("US inf")
            else:
                lines.append(f"US {us_cm:0.0f}cm")

        if self._last_cmd is None:
            lines.append("cmd ?")
        else:
            vx, wz = self._last_cmd
            lines.append(f"v{vx:+.2f} w{wz:+.2f}")

        # If the user provided text, show it after the core status.
        if self._custom_text:
            for part in self._custom_text.splitlines():
                if part.strip():
                    lines.append(part.strip())

        # Only render what fits.
        y = 0
        for line in lines[:max_lines]:
            draw.text((0, y), str(line)[:32], fill=1, font=font)
            y += line_h

        # Convert image to SSD1306 page buffer.
        pages = height // 8
        if self._np is not None:
            # Vectorised conversion: ~50× faster than pure-Python loops
            np = self._np
            arr = np.array(img, dtype=np.uint8)  # shape (height, width), values 0/1
            # Reshape into (pages, 8, width) and pack bits
            arr = arr.reshape(pages, 8, width)
            # SSD1306 page byte: bit 0 = top row of page, bit 7 = bottom
            packed = np.zeros((pages, width), dtype=np.uint8)
            for bit in range(8):
                packed |= arr[:, bit, :].astype(np.uint8) << bit
            out = bytes(packed.tobytes())
        else:
            pix = img.load()
            out = bytearray(width * pages)
            for page in range(pages):
                for x in range(width):
                    b = 0
                    for bit in range(8):
                        y = page * 8 + bit
                        if y >= height:
                            continue
                        if pix[x, y]:
                            b |= (1 << bit)
                    out[page * width + x] = b
            out = bytes(out)
        return out

    def _load_font(self, ImageFont):
        """Load a small monospace-ish font if available; fall back to default."""
        size = int(self.get_parameter("font_size").value)
        if size <= 0:
            size = 8

        path = str(self.get_parameter("font_path").value).strip()
        candidates = []
        if path:
            candidates.append(path)
        candidates.extend(
            [
                "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
                "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
                "/usr/share/fonts/truetype/liberation/LiberationMono-Regular.ttf",
            ]
        )

        for p in candidates:
            try:
                return ImageFont.truetype(p, size=size)
            except Exception:
                continue

        return ImageFont.load_default()

    def _tick(self) -> None:
        if getattr(self, "_oled", None) is None:
            return

        # Retry IP discovery until the network is up.
        if self._show_ip and not self._ip_resolved:
            now = time.monotonic()
            if now - self._ip_last_check >= self._ip_retry_interval:
                self._ip_last_check = now
                new_ip = _first_non_local_ip()
                if new_ip not in ("", "?", "127.0.0.1"):
                    self._ip = new_ip
                    self._ip_resolved = True
                    self.get_logger().info(f"OLED: IP resolved to {new_ip}")
                else:
                    self._ip = new_ip  # show "?" until resolved

        try:
            buf = self._render()
            self._oled.display_buffer(buf)
        except Exception as e:
            self.get_logger().warn(f"OLED update failed: {e!r}")

    def destroy_node(self):
        oled = getattr(self, "_oled", None)
        if oled is not None:
            try:
                oled.clear()
            except Exception:
                pass
            try:
                oled.close()
            except Exception:
                pass
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = OledNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
