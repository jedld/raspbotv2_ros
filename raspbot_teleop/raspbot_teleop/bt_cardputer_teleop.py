import asyncio
import io
import math
import subprocess
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Imu
from std_msgs.msg import Bool, Float64

try:
    from bleak import BleakClient, BleakScanner
except Exception:  # pragma: no cover
    BleakClient = None
    BleakScanner = None

try:
    from PIL import Image as PILImage
except Exception:  # pragma: no cover
    PILImage = None


SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
TX_NOTIFY_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  # Cardputer -> Pi
RX_WRITE_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"   # Pi -> Cardputer

# BLE MTU payload limit (conservative — negotiated MTU minus ATT header)
BLE_CHUNK_SIZE = 180


@dataclass
class MotionState:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


@dataclass
class OrientationState:
    roll: float = 0.0
    pitch: float = 0.0


class BtCardputerTeleop(Node):
    def __init__(self) -> None:
        super().__init__("bt_cardputer_teleop")

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter("device_name", "RaspbotCardputer")
        self.declare_parameter("device_address", "")
        self.declare_parameter("scan_timeout_sec", 8.0)
        self.declare_parameter("reconnect_delay_sec", 2.0)
        self.declare_parameter("deadman_timeout_sec", 0.5)
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("telemetry_hz", 5.0)
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("gimbal_topic", "camera_gimbal/command_deg")
        self.declare_parameter("follow_enable_topic", "follow/enable")
        self.declare_parameter("imu_topic", "imu/data")
        self.declare_parameter("imu_yaw_topic", "imu/yaw_deg")
        self.declare_parameter("imu_mag_topic", "imu/mag")
        self.declare_parameter("camera_topic", "image_raw/compressed")
        self.declare_parameter("camera_front_topic", "front_camera/compressed")
        self.declare_parameter("photo_width", 240)
        self.declare_parameter("photo_height", 135)
        self.declare_parameter("photo_jpeg_quality", 40)
        self.declare_parameter("auto_power_on_bluetooth", True)
        self.declare_parameter("auto_connect_paired_device", False)

        self._device_name = str(self.get_parameter("device_name").value)
        self._device_address = str(self.get_parameter("device_address").value).strip().upper()
        self._scan_timeout = float(self.get_parameter("scan_timeout_sec").value)
        self._reconnect_delay = float(self.get_parameter("reconnect_delay_sec").value)
        self._deadman = float(self.get_parameter("deadman_timeout_sec").value)
        self._publish_hz = float(self.get_parameter("publish_hz").value)
        self._telemetry_hz = float(self.get_parameter("telemetry_hz").value)
        self._cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._gimbal_topic = str(self.get_parameter("gimbal_topic").value)
        self._follow_enable_topic = str(self.get_parameter("follow_enable_topic").value)
        self._imu_topic = str(self.get_parameter("imu_topic").value)
        self._imu_yaw_topic = str(self.get_parameter("imu_yaw_topic").value)
        self._imu_mag_topic = str(self.get_parameter("imu_mag_topic").value)
        self._camera_topic = str(self.get_parameter("camera_topic").value)
        self._camera_front_topic = str(self.get_parameter("camera_front_topic").value)
        self._photo_w = int(self.get_parameter("photo_width").value)
        self._photo_h = int(self.get_parameter("photo_height").value)
        self._photo_quality = int(self.get_parameter("photo_jpeg_quality").value)
        self._auto_power_on_bt = bool(self.get_parameter("auto_power_on_bluetooth").value)
        self._auto_connect_paired = bool(self.get_parameter("auto_connect_paired_device").value)

        # ── Publishers ──────────────────────────────────────────────
        self._cmd_vel_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._gimbal_pub = self.create_publisher(Vector3, self._gimbal_topic, 10)
        self._follow_pub = self.create_publisher(Bool, self._follow_enable_topic, 10)

        # ── Subscribers ─────────────────────────────────────────────
        self.create_subscription(Imu, self._imu_topic, self._on_imu, 10)
        self.create_subscription(Float64, self._imu_yaw_topic, self._on_yaw, 10)
        self.create_subscription(Vector3, self._imu_mag_topic, self._on_mag, 10)
        self.create_subscription(
            CompressedImage, self._camera_topic, self._on_camera_image, 1
        )
        self.create_subscription(
            CompressedImage, self._camera_front_topic, self._on_front_image, 1
        )

        # ── Drive state ─────────────────────────────────────────────
        self._motion = MotionState()
        self._last_cmd_time = 0.0
        self._bt_active = False         # True while BT has active control
        self._sent_stop = True          # True once we've sent a single zero-stop
        self._follow_enabled = False
        self._gimbal_control_enabled = False
        self._gimbal_pan = 90.0
        self._gimbal_tilt = 45.0

        # ── Sensor state ────────────────────────────────────────────
        self._gyro_z = 0.0
        self._yaw_heading = 0.0
        self._mag_x = 0.0
        self._mag_y = 0.0
        self._orient = OrientationState()

        # ── Camera / photo transfer ─────────────────────────────────
        self._last_camera_jpeg: Optional[bytes] = None
        self._last_front_jpeg: Optional[bytes] = None
        self._camera_lock = threading.Lock()
        self._photo_requested = False
        self._photo_sending = False

        # ── BLE state ───────────────────────────────────────────────
        self._connected = False
        self._ble_client: Optional[BleakClient] = None
        self._ble_lock = threading.Lock()
        self._rx_buf = ""
        self._stop_evt = threading.Event()

        if self._auto_power_on_bt:
            self._ensure_bluetooth_ready()

        self.create_timer(1.0 / max(self._publish_hz, 1e-3), self._publish_drive_tick)
        self.create_timer(1.0 / max(self._telemetry_hz, 1e-3), self._telemetry_tick)

        self._ble_thread = threading.Thread(target=self._ble_thread_main, daemon=True)
        self._ble_thread.start()

        self.get_logger().info(
            f"Bluetooth teleop started: target name='{self._device_name}'"
            + (f" address='{self._device_address}'" if self._device_address else "")
        )

    # ── Bluetooth bootstrap ─────────────────────────────────────────
    def _ensure_bluetooth_ready(self) -> None:
        cmds = [
            ["bluetoothctl", "power", "on"],
            ["bluetoothctl", "agent", "NoInputNoOutput"],
            ["bluetoothctl", "default-agent"],
        ]
        if self._device_address:
            cmds.append(["bluetoothctl", "trust", self._device_address])
        for cmd in cmds:
            try:
                subprocess.run(cmd, check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception:
                pass

    # ── ROS callbacks ───────────────────────────────────────────────
    def _on_imu(self, msg: Imu) -> None:
        self._gyro_z = float(msg.angular_velocity.z)
        # Derive roll/pitch from quaternion
        q = msg.orientation
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self._orient.roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1.0:
            self._orient.pitch = math.copysign(90.0, sinp)
        else:
            self._orient.pitch = math.degrees(math.asin(sinp))

    def _on_yaw(self, msg: Float64) -> None:
        self._yaw_heading = float(msg.data)

    def _on_mag(self, msg: Vector3) -> None:
        self._mag_x = float(msg.x)
        self._mag_y = float(msg.y)

    def _on_camera_image(self, msg: CompressedImage) -> None:
        """Cache the most recent camera JPEG for photo capture requests."""
        with self._camera_lock:
            self._last_camera_jpeg = bytes(msg.data)

    def _on_front_image(self, msg: CompressedImage) -> None:
        """Cache the most recent front camera JPEG."""
        with self._camera_lock:
            self._last_front_jpeg = bytes(msg.data)

    # ── Drive publishing ────────────────────────────────────────────
    def _publish_drive_tick(self) -> None:
        # Don't publish at all if BLE is not connected — avoids
        # flooding cmd_vel with zeros and overriding other sources
        if not self._connected:
            return

        now = time.monotonic()
        deadman_expired = (
            self._deadman > 0.0 and (now - self._last_cmd_time) > self._deadman
        )

        if deadman_expired or self._last_cmd_time == 0.0:
            # Deadman tripped or never received a command.
            # Send exactly ONE zero-stop, then go silent so other
            # cmd_vel sources (web UI, follow-mode) aren't overridden.
            if self._bt_active and not self._sent_stop:
                self._publish_twist(0.0, 0.0, 0.0)
                self._sent_stop = True
                self._bt_active = False
            return

        # Active BT command — publish it
        self._bt_active = True
        self._sent_stop = False
        self._publish_twist(self._motion.vx, self._motion.vy, self._motion.wz)

    def _publish_twist(self, vx: float, vy: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self._cmd_vel_pub.publish(msg)

    def _publish_gimbal(self, pan: float, tilt: float) -> None:
        msg = Vector3()
        msg.x = float(max(0.0, min(180.0, pan)))
        msg.y = float(max(0.0, min(110.0, tilt)))
        msg.z = 0.0
        self._gimbal_pub.publish(msg)

    def _set_follow(self, enabled: bool) -> None:
        self._follow_enabled = bool(enabled)
        msg = Bool()
        msg.data = self._follow_enabled
        self._follow_pub.publish(msg)

    # ── Parse Cardputer → Pi commands ───────────────────────────────
    def _parse_line(self, line: str) -> None:
        parts = line.strip().split("|")
        if len(parts) < 2 or parts[0] != "CMD":
            return

        now = time.monotonic()
        kind = parts[1]

        try:
            if kind == "VEL" and len(parts) >= 5:
                vx = float(parts[2])
                vy = float(parts[3])
                wz = float(parts[4])
                has_motion = abs(vx) > 1e-4 or abs(vy) > 1e-4 or abs(wz) > 1e-4
                self._motion.vx = vx
                self._motion.vy = vy
                self._motion.wz = wz
                if has_motion:
                    self._last_cmd_time = now
                    self._bt_active = True
                    self._sent_stop = False
            elif kind == "GMB" and len(parts) >= 4:
                if self._gimbal_control_enabled:
                    self._gimbal_pan = float(parts[2])
                    self._gimbal_tilt = float(parts[3])
                    self._publish_gimbal(self._gimbal_pan, self._gimbal_tilt)
            elif kind == "FOL" and len(parts) >= 3:
                self._set_follow(parts[2] == "1")
            elif kind == "GMODE" and len(parts) >= 3:
                self._gimbal_control_enabled = parts[2] == "1"
            elif kind == "PHOTO":
                self._photo_requested = True
            elif kind == "PING":
                pass  # keepalive; don't bump _last_cmd_time (only motion does)
        except Exception:
            return

    # ── Telemetry: Pi → Cardputer ───────────────────────────────────
    def _telemetry_tick(self) -> None:
        speed = math.hypot(self._motion.vx, self._motion.vy)
        compass = math.degrees(math.atan2(self._mag_y, self._mag_x))
        if compass < 0.0:
            compass += 360.0

        # Extended TEL with roll and pitch
        line = (
            f"TEL|{self._motion.vx:.3f}|{self._motion.vy:.3f}|{self._motion.wz:.3f}|"
            f"{speed:.3f}|{self._gyro_z:.3f}|{compass:.1f}|{self._yaw_heading:.1f}|"
            f"{1 if self._follow_enabled else 0}|{1 if self._gimbal_control_enabled else 0}|"
            f"{self._orient.roll:.1f}|{self._orient.pitch:.1f}\n"
        )
        self._ble_send(line)

        # Handle pending photo request (after telemetry so we don't block it)
        if self._photo_requested and not self._photo_sending:
            self._photo_requested = False
            self._photo_sending = True
            threading.Thread(target=self._handle_photo_capture, daemon=True).start()

    # ── Photo capture & transfer ────────────────────────────────────
    def _resize_jpeg(self, jpeg_data: bytes) -> bytes:
        """Resize a JPEG to fit the Cardputer display with low quality."""
        if PILImage is None:
            # No Pillow — send raw but warn
            self.get_logger().warn("Pillow not available; sending full-size JPEG")
            return jpeg_data
        try:
            img = PILImage.open(io.BytesIO(jpeg_data))
            img.thumbnail((self._photo_w, self._photo_h), PILImage.LANCZOS)
            out = io.BytesIO()
            img.save(out, format="JPEG", quality=self._photo_quality)
            return out.getvalue()
        except Exception as exc:
            self.get_logger().warn(f"JPEG resize failed: {exc}")
            return jpeg_data

    def _handle_photo_capture(self) -> None:
        """Grab the latest camera frames, downscale, and send over BLE sequentially."""
        try:
            with self._camera_lock:
                raw_main = self._last_camera_jpeg
                raw_front = self._last_front_jpeg

            # List of images to send
            images_to_send = []
            if raw_main:
                images_to_send.append(("Main", raw_main))
            if raw_front:
                images_to_send.append(("Front", raw_front))
            
            if not images_to_send:
                self._ble_send("IMG|ERROR|No camera images available\n")
                self.get_logger().warn("Photo requested but no camera frames cached")
                return

            for i, (name, raw_data) in enumerate(images_to_send):
                # If sending multiple, add delay between them to let Cardputer save to SD
                if i > 0:
                    self.get_logger().info("Waiting for Cardputer to save previous photo...")
                    time.sleep(3.0)

                small_jpeg = self._resize_jpeg(raw_data)
                total = len(small_jpeg)
                self.get_logger().info(f"Sending {name} photo: {total} bytes to Cardputer")

                # Send header
                self._ble_send(f"IMG|BEGIN|{total}\n")
                time.sleep(0.05)

                # Send binary chunks
                offset = 0
                while offset < total:
                    end = min(offset + BLE_CHUNK_SIZE, total)
                    chunk = small_jpeg[offset:end]
                    self._ble_send_bytes(chunk)
                    offset = end
                    # Small delay between chunks to avoid BLE congestion
                    time.sleep(0.015)
                
                self.get_logger().info(f"{name} photo transfer complete")

        except Exception as exc:
            self.get_logger().warn(f"Photo transfer error: {exc}")
            try:
                self._ble_send(f"IMG|ERROR|{exc}\n")
            except Exception:
                pass
        finally:
            self._photo_sending = False

    # ── BLE send helpers ────────────────────────────────────────────
    def _ble_send(self, payload: str) -> None:
        with self._ble_lock:
            client = self._ble_client
        if client is None or not self._connected:
            return
        try:
            future = asyncio.run_coroutine_threadsafe(
                client.write_gatt_char(RX_WRITE_UUID, payload.encode("utf-8"), response=False),
                self._ble_loop,
            )
            future.result(timeout=0.4)
        except Exception:
            pass

    def _ble_send_bytes(self, data: bytes) -> None:
        """Send raw binary data over BLE (for JPEG chunks)."""
        with self._ble_lock:
            client = self._ble_client
        if client is None or not self._connected:
            return
        try:
            future = asyncio.run_coroutine_threadsafe(
                client.write_gatt_char(RX_WRITE_UUID, data, response=False),
                self._ble_loop,
            )
            future.result(timeout=0.8)
        except Exception:
            pass

    # ── BLE notification handler ────────────────────────────────────
    def _on_notify(self, _sender: int, data: bytearray) -> None:
        try:
            chunk = data.decode("utf-8", errors="ignore")
        except Exception:
            return
        self._rx_buf += chunk
        while "\n" in self._rx_buf:
            line, self._rx_buf = self._rx_buf.split("\n", 1)
            if line:
                self._parse_line(line)

    # ── Bonding management ──────────────────────────────────────────
    def _clear_bonding(self, address: str) -> None:
        """Remove BlueZ pairing/bonding for a device."""
        try:
            subprocess.run(
                ["bluetoothctl", "remove", address],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=5,
            )
        except Exception:
            pass

    # ── BLE connection lifecycle ────────────────────────────────────
    async def _scan_and_connect(self) -> Optional[BleakClient]:
        if BleakScanner is None or BleakClient is None:
            if not hasattr(self, "_bleak_warned"):
                self._bleak_warned = True
                self.get_logger().error(
                    "Python package 'bleak' is missing. Install with: pip install bleak"
                )
            await asyncio.sleep(self._reconnect_delay)
            return None

        target = None
        if self._device_address:
            target = await BleakScanner.find_device_by_address(
                self._device_address, timeout=self._scan_timeout
            )
        else:
            target = await BleakScanner.find_device_by_name(
                self._device_name, timeout=self._scan_timeout
            )

        if target is None:
            return None

        client = BleakClient(target, timeout=max(self._scan_timeout, 4.0))
        await client.connect()
        await client.start_notify(TX_NOTIFY_UUID, self._on_notify)
        return client

    async def _ble_loop_task(self) -> None:
        while not self._stop_evt.is_set():
            try:
                client = await self._scan_and_connect()
                if client is None:
                    await asyncio.sleep(self._reconnect_delay)
                    continue

                with self._ble_lock:
                    self._ble_client = client
                self._connected = True
                self.get_logger().info("Cardputer BLE connected")

                while client.is_connected and not self._stop_evt.is_set():
                    await asyncio.sleep(0.2)

            except Exception as exc:
                err_msg = str(exc)
                self.get_logger().warn(f"BLE connection error: {err_msg}")
                if "failed to discover services" in err_msg:
                    addr = self._device_address or "30:ED:A0:CA:17:B5"
                    self.get_logger().info(
                        f"Clearing stale bonding for {addr} and retrying"
                    )
                    self._clear_bonding(addr)
            finally:
                self._connected = False
                self._bt_active = False
                self._sent_stop = True
                self._last_cmd_time = 0.0
                self._motion = MotionState()
                with self._ble_lock:
                    old = self._ble_client
                    self._ble_client = None
                if old is not None:
                    try:
                        await old.disconnect()
                    except Exception:
                        pass
                self._publish_twist(0.0, 0.0, 0.0)
                await asyncio.sleep(self._reconnect_delay)

    def _ble_thread_main(self) -> None:
        self._ble_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._ble_loop)
        self._ble_loop.run_until_complete(self._ble_loop_task())

    def destroy_node(self):
        self._stop_evt.set()
        try:
            self._publish_twist(0.0, 0.0, 0.0)
        except Exception:
            pass
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = BtCardputerTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
