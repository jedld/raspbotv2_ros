import asyncio
import math
import subprocess
import threading
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float64

try:
    from bleak import BleakClient, BleakScanner
except Exception:  # pragma: no cover
    BleakClient = None
    BleakScanner = None


SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
TX_NOTIFY_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  # Cardputer -> Pi
RX_WRITE_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"   # Pi -> Cardputer


@dataclass
class MotionState:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


class BtCardputerTeleop(Node):
    def __init__(self) -> None:
        super().__init__("bt_cardputer_teleop")

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
        self._auto_power_on_bt = bool(self.get_parameter("auto_power_on_bluetooth").value)
        self._auto_connect_paired = bool(self.get_parameter("auto_connect_paired_device").value)

        self._cmd_vel_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._gimbal_pub = self.create_publisher(Vector3, self._gimbal_topic, 10)
        self._follow_pub = self.create_publisher(Bool, self._follow_enable_topic, 10)

        self.create_subscription(Imu, self._imu_topic, self._on_imu, 10)
        self.create_subscription(Float64, self._imu_yaw_topic, self._on_yaw, 10)
        self.create_subscription(Vector3, self._imu_mag_topic, self._on_mag, 10)

        self._motion = MotionState()
        self._last_cmd_time = 0.0
        self._follow_enabled = False
        self._gimbal_control_enabled = False
        self._gimbal_pan = 90.0
        self._gimbal_tilt = 45.0

        self._gyro_z = 0.0
        self._yaw_heading = 0.0
        self._mag_x = 0.0
        self._mag_y = 0.0

        self._connected = False
        self._ble_client = None
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

    def _on_imu(self, msg: Imu) -> None:
        self._gyro_z = float(msg.angular_velocity.z)

    def _on_yaw(self, msg: Float64) -> None:
        self._yaw_heading = float(msg.data)

    def _on_mag(self, msg: Vector3) -> None:
        self._mag_x = float(msg.x)
        self._mag_y = float(msg.y)

    def _publish_drive_tick(self) -> None:
        now = time.monotonic()
        if self._deadman > 0.0 and (now - self._last_cmd_time) > self._deadman:
            vx = 0.0
            vy = 0.0
            wz = 0.0
        else:
            vx = self._motion.vx
            vy = self._motion.vy
            wz = self._motion.wz
        self._publish_twist(vx, vy, wz)

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

    def _parse_line(self, line: str) -> None:
        # Format: CMD|TYPE|...
        parts = line.strip().split("|")
        if len(parts) < 2 or parts[0] != "CMD":
            return

        now = time.monotonic()
        kind = parts[1]

        try:
            if kind == "VEL" and len(parts) >= 5:
                self._motion.vx = float(parts[2])
                self._motion.vy = float(parts[3])
                self._motion.wz = float(parts[4])
                self._last_cmd_time = now
            elif kind == "GMB" and len(parts) >= 4:
                if self._gimbal_control_enabled:
                    self._gimbal_pan = float(parts[2])
                    self._gimbal_tilt = float(parts[3])
                    self._publish_gimbal(self._gimbal_pan, self._gimbal_tilt)
            elif kind == "FOL" and len(parts) >= 3:
                self._set_follow(parts[2] == "1")
            elif kind == "GMODE" and len(parts) >= 3:
                self._gimbal_control_enabled = parts[2] == "1"
            elif kind == "PING":
                self._last_cmd_time = now
        except Exception:
            return

    def _telemetry_tick(self) -> None:
        speed = math.hypot(self._motion.vx, self._motion.vy)
        compass = math.degrees(math.atan2(self._mag_y, self._mag_x))
        if compass < 0.0:
            compass += 360.0

        line = (
            f"TEL|{self._motion.vx:.3f}|{self._motion.vy:.3f}|{self._motion.wz:.3f}|"
            f"{speed:.3f}|{self._gyro_z:.3f}|{compass:.1f}|{self._yaw_heading:.1f}|"
            f"{1 if self._follow_enabled else 0}|{1 if self._gimbal_control_enabled else 0}\n"
        )
        self._ble_send(line)

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

    def _clear_bonding(self, address: str) -> None:
        """Remove BlueZ pairing/bonding for a device.

        After reflashing the ESP32, stored encryption keys become invalid
        and cause 'failed to discover services, device disconnected' errors.
        Removing the device forces BlueZ to forget the stale keys.
        """
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
            target = await BleakScanner.find_device_by_address(self._device_address, timeout=self._scan_timeout)
        else:
            target = await BleakScanner.find_device_by_name(self._device_name, timeout=self._scan_timeout)

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
                # Stale bonding keys cause "failed to discover services"
                # — clear them so the next attempt starts fresh.
                if "failed to discover services" in err_msg:
                    addr = self._device_address or "30:ED:A0:CA:17:B5"
                    self.get_logger().info(
                        f"Clearing stale bonding for {addr} and retrying"
                    )
                    self._clear_bonding(addr)
            finally:
                self._connected = False
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
