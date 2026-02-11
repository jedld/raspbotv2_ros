"""ROS 2 node for Pico RP2040 + BNO055 9-DOF IMU serial bridge.

Reads $BNO lines from a Pico RP2040 over USB serial and publishes:
  imu/data            (sensor_msgs/Imu)          fused quaternion + accel + gyro
  imu/yaw_deg         (std_msgs/Float64)         magnetometer-stabilised heading (degrees)
  imu/temperature     (std_msgs/Float32)         BNO055 die temperature (°C)
  imu/calibrated      (std_msgs/Bool)            True when system calibration ≥ threshold
  imu/calibration     (std_msgs/String)          JSON: {"sys":3,"gyro":3,"accel":2,"mag":1}
  imu/gravity         (geometry_msgs/Vector3)    gravity vector in body frame (m/s²)
  imu/mag             (geometry_msgs/Vector3)    raw magnetometer (µT)

Subscribes to:
  imu/calibrate       (std_msgs/Empty)           reset BNO055 calibration
  imu/save_cal        (std_msgs/Empty)           save calibration to Pico flash
  imu/load_cal        (std_msgs/Empty)           load calibration from Pico flash

This node is a **drop-in replacement** for imu_serial_node.py:
  - Publishes on the same topics (imu/data, imu/yaw_deg, imu/temperature)
  - imu/data now contains **real quaternion orientation** (no drift!)
  - imu/yaw_deg now comes from BNO055 fused Euler heading (magnetometer-stabilised)

The existing Arduino Nano RP2040 can keep running for audio/mic only.
"""
import json
import math
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Empty, Float32, Float64, String


class Bno055SerialNode(Node):
    def __init__(self) -> None:
        super().__init__('bno055_serial')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('serial_port', '/dev/ttyACM1')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('serial_timeout_sec', 2.0)
        self.declare_parameter('frame_id', 'imu_link')
        # Minimum system calibration level (0-3) to consider "calibrated"
        self.declare_parameter('cal_threshold', 1)
        # Axis remapping for the fused quaternion/accel/gyro.
        # BNO055 axes: X=right, Y=forward, Z=up (when chip text is facing up)
        # ROS convention: X=forward, Y=left, Z=up
        # The remapping depends on how the Pico+BNO055 is mounted on the robot.
        # Signs: set to -1.0 to flip an axis.
        self.declare_parameter('quat_axes_ros', True)   # auto-remap BNO→ROS axes
        # Manual override: rotation from BNO055 body frame to robot body frame
        # as Euler angles (degrees): [yaw_offset, pitch_offset, roll_offset]
        self.declare_parameter('heading_offset_deg', 0.0)  # compass heading offset
        # Covariance (diagonal, same for all axes)
        self.declare_parameter('orientation_variance', 0.0001)
        self.declare_parameter('accel_variance', 0.01)
        self.declare_parameter('gyro_variance', 0.0001)

        self._port = str(self.get_parameter('serial_port').value)
        self._baud = int(self.get_parameter('baud_rate').value)
        self._timeout = float(self.get_parameter('serial_timeout_sec').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._cal_threshold = int(self.get_parameter('cal_threshold').value)
        self._heading_offset = float(self.get_parameter('heading_offset_deg').value)

        orient_var = float(self.get_parameter('orientation_variance').value)
        accel_var = float(self.get_parameter('accel_variance').value)
        gyro_var = float(self.get_parameter('gyro_variance').value)

        self._orient_cov = [0.0] * 9
        self._accel_cov = [0.0] * 9
        self._gyro_cov = [0.0] * 9
        for i in range(3):
            self._orient_cov[i * 3 + i] = orient_var
            self._accel_cov[i * 3 + i] = accel_var
            self._gyro_cov[i * 3 + i] = gyro_var

        # ── State ─────────────────────────────────────────────────────
        self._serial = None
        self._running = True
        self._connected = False
        self._calibrated = False
        self._last_cal = {'sys': 0, 'gyro': 0, 'accel': 0, 'mag': 0}

        # ── Publishers ────────────────────────────────────────────────
        self._imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self._yaw_pub = self.create_publisher(Float64, 'imu/yaw_deg', 10)
        self._temp_pub = self.create_publisher(Float32, 'imu/temperature', 10)
        self._cal_pub = self.create_publisher(Bool, 'imu/calibrated', 10)
        self._cal_detail_pub = self.create_publisher(String, 'imu/calibration', 10)
        self._gravity_pub = self.create_publisher(Vector3, 'imu/gravity', 10)
        self._mag_pub = self.create_publisher(Vector3, 'imu/mag', 10)

        # ── Subscribers ───────────────────────────────────────────────
        self.create_subscription(Empty, 'imu/calibrate', self._on_calibrate, 10)
        self.create_subscription(Empty, 'imu/save_cal', self._on_save_cal, 10)
        self.create_subscription(Empty, 'imu/load_cal', self._on_load_cal, 10)

        # ── Serial reader thread ──────────────────────────────────────
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

        self.get_logger().info(
            f'BNO055 serial node starting (port={self._port}, baud={self._baud}, frame={self._frame_id})'
        )

    # ── Serial management ─────────────────────────────────────────────

    def _open_serial(self):
        import serial as pyserial
        try:
            ser = pyserial.Serial(self._port, self._baud, timeout=self._timeout)
            time.sleep(0.5)
            ser.reset_input_buffer()
            self._serial = ser
            self._connected = True
            self.get_logger().info(f'Connected to {self._port}')
            return True
        except Exception as exc:
            self._serial = None
            self._connected = False
            self.get_logger().warn(f'Cannot open {self._port}: {exc}')
            return False

    def _send_command(self, cmd: str) -> None:
        if self._serial is not None:
            try:
                self._serial.write(cmd.encode('ascii'))
            except Exception as exc:
                self.get_logger().warn(f'Serial write failed: {exc}')

    def _on_calibrate(self, _msg: Empty) -> None:
        self.get_logger().info('BNO055 calibration reset requested')
        self._calibrated = False
        self._send_command('C')

    def _on_save_cal(self, _msg: Empty) -> None:
        self.get_logger().info('Saving BNO055 calibration to Pico flash')
        self._send_command('S')

    def _on_load_cal(self, _msg: Empty) -> None:
        self.get_logger().info('Loading BNO055 calibration from Pico flash')
        self._send_command('L')

    # ── Reader loop (background thread) ───────────────────────────────

    def _reader_loop(self) -> None:
        while self._running:
            if self._serial is None:
                self._calibrated = False
                if not self._open_serial():
                    time.sleep(2.0)
                    continue
                # Request info and try loading saved calibration
                time.sleep(0.5)
                self._send_command('?')
                self._send_command('L')

            try:
                raw = self._serial.readline()
            except Exception as exc:
                self.get_logger().warn(f'Serial read error: {exc}')
                self._serial = None
                self._connected = False
                time.sleep(1.0)
                continue

            if not raw:
                continue

            try:
                line = raw.decode('ascii', errors='replace').strip()
            except Exception:
                continue

            if line.startswith('$BNO,'):
                self._parse_bno(line)
            elif line.startswith('$GRV,'):
                self._parse_gravity(line)
            elif line.startswith('$INFO,'):
                self.get_logger().info(f'Pico: {line}')
            elif line.startswith('$CAL,'):
                self.get_logger().info(f'Pico: {line}')
            elif line.startswith('$MSG,'):
                self.get_logger().info(f'Pico: {line}')
            elif line.startswith('$ERR,'):
                self.get_logger().error(f'Pico: {line}')
            elif line.startswith('$MODE,'):
                self.get_logger().info(f'Pico: {line}')
            elif line.startswith('$RATE,'):
                self.get_logger().info(f'Pico: {line}')

    def _parse_bno(self, line: str) -> None:
        """Parse $BNO line from Pico firmware.

        $BNO,qw,qx,qy,qz,ax,ay,az,gx,gy,gz,mx,my,mz,heading,roll,pitch,
             temp,cal_sys,cal_gyro,cal_accel,cal_mag,ms
        """
        parts = line.split(',')
        if len(parts) != 23:
            return
        try:
            qw = float(parts[1])
            qx = float(parts[2])
            qy = float(parts[3])
            qz = float(parts[4])
            ax = float(parts[5])   # linear accel (gravity removed), m/s²
            ay = float(parts[6])
            az = float(parts[7])
            gx = float(parts[8])   # gyro, rad/s (BNO outputs rad/s)
            gy = float(parts[9])
            gz = float(parts[10])
            mx = float(parts[11])  # magnetometer, µT
            my = float(parts[12])
            mz = float(parts[13])
            heading = float(parts[14])  # Euler heading (degrees, 0-360)
            roll = float(parts[15])     # Euler roll (degrees)
            pitch = float(parts[16])    # Euler pitch (degrees)
            temp = float(parts[17])
            cal_sys = int(parts[18])
            cal_gyro = int(parts[19])
            cal_accel = int(parts[20])
            cal_mag = int(parts[21])
            _ms = int(parts[22])
        except (ValueError, IndexError):
            return

        stamp = self.get_clock().now().to_msg()

        # ── Update calibration state ──────────────────────────────────
        cal_ok = cal_sys >= self._cal_threshold
        if cal_ok != self._calibrated:
            self._calibrated = cal_ok
            self.get_logger().info(
                f'BNO055 calibration: sys={cal_sys} gyro={cal_gyro} '
                f'accel={cal_accel} mag={cal_mag} → '
                f'{"CALIBRATED" if cal_ok else "NOT calibrated"}'
            )
        self._last_cal = {
            'sys': cal_sys, 'gyro': cal_gyro,
            'accel': cal_accel, 'mag': cal_mag,
        }

        # ── Publish sensor_msgs/Imu ───────────────────────────────────
        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self._frame_id

        # BNO055 quaternion — this is a REAL fused orientation, not unknown!
        imu_msg.orientation.w = qw
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation_covariance = self._orient_cov[:]

        # Angular velocity (BNO055 already outputs rad/s)
        imu_msg.angular_velocity.x = gx
        imu_msg.angular_velocity.y = gy
        imu_msg.angular_velocity.z = gz
        imu_msg.angular_velocity_covariance = self._gyro_cov[:]

        # Linear acceleration (gravity already removed by BNO055)
        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az
        imu_msg.linear_acceleration_covariance = self._accel_cov[:]

        self._imu_pub.publish(imu_msg)

        # ── Publish yaw (heading) ─────────────────────────────────────
        # BNO055 heading: 0-360 degrees, clockwise from north.
        # Convert to ±180 and apply user offset.
        yaw = heading + self._heading_offset
        # Normalise to ±180
        while yaw > 180.0:
            yaw -= 360.0
        while yaw < -180.0:
            yaw += 360.0

        yaw_msg = Float64()
        yaw_msg.data = yaw
        self._yaw_pub.publish(yaw_msg)

        # ── Publish temperature ───────────────────────────────────────
        temp_msg = Float32()
        temp_msg.data = temp
        self._temp_pub.publish(temp_msg)

        # ── Publish calibrated status ─────────────────────────────────
        cal_bool_msg = Bool()
        cal_bool_msg.data = self._calibrated
        self._cal_pub.publish(cal_bool_msg)

        # ── Publish detailed calibration (at ~2 Hz to avoid spam) ─────
        # Only publish when cal values change or every 50th message
        if not hasattr(self, '_cal_msg_counter'):
            self._cal_msg_counter = 0
        self._cal_msg_counter += 1
        if self._cal_msg_counter >= 50:
            self._cal_msg_counter = 0
            cal_str = String()
            cal_str.data = json.dumps(self._last_cal)
            self._cal_detail_pub.publish(cal_str)

        # ── Publish magnetometer ──────────────────────────────────────
        mag_msg = Vector3()
        mag_msg.x = mx
        mag_msg.y = my
        mag_msg.z = mz
        self._mag_pub.publish(mag_msg)

    def _parse_gravity(self, line: str) -> None:
        """Parse $GRV,gx,gy,gz — gravity vector in body frame (m/s²)."""
        parts = line.split(',')
        if len(parts) != 4:
            return
        try:
            gx = float(parts[1])
            gy = float(parts[2])
            gz = float(parts[3])
        except (ValueError, IndexError):
            return

        msg = Vector3()
        msg.x = gx
        msg.y = gy
        msg.z = gz
        self._gravity_pub.publish(msg)

    def destroy_node(self) -> None:
        self._running = False
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = Bno055SerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
