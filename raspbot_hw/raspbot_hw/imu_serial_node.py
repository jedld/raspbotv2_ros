"""ROS 2 node for Arduino Nano RP2040 Connect IMU serial bridge.

Reads $IMU and (optionally) $BNO lines from USB serial and publishes:
  imu/data            (sensor_msgs/Imu)          fused quaternion + accel + gyro (BNO055)
                                                  OR accel + gyro only (LSM6DSOX fallback)
  imu/yaw_deg         (std_msgs/Float64)         magnetometer heading (BNO055)
                                                  OR integrated gyro yaw (fallback)
  imu/temperature     (std_msgs/Float32)         die temperature °C
  imu/mic_level       (std_msgs/Int32)           microphone RMS
  imu/calibrated      (std_msgs/Bool)            calibration status
  imu/calibration     (std_msgs/String)          BNO055 cal detail JSON
  imu/gravity         (geometry_msgs/Vector3)    gravity vector (BNO055)
  imu/mag             (geometry_msgs/Vector3)    magnetometer µT (BNO055)

Subscribes to:
  imu/calibrate       (std_msgs/Empty)           trigger gyro re-calibration
  imu/led             (std_msgs/String)           set Arduino RGB LED (R/G/B/W/O)
  imu/save_cal        (std_msgs/Empty)           save BNO055 calibration
  imu/load_cal        (std_msgs/Empty)           load BNO055 calibration

When an external BNO055 9-DOF sensor is connected to the Arduino's A4/A5
pins, the firmware (v2.0.0+) sends $BNO lines with fused quaternion
orientation.  imu/data will then carry **real orientation** (no drift)
and imu/yaw_deg will use the magnetometer-stabilised heading.
If no BNO055 is detected, everything falls back to the on-board LSM6DSOX
with gyro-only yaw integration (same as firmware v1.x behaviour).
"""
import base64
import json
import math
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Empty, Float32, Float64, Int32, String, UInt8MultiArray


G_MPS2 = 9.80665  # 1 g in m/s²
DEG_TO_RAD = math.pi / 180.0


class ImuSerialNode(Node):
    def __init__(self) -> None:
        super().__init__('imu_serial')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('serial_timeout_sec', 2.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_hz', 100.0)
        # Delay before sending calibrate command after serial connects (seconds).
        # Lets the Arduino + USB link settle.
        self.declare_parameter('startup_calibrate_delay_sec', 1.0)
        # Axis remapping: indices into [ax, ay, az] / [gx, gy, gz] (0/1/2)
        # and sign multiplier. Default assumes board flat, USB rear:
        #   board X → ROS X (forward), board Y → ROS Y (left), board Z → ROS Z (up)
        self.declare_parameter('accel_axis_map', [0, 1, 2])
        self.declare_parameter('accel_axis_sign', [1.0, 1.0, 1.0])
        self.declare_parameter('gyro_axis_map', [0, 1, 2])
        self.declare_parameter('gyro_axis_sign', [1.0, 1.0, 1.0])
        # Yaw integration
        self.declare_parameter('enable_yaw_integration', True)
        self.declare_parameter('yaw_gyro_axis', 2)  # which gyro axis is yaw (Z=2)
        self.declare_parameter('yaw_gyro_sign', 1.0)
        # Covariance (diagonal, same for all axes)
        self.declare_parameter('accel_variance', 0.01)
        self.declare_parameter('gyro_variance', 0.001)
        # BNO055 parameters (used when firmware reports $BNO lines)
        self.declare_parameter('orientation_variance', 0.0001)
        self.declare_parameter('bno_cal_threshold', 1)  # min sys cal (0-3) for calibrated=True
        self.declare_parameter('heading_offset_deg', 0.0)  # compass heading offset

        self._port = str(self.get_parameter('serial_port').value)
        self._baud = int(self.get_parameter('baud_rate').value)
        self._timeout = float(self.get_parameter('serial_timeout_sec').value)
        self._frame_id = str(self.get_parameter('frame_id').value)

        self._accel_map = list(self.get_parameter('accel_axis_map').value)
        self._accel_sign = list(self.get_parameter('accel_axis_sign').value)
        self._gyro_map = list(self.get_parameter('gyro_axis_map').value)
        self._gyro_sign = list(self.get_parameter('gyro_axis_sign').value)

        self._enable_yaw = bool(self.get_parameter('enable_yaw_integration').value)
        self._yaw_axis = int(self.get_parameter('yaw_gyro_axis').value)
        self._yaw_sign = float(self.get_parameter('yaw_gyro_sign').value)

        accel_var = float(self.get_parameter('accel_variance').value)
        gyro_var = float(self.get_parameter('gyro_variance').value)

        self._accel_cov = [0.0] * 9
        self._gyro_cov = [0.0] * 9
        for i in range(3):
            self._accel_cov[i * 3 + i] = accel_var
            self._gyro_cov[i * 3 + i] = gyro_var

        orient_var = float(self.get_parameter('orientation_variance').value)
        self._orient_cov = [0.0] * 9
        for i in range(3):
            self._orient_cov[i * 3 + i] = orient_var

        self._bno_cal_threshold = int(self.get_parameter('bno_cal_threshold').value)
        self._heading_offset = float(self.get_parameter('heading_offset_deg').value)

        # ── State ─────────────────────────────────────────────────────
        self._yaw_deg = 0.0
        self._last_imu_time: Optional[float] = None
        self._serial = None
        self._running = True
        self._connected = False
        self._calibrated = False  # True after gyro calibration completes
        # Accel bias from calibration (raw g units, pre-axis-remap)
        self._accel_bias = [0.0, 0.0, 0.0]
        self._bno_available = False  # True once we receive a $BNO line
        self._bno_cal = {'sys': 0, 'gyro': 0, 'accel': 0, 'mag': 0}
        self._bno_msg_counter = 0

        # ── Publishers ────────────────────────────────────────────────
        self._imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self._temp_pub = self.create_publisher(Float32, 'imu/temperature', 10)
        self._mic_pub = self.create_publisher(Int32, 'imu/mic_level', 10)
        self._yaw_pub = self.create_publisher(Float64, 'imu/yaw_deg', 10)
        self._cal_pub = self.create_publisher(Bool, 'imu/calibrated', 10)

        # ── BNO055 publishers ──────────────────────────────────────────
        self._cal_detail_pub = self.create_publisher(String, 'imu/calibration', 10)
        self._gravity_pub = self.create_publisher(Vector3, 'imu/gravity', 10)
        self._mag_pub = self.create_publisher(Vector3, 'imu/mag', 10)

        # ── Audio streaming ───────────────────────────────────────────
        self.declare_parameter('audio_sample_rate', 8000)
        self._audio_sample_rate = int(self.get_parameter('audio_sample_rate').value)
        self._audio_pub = self.create_publisher(UInt8MultiArray, 'imu/audio', 10)
        self._audio_enabled = False

        # ── Subscribers ─────────────────────────────────────────────
        self.create_subscription(Empty, 'imu/calibrate', self._on_calibrate, 10)
        self.create_subscription(String, 'imu/led', self._on_led, 10)
        self.create_subscription(Bool, 'imu/audio_enable', self._on_audio_enable, 10)
        self.create_subscription(Empty, 'imu/save_cal', self._on_save_cal, 10)
        self.create_subscription(Empty, 'imu/load_cal', self._on_load_cal, 10)

        # ── Serial reader thread ──────────────────────────────────────
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

        self.get_logger().info(
            f'IMU serial node starting (port={self._port}, baud={self._baud}, frame={self._frame_id})'
        )

    # ── Serial management ─────────────────────────────────────────────

    def _open_serial(self):
        import serial as pyserial
        try:
            ser = pyserial.Serial(self._port, self._baud, timeout=self._timeout)
            time.sleep(0.3)
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
        self.get_logger().info('Gyro calibration requested — hold robot still for 2 seconds')
        self._calibrated = False
        self._publish_calibrated(False)
        self._send_command('C')
        self._yaw_deg = 0.0

    def _publish_calibrated(self, state: bool) -> None:
        msg = Bool()
        msg.data = state
        self._cal_pub.publish(msg)

    def _on_led(self, msg: String) -> None:
        c = str(msg.data).strip().upper()
        if c in ('R', 'G', 'B', 'W', 'O'):
            self._send_command(c)

    def _on_audio_enable(self, msg: Bool) -> None:
        enable = bool(msg.data)
        if enable != self._audio_enabled:
            self._audio_enabled = enable
            self._send_command('A' if enable else 'a')
            self.get_logger().info(f'Audio streaming {"enabled" if enable else "disabled"}')

    def _on_save_cal(self, _msg: Empty) -> None:
        self.get_logger().info('Saving BNO055 calibration on Arduino')
        self._send_command('S')

    def _on_load_cal(self, _msg: Empty) -> None:
        self.get_logger().info('Loading BNO055 calibration on Arduino')
        self._send_command('L')

    def _parse_audio(self, line: str) -> None:
        """Parse $AUD,<base64_data> and publish raw PCM bytes."""
        try:
            b64_data = line[5:]  # skip "$AUD,"
            raw = base64.b64decode(b64_data)
            msg = UInt8MultiArray()
            msg.data = list(raw)
            self._audio_pub.publish(msg)
        except Exception:
            pass

    # ── Reader loop (background thread) ───────────────────────────────

    def _reader_loop(self) -> None:
        calibrate_delay = float(self.get_parameter('startup_calibrate_delay_sec').value)
        need_calibration = True  # always calibrate on first connect

        while self._running:
            if self._serial is None:
                # Mark uncalibrated whenever the serial link drops
                self._calibrated = False
                self._publish_calibrated(False)
                need_calibration = True
                if not self._open_serial():
                    time.sleep(2.0)
                    continue

            # Auto-calibrate after every (re)connection
            if need_calibration:
                time.sleep(calibrate_delay)
                self.get_logger().info(
                    'Sending gyro calibration (auto on connect) — hold robot still for 2 s'
                )
                self._publish_calibrated(False)
                self._accel_bias = [0.0, 0.0, 0.0]
                self._send_command('C')
                # Actively read lines while waiting for $CAL,done (up to 5 s)
                cal_deadline = time.monotonic() + 5.0
                cal_ok = False
                while time.monotonic() < cal_deadline:
                    try:
                        raw = self._serial.readline()
                    except Exception:
                        break
                    if not raw:
                        continue
                    try:
                        line = raw.decode('ascii', errors='replace').strip()
                    except Exception:
                        continue
                    self.get_logger().debug(f'cal-wait: {line}')
                    if line.startswith('$CAL,'):
                        self.get_logger().info(f'Arduino: {line}')
                        if 'done' in line:
                            self._parse_cal_offsets(line)
                            cal_ok = True
                            self._calibrated = True
                            self._publish_calibrated(True)
                            self.get_logger().info(
                                'Gyro calibration complete — IMU data is now valid'
                            )
                            break
                if not cal_ok:
                    self.get_logger().warn(
                        'Did not see $CAL,done within timeout — '
                        'will keep looking in normal reader loop'
                    )
                self._yaw_deg = 0.0
                self._last_imu_time = None
                need_calibration = False

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

            if line.startswith('$IMU,'):
                self._parse_imu(line)
            elif line.startswith('$BNO,'):
                self._parse_bno(line)
            elif line.startswith('$GRV,'):
                self._parse_gravity(line)
            elif line.startswith('$AUD,'):
                self._parse_audio(line)
            elif line.startswith('$CAL,'):
                self.get_logger().info(f'Arduino: {line}')
                if 'done' in line:
                    self._parse_cal_offsets(line)
                    self._yaw_deg = 0.0
                    self._calibrated = True
                    self._publish_calibrated(True)
                    self.get_logger().info('Gyro calibration complete — IMU data is now valid')
            elif line.startswith('$BNOCAL,'):
                self.get_logger().info(f'Arduino: {line}')
            elif line.startswith('$INFO,'):
                self.get_logger().info(f'Arduino: {line}')
                # Detect BNO055 availability from $INFO line
                if 'bno055=yes' in line:
                    if not self._bno_available:
                        self.get_logger().info(
                            'BNO055 detected — imu/data will use fused quaternion orientation'
                        )
                    self._bno_available = True
                elif 'bno055=no' in line:
                    self._bno_available = False
                    self.get_logger().info(
                        'BNO055 not detected — using LSM6DSOX with gyro-only yaw'
                    )
            elif line.startswith('$ERR,') or line.startswith('$MSG,') or line.startswith('$MODE,'):
                self.get_logger().info(f'Arduino: {line}')

    def _parse_cal_offsets(self, line: str) -> None:
        """Extract accel offsets from $CAL,done,<ng>,<gx>,<gy>,<gz>,<ax>,<ay>,<az>."""
        parts = line.split(',')
        # New firmware: 8+ fields; old firmware: 5 fields (no accel offsets)
        if len(parts) >= 8:
            try:
                abx = float(parts[5])
                aby = float(parts[6])
                abz = float(parts[7])
                self._accel_bias = [abx, aby, abz]
                self.get_logger().info(
                    f'Accel bias from calibration: '
                    f'X={abx:+.4f}g  Y={aby:+.4f}g  Z={abz:+.4f}g'
                )
            except (ValueError, IndexError):
                self.get_logger().warn('Could not parse accel offsets from $CAL line')
        else:
            self.get_logger().info('Firmware did not report accel offsets (old format)')

    def _parse_imu(self, line: str) -> None:
        """Parse $IMU line — on-board LSM6DSOX data.

        When BNO055 is active, $BNO handles imu/data and imu/yaw_deg.
        $IMU still provides mic level (always) and serves as fallback
        for imu/data + imu/yaw_deg when no BNO055 is present.
        """
        # $IMU,ax,ay,az,gx,gy,gz,temp,mic,ms
        parts = line.split(',')
        if len(parts) != 10:
            return
        try:
            raw_ax = float(parts[1])
            raw_ay = float(parts[2])
            raw_az = float(parts[3])
            raw_gx = float(parts[4])
            raw_gy = float(parts[5])
            raw_gz = float(parts[6])
            temp = float(parts[7])
            mic = int(parts[8])
            _ms = int(parts[9])
        except (ValueError, IndexError):
            return

        # ── Always publish mic level (only source) ────────────────────
        mic_msg = Int32()
        mic_msg.data = mic
        self._mic_pub.publish(mic_msg)

        # ── If BNO055 is active, skip LSM6DSOX-based imu/data + yaw ──
        # The $BNO handler publishes superior fused orientation.
        # We still publish temperature from $IMU when BNO055 is providing
        # its own temp (but $IMU temp is the LSM6DSOX die — different).
        if self._bno_available:
            return

        # ── Fallback: LSM6DSOX-only mode (no BNO055) ─────────────────
        accel_raw = [raw_ax, raw_ay, raw_az]
        gyro_raw = [raw_gx, raw_gy, raw_gz]

        ax = self._accel_sign[0] * accel_raw[self._accel_map[0]] * G_MPS2
        ay = self._accel_sign[1] * accel_raw[self._accel_map[1]] * G_MPS2
        az = self._accel_sign[2] * accel_raw[self._accel_map[2]] * G_MPS2

        gx = self._gyro_sign[0] * gyro_raw[self._gyro_map[0]] * DEG_TO_RAD
        gy = self._gyro_sign[1] * gyro_raw[self._gyro_map[1]] * DEG_TO_RAD
        gz = self._gyro_sign[2] * gyro_raw[self._gyro_map[2]] * DEG_TO_RAD

        now = time.monotonic()

        # ── Yaw integration (gyro-only, drifts) ──────────────────────
        if self._enable_yaw:
            yaw_rate_dps = self._yaw_sign * gyro_raw[self._yaw_axis]
            if self._last_imu_time is not None:
                dt = now - self._last_imu_time
                if 0.0 < dt < 0.5:
                    self._yaw_deg += yaw_rate_dps * dt
                    while self._yaw_deg > 180.0:
                        self._yaw_deg -= 360.0
                    while self._yaw_deg < -180.0:
                        self._yaw_deg += 360.0
        self._last_imu_time = now

        # ── Publish sensor_msgs/Imu (no orientation) ──────────────────
        stamp = self.get_clock().now().to_msg()

        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self._frame_id

        # Orientation: not measured; set covariance[0] = -1 to indicate unknown
        imu_msg.orientation_covariance[0] = -1.0

        imu_msg.angular_velocity.x = gx
        imu_msg.angular_velocity.y = gy
        imu_msg.angular_velocity.z = gz
        imu_msg.angular_velocity_covariance = self._gyro_cov[:]

        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az
        imu_msg.linear_acceleration_covariance = self._accel_cov[:]

        self._imu_pub.publish(imu_msg)

        # ── Publish calibrated status ─────────────────────────────────
        self._publish_calibrated(self._calibrated)

        # ── Publish yaw (gyro-only fallback) ──────────────────────────
        if self._enable_yaw:
            yaw_msg = Float64()
            yaw_msg.data = self._yaw_deg
            self._yaw_pub.publish(yaw_msg)

        # ── Publish temperature ───────────────────────────────────────
        temp_msg = Float32()
        temp_msg.data = temp
        self._temp_pub.publish(temp_msg)

    def _parse_bno(self, line: str) -> None:
        """Parse $BNO line — fused 9-DOF data from BNO055.

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
            gx = float(parts[8])   # gyro, rad/s
            gy = float(parts[9])
            gz = float(parts[10])
            mx = float(parts[11])  # magnetometer, µT
            my = float(parts[12])
            mz = float(parts[13])
            heading = float(parts[14])  # Euler heading 0-360
            roll = float(parts[15])
            pitch = float(parts[16])
            temp = float(parts[17])
            cal_sys = int(parts[18])
            cal_gyro = int(parts[19])
            cal_accel = int(parts[20])
            cal_mag = int(parts[21])
            _ms = int(parts[22])
        except (ValueError, IndexError):
            return

        self._bno_available = True
        stamp = self.get_clock().now().to_msg()

        # ── Track BNO055 calibration (informational only) ─────────────
        # imu/calibrated is driven by the LSM6DSOX gyro-cal handshake
        # ($CAL,done) so that heading-hold and tilt compensation stay
        # active.  BNO055 fusion is usable even at sys=0; accuracy
        # simply improves as the sensor auto-calibrates over time.
        bno_cal_ok = cal_sys >= self._bno_cal_threshold
        prev_bno_cal = getattr(self, '_bno_cal_ok', None)
        if bno_cal_ok != prev_bno_cal:
            self._bno_cal_ok = bno_cal_ok
            self.get_logger().info(
                f'BNO055 cal: sys={cal_sys} gyro={cal_gyro} '
                f'accel={cal_accel} mag={cal_mag} → '
                f'{"CALIBRATED" if bno_cal_ok else "NOT calibrated"}'
            )
        self._bno_cal = {
            'sys': cal_sys, 'gyro': cal_gyro,
            'accel': cal_accel, 'mag': cal_mag,
        }

        # ── Publish sensor_msgs/Imu with REAL orientation ─────────────
        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self._frame_id

        imu_msg.orientation.w = qw
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation_covariance = self._orient_cov[:]

        imu_msg.angular_velocity.x = gx
        imu_msg.angular_velocity.y = gy
        imu_msg.angular_velocity.z = gz
        imu_msg.angular_velocity_covariance = self._gyro_cov[:]

        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az
        imu_msg.linear_acceleration_covariance = self._accel_cov[:]

        self._imu_pub.publish(imu_msg)

        # ── Publish yaw (magnetometer-stabilised heading) ─────────────
        yaw = heading + self._heading_offset
        while yaw > 180.0:
            yaw -= 360.0
        while yaw < -180.0:
            yaw += 360.0
        self._yaw_deg = yaw  # keep state in sync for any readers

        yaw_msg = Float64()
        yaw_msg.data = yaw
        self._yaw_pub.publish(yaw_msg)

        # ── Publish calibrated status ─────────────────────────────────
        self._publish_calibrated(self._calibrated)

        # ── Publish detailed calibration JSON (~4 Hz) ─────────────────
        self._bno_msg_counter += 1
        if self._bno_msg_counter >= 3:
            self._bno_msg_counter = 0
            cal_str = String()
            cal_str.data = json.dumps(self._bno_cal)
            self._cal_detail_pub.publish(cal_str)

        # ── Publish temperature ───────────────────────────────────────
        temp_msg = Float32()
        temp_msg.data = temp
        self._temp_pub.publish(temp_msg)

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
                # Turn LED off on shutdown
                self._serial.write(b'O')
                self._serial.close()
            except Exception:
                pass
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = ImuSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
