"""
Odometry node for Raspbot V2 mecanum drive.

Dead-reckons robot pose by integrating cmd_vel (vx, vy, wz) with IMU yaw.
Publishes:
  - nav_msgs/Odometry on /odom
  - TF: odom → base_link
  - nav_msgs/Path on /odom/path        (recorded waypoints)
  - std_msgs/Bool on /odom/recording    (true while recording)
  - std_msgs/Bool on /odom/returning    (true while return-to-origin active)

Services:
  - /odom/set_origin        (Trigger)  reset pose to (0,0,0)
  - /odom/start_recording   (Trigger)  begin recording waypoints
  - /odom/stop_recording    (Trigger)  stop recording waypoints
  - /odom/return_to_origin  (Trigger)  replay recorded path in reverse
  - /odom/cancel_return     (Trigger)  abort return-to-origin
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import (
    Twist,
    PoseStamped,
    TransformStamped,
    Quaternion,
)
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float64
from std_srvs.srv import Trigger

import tf2_ros

# ── helpers ──────────────────────────────────────────────────────────

DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def yaw_from_quaternion(q) -> float:
    """Extract yaw (rad) from a geometry_msgs/Quaternion."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    return q


def normalize_angle(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


# ── Odometry Node ───────────────────────────────────────────────────

class OdometryNode(Node):
    """Dead-reckoning odometry with path recording and return-to-origin."""

    # Parameters that can be changed at runtime via ros2 param set
    _TUNABLE_PARAMS = {
        'odom_publish_hz': '_publish_hz',
        'odom_use_imu_yaw': '_use_imu_yaw',
        'odom_record_interval_m': '_record_interval_m',
        'odom_record_interval_rad': '_record_interval_rad',
        'return_linear_speed': '_return_linear_speed',
        'return_angular_speed': '_return_angular_speed',
        'return_goal_tolerance_m': '_return_goal_tolerance_m',
        'return_goal_tolerance_rad': '_return_goal_tolerance_rad',
        'return_timeout_sec': '_return_timeout_sec',
        'odom_vx_scale': '_vx_scale',
        'odom_vy_scale': '_vy_scale',
        'odom_zupt_accel_threshold': '_zupt_accel_threshold',
        'odom_zupt_gyro_threshold': '_zupt_gyro_threshold',
    }

    def __init__(self):
        super().__init__('odometry')

        # ── Declare parameters ────────────────────────────────────────
        self.declare_parameter('odom_publish_hz', 20.0)
        self.declare_parameter('odom_use_imu_yaw', True)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('odom_child_frame_id', 'base_link')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('imu_topic', 'imu/data')
        self.declare_parameter('imu_yaw_topic', 'imu/yaw_deg')

        # Path recording: capture a waypoint when distance OR angle change exceeds
        self.declare_parameter('odom_record_interval_m', 0.05)
        self.declare_parameter('odom_record_interval_rad', 0.10)  # ~5.7 deg
        self.declare_parameter('odom_max_waypoints', 5000)

        # Return-to-origin controller
        self.declare_parameter('return_linear_speed', 0.15)   # m/s
        self.declare_parameter('return_angular_speed', 0.6)    # rad/s
        self.declare_parameter('return_goal_tolerance_m', 0.08)
        self.declare_parameter('return_goal_tolerance_rad', 0.15)  # ~8.6 deg
        self.declare_parameter('return_timeout_sec', 120.0)
        self.declare_parameter('return_cmd_vel_topic', 'cmd_vel')

        # Velocity scale factors (calibrate for wheel slip/gear ratio)
        self.declare_parameter('odom_vx_scale', 1.0)   # forward velocity multiplier
        self.declare_parameter('odom_vy_scale', 1.0)    # lateral velocity multiplier

        # IMU-based Zero-Velocity Update (ZUPT)
        self.declare_parameter('odom_zupt_accel_threshold', 0.15)  # m/s² — XY accel below this ≈ stationary
        self.declare_parameter('odom_zupt_gyro_threshold', 0.05)   # rad/s — gyro below this ≈ not rotating

        # Live path publish rate during recording
        self.declare_parameter('odom_path_publish_hz', 2.0)  # how often to publish recorded path

        # ── Load parameters ───────────────────────────────────────────
        self._publish_hz = self.get_parameter('odom_publish_hz').value
        self._use_imu_yaw = self.get_parameter('odom_use_imu_yaw').value
        self._odom_frame = self.get_parameter('odom_frame_id').value
        self._child_frame = self.get_parameter('odom_child_frame_id').value
        self._record_interval_m = self.get_parameter('odom_record_interval_m').value
        self._record_interval_rad = self.get_parameter('odom_record_interval_rad').value
        self._max_waypoints = int(self.get_parameter('odom_max_waypoints').value)

        self._return_linear_speed = self.get_parameter('return_linear_speed').value
        self._return_angular_speed = self.get_parameter('return_angular_speed').value
        self._return_goal_tolerance_m = self.get_parameter('return_goal_tolerance_m').value
        self._return_goal_tolerance_rad = self.get_parameter('return_goal_tolerance_rad').value
        self._return_timeout_sec = self.get_parameter('return_timeout_sec').value

        self._vx_scale = self.get_parameter('odom_vx_scale').value
        self._vy_scale = self.get_parameter('odom_vy_scale').value
        self._zupt_accel_threshold = self.get_parameter('odom_zupt_accel_threshold').value
        self._zupt_gyro_threshold = self.get_parameter('odom_zupt_gyro_threshold').value
        self._path_publish_hz = self.get_parameter('odom_path_publish_hz').value

        # ── Pose state ────────────────────────────────────────────────
        self._x = 0.0  # metres in odom frame
        self._y = 0.0
        self._yaw = 0.0  # radians
        self._vx = 0.0  # body-frame velocities (from cmd_vel)
        self._vy = 0.0
        self._wz = 0.0
        self._last_update_time: float | None = None

        # IMU yaw tracking
        self._imu_yaw: float | None = None      # latest IMU fused yaw (rad)
        self._imu_yaw_offset: float | None = None  # offset between odom yaw and IMU yaw
        self._imu_yaw_deg: float | None = None   # from imu/yaw_deg topic

        # IMU acceleration & gyro for ZUPT and accuracy improvements
        self._imu_accel_x = 0.0  # body-frame linear acceleration (gravity removed by BNO055)
        self._imu_accel_y = 0.0
        self._imu_gyro_z = 0.0   # angular velocity from gyro
        self._imu_accel_ema = 0.0  # exponential moving average of XY accel magnitude
        self._imu_gyro_ema = 0.0   # EMA of abs(gyro_z)
        self._zupt_active = False  # True when zero-velocity update is applied
        self._last_path_publish = 0.0  # monotonic time of last path publish
        self._last_status_publish = 0.0  # monotonic time of last recording/returning publish

        # Path recording state
        self._recording = False
        self._waypoints: list[tuple[float, float, float]] = []  # (x, y, yaw)
        self._last_wp_x = 0.0
        self._last_wp_y = 0.0
        self._last_wp_yaw = 0.0

        # Return-to-origin state
        self._returning = False
        self._return_waypoints: list[tuple[float, float, float]] = []
        self._return_wp_idx = 0
        self._return_start_time = 0.0
        self._lock = threading.Lock()

        # ── Publishers ────────────────────────────────────────────────
        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self._path_pub = self.create_publisher(Path, 'odom/path', 10)
        self._recording_pub = self.create_publisher(Bool, 'odom/recording', 10)
        self._returning_pub = self.create_publisher(Bool, 'odom/returning', 10)

        # TF broadcaster
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Return-to-origin publishes Twist
        return_topic = self.get_parameter('return_cmd_vel_topic').value
        self._return_cmd_pub = self.create_publisher(Twist, return_topic, 10)

        # ── Subscribers ───────────────────────────────────────────────
        cmd_topic = self.get_parameter('cmd_vel_topic').value
        self.create_subscription(Twist, cmd_topic, self._on_cmd_vel, 10)

        imu_topic = self.get_parameter('imu_topic').value
        self.create_subscription(Imu, imu_topic, self._on_imu, 10)

        imu_yaw_topic = self.get_parameter('imu_yaw_topic').value
        self.create_subscription(Float64, imu_yaw_topic, self._on_imu_yaw, 10)

        # ── Services ─────────────────────────────────────────────────
        self.create_service(Trigger, 'odom/set_origin', self._srv_set_origin)
        self.create_service(Trigger, 'odom/start_recording', self._srv_start_recording)
        self.create_service(Trigger, 'odom/stop_recording', self._srv_stop_recording)
        self.create_service(Trigger, 'odom/return_to_origin', self._srv_return_to_origin)
        self.create_service(Trigger, 'odom/cancel_return', self._srv_cancel_return)

        # ── Timer ─────────────────────────────────────────────────────
        period = 1.0 / max(self._publish_hz, 1.0)
        self._timer = self.create_timer(period, self._tick)

        # Live-tuning callback
        self.add_on_set_parameters_callback(self._on_param_change)

        self.get_logger().info(
            f'Odometry node started — '
            f'frame: {self._odom_frame}→{self._child_frame}, '
            f'hz: {self._publish_hz}, imu_yaw: {self._use_imu_yaw}, '
            f'vx_scale: {self._vx_scale}, vy_scale: {self._vy_scale}, '
            f'ZUPT accel<{self._zupt_accel_threshold} gyro<{self._zupt_gyro_threshold}'
        )

    # ── Parameter change callback ─────────────────────────────────────

    def _on_param_change(self, params) -> SetParametersResult:
        for p in params:
            attr = self._TUNABLE_PARAMS.get(p.name)
            if attr:
                setattr(self, attr, p.value)
                self.get_logger().info(f'[odom] param {p.name} → {p.value}')
        return SetParametersResult(successful=True)

    # ── Subscriber callbacks ──────────────────────────────────────────

    def _on_cmd_vel(self, msg: Twist) -> None:
        """Latch last cmd_vel for integration."""
        self._vx = float(msg.linear.x)
        self._vy = float(getattr(msg.linear, 'y', 0.0))
        self._wz = float(msg.angular.z)

    def _on_imu(self, msg: Imu) -> None:
        """Extract fused yaw, linear acceleration, and angular velocity from IMU."""
        q = msg.orientation
        # Ignore zero quaternion (sensor not ready)
        if q.w == 0.0 and q.x == 0.0 and q.y == 0.0 and q.z == 0.0:
            return
        self._imu_yaw = yaw_from_quaternion(q)

        # Linear acceleration (BNO055 provides gravity-compensated in NDOF mode)
        self._imu_accel_x = float(msg.linear_acceleration.x)
        self._imu_accel_y = float(msg.linear_acceleration.y)

        # Angular velocity (gyroscope)
        self._imu_gyro_z = float(msg.angular_velocity.z)

        # Update EMAs for smoother ZUPT detection (alpha ≈ 0.15 at ~50Hz)
        accel_mag = math.sqrt(self._imu_accel_x ** 2 + self._imu_accel_y ** 2)
        gyro_mag = abs(self._imu_gyro_z)
        alpha = 0.15
        self._imu_accel_ema = alpha * accel_mag + (1.0 - alpha) * self._imu_accel_ema
        self._imu_gyro_ema = alpha * gyro_mag + (1.0 - alpha) * self._imu_gyro_ema

    def _on_imu_yaw(self, msg: Float64) -> None:
        """Track imu/yaw_deg for possible future use."""
        self._imu_yaw_deg = float(msg.data)

    # ── Main tick ─────────────────────────────────────────────────────

    def _tick(self) -> None:
        now = time.time()
        if self._last_update_time is None:
            self._last_update_time = now
            return
        dt = now - self._last_update_time
        self._last_update_time = now
        if dt <= 0.0 or dt > 1.0:
            return  # skip huge jumps

        try:
            self._tick_inner(dt, now)
        except Exception as e:
            self.get_logger().error(f'[odom] tick error: {e}')
            # Safety: if we crash during return, stop motors and abort
            if self._returning:
                self._returning = False
                self._return_cmd_pub.publish(Twist())
                b = Bool()
                b.data = False
                self._returning_pub.publish(b)
                self.get_logger().warn('[odom] return aborted due to error')

    def _tick_inner(self, dt: float, now: float) -> None:

        # ── Integrate pose ────────────────────────────────────────────
        self._integrate(dt)

        # ── Return-to-origin controller ───────────────────────────────
        if self._returning:
            self._return_tick(dt)

        # ── Publish odometry ──────────────────────────────────────────
        self._publish_odom()

        # ── Periodic status publishing (~2 Hz) ────────────────────────
        # Continuously publish recording/returning state so subscribers
        # never miss a one-shot transition message.
        if (now - self._last_status_publish) >= 0.5:
            self._last_status_publish = now
            b_rec = Bool()
            b_rec.data = self._recording
            self._recording_pub.publish(b_rec)
            b_ret = Bool()
            b_ret.data = self._returning
            self._returning_pub.publish(b_ret)
            # Keep the path visible even when not recording
            if self._waypoints:
                self._publish_path()

        # ── Record waypoints if active ────────────────────────────────
        if self._recording:
            self._maybe_record_waypoint()
            # Periodically publish the in-progress path for live visualisation
            if self._path_publish_hz > 0:
                interval = 1.0 / self._path_publish_hz
                if (now - self._last_path_publish) >= interval:
                    self._publish_path()
                    self._last_path_publish = now

    def _integrate(self, dt: float) -> None:
        """Dead-reckon position from cmd_vel + IMU yaw, with ZUPT & velocity scaling."""
        if self._use_imu_yaw and self._imu_yaw is not None:
            # Use fused IMU yaw for heading, anchored to odom frame
            if self._imu_yaw_offset is None:
                # First IMU reading: calibrate offset so odom yaw stays continuous
                self._imu_yaw_offset = self._yaw - self._imu_yaw
            self._yaw = self._imu_yaw + self._imu_yaw_offset
        else:
            # Fallback: integrate gyro wz from cmd_vel
            self._yaw += self._wz * dt

        self._yaw = normalize_angle(self._yaw)

        # Apply velocity scale factors (calibrate for wheel diameter / slip)
        vx = self._vx * self._vx_scale
        vy = self._vy * self._vy_scale

        # ── ZUPT (Zero-Velocity Update) ───────────────────────────────
        # When IMU indicates the robot is physically stationary (low accel + low gyro),
        # suppress any residual cmd_vel drift to prevent position creep.
        if (self._imu_accel_ema < self._zupt_accel_threshold and
                self._imu_gyro_ema < self._zupt_gyro_threshold):
            # Only apply ZUPT when cmd_vel is also small (avoid blocking intentional moves
            # that haven't physically started yet due to motor latency)
            if abs(vx) < 0.01 and abs(vy) < 0.01:
                vx = 0.0
                vy = 0.0
                self._zupt_active = True
            else:
                self._zupt_active = False
        else:
            self._zupt_active = False

        # Transform body velocities to odom frame
        cos_yaw = math.cos(self._yaw)
        sin_yaw = math.sin(self._yaw)
        dx = (vx * cos_yaw - vy * sin_yaw) * dt
        dy = (vx * sin_yaw + vy * cos_yaw) * dt

        self._x += dx
        self._y += dy

    def _publish_odom(self) -> None:
        """Publish Odometry message and TF."""
        stamp = self.get_clock().now().to_msg()
        q = quaternion_from_yaw(self._yaw)

        # nav_msgs/Odometry
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._child_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q
        # Body-frame velocity
        odom.twist.twist.linear.x = self._vx
        odom.twist.twist.linear.y = self._vy
        odom.twist.twist.angular.z = self._wz
        self._odom_pub.publish(odom)

        # TF: odom → base_link
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self._odom_frame
        t.child_frame_id = self._child_frame
        t.transform.translation.x = self._x
        t.transform.translation.y = self._y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self._tf_broadcaster.sendTransform(t)

    # ── Path recording ────────────────────────────────────────────────

    def _maybe_record_waypoint(self) -> None:
        dx = self._x - self._last_wp_x
        dy = self._y - self._last_wp_y
        dist = math.sqrt(dx * dx + dy * dy)
        dyaw = abs(normalize_angle(self._yaw - self._last_wp_yaw))

        if dist >= self._record_interval_m or dyaw >= self._record_interval_rad:
            if len(self._waypoints) < self._max_waypoints:
                self._waypoints.append((self._x, self._y, self._yaw))
                self._last_wp_x = self._x
                self._last_wp_y = self._y
                self._last_wp_yaw = self._yaw
            else:
                self.get_logger().warn(
                    f'[odom] max waypoints ({self._max_waypoints}) reached, '
                    f'not recording more'
                )

    def _publish_path(self) -> None:
        """Publish the recorded waypoints as a nav_msgs/Path for visualisation."""
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self._odom_frame
        for (wx, wy, wyaw) in self._waypoints:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.orientation = quaternion_from_yaw(wyaw)
            path.poses.append(ps)
        self._path_pub.publish(path)

    # ── Return-to-origin controller ───────────────────────────────────

    def _return_tick(self, dt: float) -> None:
        """Simple waypoint-following controller for the return leg."""
        if not self._return_waypoints:
            self._finish_return('arrived at origin (no waypoints)')
            return

        # Timeout
        if (time.time() - self._return_start_time) > self._return_timeout_sec:
            self._finish_return('timeout')
            return

        # If we've passed all waypoints, do final yaw alignment
        if self._return_wp_idx >= len(self._return_waypoints):
            self._return_align_to_origin()
            return

        # Current target waypoint
        tx, ty, tyaw = self._return_waypoints[self._return_wp_idx]
        dx = tx - self._x
        dy = ty - self._y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < self._return_goal_tolerance_m:
            # Reached this waypoint, advance
            self._return_wp_idx += 1
            if self._return_wp_idx >= len(self._return_waypoints):
                # Final waypoint reached — do a final yaw alignment to origin heading
                self._return_align_to_origin()
                return
            return  # will process next waypoint on next tick

        # ── Navigate towards waypoint ─────────────────────────────────
        # Desired heading to waypoint
        desired_yaw = math.atan2(dy, dx)
        yaw_err = normalize_angle(desired_yaw - self._yaw)

        cmd = Twist()
        if abs(yaw_err) > 0.3:  # ~17 deg — rotate-in-place first
            cmd.angular.z = clamp(
                self._return_angular_speed * (1.0 if yaw_err > 0 else -1.0),
                -self._return_angular_speed,
                self._return_angular_speed,
            )
        else:
            # Drive forward with proportional yaw correction
            cmd.linear.x = self._return_linear_speed
            cmd.angular.z = clamp(
                yaw_err * 2.0,  # proportional steering gain
                -self._return_angular_speed,
                self._return_angular_speed,
            )
        self._return_cmd_pub.publish(cmd)

    def _return_align_to_origin(self) -> None:
        """Rotate in place to face origin heading (yaw=0)."""
        yaw_err = normalize_angle(0.0 - self._yaw)
        if abs(yaw_err) < self._return_goal_tolerance_rad:
            self._finish_return('arrived at origin (aligned)')
            return
        cmd = Twist()
        cmd.angular.z = clamp(
            self._return_angular_speed * (1.0 if yaw_err > 0 else -1.0),
            -self._return_angular_speed,
            self._return_angular_speed,
        )
        self._return_cmd_pub.publish(cmd)

    def _finish_return(self, reason: str) -> None:
        self._returning = False
        # Send zero velocity to stop
        self._return_cmd_pub.publish(Twist())
        b = Bool()
        b.data = False
        self._returning_pub.publish(b)
        self.get_logger().info(f'[odom] return-to-origin finished: {reason}')

    # ── Service handlers ──────────────────────────────────────────────

    def _srv_set_origin(self, req, resp):
        """Reset odometry pose to (0, 0, 0)."""
        with self._lock:
            self._x = 0.0
            self._y = 0.0
            self._yaw = 0.0
            self._imu_yaw_offset = None  # re-calibrate on next IMU reading
            self._last_update_time = None
            self._waypoints.clear()
            self._last_wp_x = 0.0
            self._last_wp_y = 0.0
            self._last_wp_yaw = 0.0
            if self._returning:
                self._returning = False
                self._return_cmd_pub.publish(Twist())
        resp.success = True
        resp.message = 'Origin set to current position'
        self.get_logger().info('[odom] origin reset to (0, 0, 0)')
        b = Bool()
        b.data = False
        self._returning_pub.publish(b)
        return resp

    def _srv_start_recording(self, req, resp):
        """Begin recording waypoints."""
        with self._lock:
            if self._recording:
                resp.success = False
                resp.message = 'Already recording'
                return resp
            self._recording = True
            self._waypoints.clear()
            self._last_wp_x = self._x
            self._last_wp_y = self._y
            self._last_wp_yaw = self._yaw
            # Record the starting position as first waypoint
            self._waypoints.append((self._x, self._y, self._yaw))
        resp.success = True
        resp.message = f'Recording started at ({self._x:.2f}, {self._y:.2f})'
        self.get_logger().info(f'[odom] recording started — origin ({self._x:.2f}, {self._y:.2f}, {math.degrees(self._yaw):.1f}°)')
        b = Bool()
        b.data = True
        self._recording_pub.publish(b)
        return resp

    def _srv_stop_recording(self, req, resp):
        """Stop recording waypoints."""
        with self._lock:
            if not self._recording:
                resp.success = False
                resp.message = 'Not recording'
                return resp
            self._recording = False
            # Record final position
            self._waypoints.append((self._x, self._y, self._yaw))
        n = len(self._waypoints)
        resp.success = True
        resp.message = f'Recording stopped — {n} waypoints captured'
        self.get_logger().info(f'[odom] recording stopped — {n} waypoints')
        b = Bool()
        b.data = False
        self._recording_pub.publish(b)
        # Publish the path for visualisation
        self._publish_path()
        return resp

    def _srv_return_to_origin(self, req, resp):
        """Navigate back to origin following recorded path in reverse."""
        with self._lock:
            if self._returning:
                resp.success = False
                resp.message = 'Already returning to origin'
                return resp
            if self._recording:
                # Auto-stop recording
                self._recording = False
                self._waypoints.append((self._x, self._y, self._yaw))
                b = Bool()
                b.data = False
                self._recording_pub.publish(b)
                self.get_logger().info('[odom] auto-stopped recording for return')
                # Publish the path so it stays visible during the return
                self._publish_path()

            if not self._waypoints:
                resp.success = False
                resp.message = 'No waypoints recorded — record a path first'
                return resp

            # Reverse the waypoints so we traverse them from current position back to start
            self._return_waypoints = list(reversed(self._waypoints))
            self._return_wp_idx = 0
            self._return_start_time = time.time()
            self._returning = True

        n = len(self._return_waypoints)
        resp.success = True
        resp.message = f'Returning to origin — {n} waypoints to traverse'
        self.get_logger().info(
            f'[odom] return-to-origin started — {n} waypoints, '
            f'timeout {self._return_timeout_sec}s'
        )
        b = Bool()
        b.data = True
        self._returning_pub.publish(b)
        return resp

    def _srv_cancel_return(self, req, resp):
        """Abort return-to-origin."""
        with self._lock:
            if not self._returning:
                resp.success = False
                resp.message = 'Not currently returning'
                return resp
            self._returning = False
            self._return_cmd_pub.publish(Twist())
        resp.success = True
        resp.message = 'Return-to-origin cancelled'
        self.get_logger().info('[odom] return-to-origin cancelled by user')
        b = Bool()
        b.data = False
        self._returning_pub.publish(b)
        return resp


# ── Entry point ───────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
