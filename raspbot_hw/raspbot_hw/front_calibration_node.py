"""
Front-direction auto-calibration for Raspbot V2.

Determines the angular offset of the LiDAR relative to the robot's
mechanical forward direction using multiple cross-sensor correlation
methods.  Also verifies motor direction and IMU yaw sign.

Calibration methods:
  1. **Static (no motion)** — cross-correlate ultrasonic range with LiDAR
     scan angles.  The ultrasonic sensor is physically fixed pointing forward;
     the LiDAR angle that best matches it reveals the angular offset.
  2. **Motion (drive forward)** — send a brief forward cmd_vel, observe which
     LiDAR angles show decreasing range.  That direction is "forward".
  3. **IMU yaw check** — send a brief CCW rotation, verify IMU yaw increases.

Results are published on ``calibration/result`` (String, JSON) and applied
by setting ``lidar_yaw_offset`` on ``lidar_obstacle`` and ``lidar_slam``
nodes via ``ros2 param set``.

Services:
  calibrate_front          (Trigger)  — full calibration (static + motion + IMU)
  calibrate_front_static   (Trigger)  — static only (no motion, safe)

Subscribes:
  /scan            (LaserScan)
  ultrasonic/range (Range)
  imu/yaw_deg      (Float64)

Publishes:
  cmd_vel              (Twist)         — brief pulses during motion calibration
  calibration/result   (String, JSON)  — calibration outcome
  calibration/active   (Bool)          — true while calibrating
"""

import math
import time
import threading
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, Range
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger

# ── helpers ──────────────────────────────────────────────────────────

DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi


def normalize_angle(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


# ── Calibration Node ─────────────────────────────────────────────────

class FrontCalibrationNode(Node):
    """Auto-calibrate the robot's front-direction using sensor fusion."""

    def __init__(self):
        super().__init__('front_calibration')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('ultrasonic_topic', 'ultrasonic/range')
        self.declare_parameter('imu_yaw_topic', 'imu/yaw_deg')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')

        # Static calibration
        self.declare_parameter('static_samples', 15)        # scan + US samples
        self.declare_parameter('static_sample_interval', 0.15)  # seconds
        self.declare_parameter('us_beam_half_deg', 15.0)     # ultrasonic half-cone
        self.declare_parameter('us_match_tolerance_m', 0.20) # range match tolerance
        self.declare_parameter('us_max_range', 3.0)          # ignore US > this

        # Motion calibration
        self.declare_parameter('drive_speed', 0.10)          # m/s
        self.declare_parameter('drive_duration', 0.6)        # seconds
        self.declare_parameter('drive_settle', 0.5)          # wait after stop
        self.declare_parameter('drive_scan_samples', 8)      # scans to average
        self.declare_parameter('angle_smooth_window_deg', 20.0)  # smoothing window

        # IMU yaw check
        self.declare_parameter('rotate_speed', 0.3)          # rad/s
        self.declare_parameter('rotate_duration', 1.0)       # seconds
        self.declare_parameter('rotate_settle', 0.5)

        # Auto-apply
        self.declare_parameter('auto_apply', True)           # set params on nodes
        self.declare_parameter('lidar_obstacle_node', 'lidar_obstacle')
        self.declare_parameter('lidar_slam_node', 'lidar_slam')

        # ── Load parameters ───────────────────────────────────────────
        scan_topic = str(self.get_parameter('scan_topic').value)
        us_topic = str(self.get_parameter('ultrasonic_topic').value)
        imu_topic = str(self.get_parameter('imu_yaw_topic').value)
        cmd_topic = str(self.get_parameter('cmd_vel_topic').value)

        # ── State ─────────────────────────────────────────────────────
        self._latest_scan = None       # most recent LaserScan
        self._latest_us = None         # most recent ultrasonic range (m)
        self._latest_yaw = None        # most recent IMU yaw (degrees)
        self._calibrating = False
        self._lock = threading.Lock()

        # ── Subscribers ───────────────────────────────────────────────
        self._scan_sub = self.create_subscription(
            LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data)
        self._us_sub = self.create_subscription(
            Range, us_topic, self._on_us, 10)
        self._yaw_sub = self.create_subscription(
            Float64, imu_topic, self._on_yaw, 10)

        # ── Publishers ────────────────────────────────────────────────
        self._cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self._result_pub = self.create_publisher(String, 'calibration/result', 10)
        self._active_pub = self.create_publisher(Bool, 'calibration/active', 10)

        # ── Services ──────────────────────────────────────────────────
        self.create_service(
            Trigger, 'calibrate_front', self._srv_full_calibrate)
        self.create_service(
            Trigger, 'calibrate_front_static', self._srv_static_calibrate)

        # ── Heartbeat (publish active=false) ──────────────────────────
        self._heartbeat_timer = self.create_timer(1.0, self._publish_active)

        self.get_logger().info(
            'Front calibration node ready — call calibrate_front or calibrate_front_static')

    # ── Callbacks ─────────────────────────────────────────────────────

    def _on_scan(self, msg: LaserScan) -> None:
        self._latest_scan = msg

    def _on_us(self, msg: Range) -> None:
        r = msg.range
        if not math.isinf(r) and not math.isnan(r) and r > 0:
            self._latest_us = r

    def _on_yaw(self, msg: Float64) -> None:
        self._latest_yaw = msg.data

    def _publish_active(self) -> None:
        b = Bool()
        b.data = self._calibrating
        self._active_pub.publish(b)

    # ── Service handlers ──────────────────────────────────────────────

    def _srv_full_calibrate(self, request, response):
        """Full calibration: static + motion + IMU check."""
        if self._calibrating:
            response.success = False
            response.message = 'Calibration already in progress'
            return response

        self._calibrating = True
        self._publish_active()

        try:
            result = self._run_full_calibration()
            response.success = result.get('success', False)
            response.message = json.dumps(result)
        except Exception as e:
            response.success = False
            response.message = f'Error: {e}'
            self.get_logger().error(f'Calibration error: {e}')
        finally:
            self._stop_robot()
            self._calibrating = False
            self._publish_active()

        return response

    def _srv_static_calibrate(self, request, response):
        """Static-only calibration (no motion)."""
        if self._calibrating:
            response.success = False
            response.message = 'Calibration already in progress'
            return response

        self._calibrating = True
        self._publish_active()

        try:
            result = self._run_static_calibration()
            response.success = result.get('success', False)
            response.message = json.dumps(result)
        except Exception as e:
            response.success = False
            response.message = f'Error: {e}'
            self.get_logger().error(f'Calibration error: {e}')
        finally:
            self._calibrating = False
            self._publish_active()

        return response

    # ── Full calibration ──────────────────────────────────────────────

    def _run_full_calibration(self) -> dict:
        """Run all three calibration phases."""
        results = {}

        # Phase 1: static LiDAR-ultrasonic correlation
        self.get_logger().info('[Cal] Phase 1: Static LiDAR-ultrasonic cross-correlation')
        static_result = self._calibrate_static()
        results['static'] = static_result

        # Phase 2: motion-based forward detection
        self.get_logger().info('[Cal] Phase 2: Drive-and-scan forward detection')
        motion_result = self._calibrate_motion()
        results['motion'] = motion_result

        # Phase 3: IMU yaw sign check
        self.get_logger().info('[Cal] Phase 3: IMU yaw sign verification')
        imu_result = self._calibrate_imu_yaw()
        results['imu'] = imu_result

        # ── Determine best lidar_yaw_offset ───────────────────────────
        # Prefer motion-based (more reliable), fall back to static
        lidar_yaw_offset_deg = 0.0
        method_used = 'none'

        if motion_result.get('success'):
            lidar_yaw_offset_deg = motion_result['forward_angle_deg']
            method_used = 'motion'
        elif static_result.get('success'):
            lidar_yaw_offset_deg = static_result['offset_deg']
            method_used = 'static'

        results['lidar_yaw_offset_deg'] = round(lidar_yaw_offset_deg, 1)
        results['method_used'] = method_used
        results['imu_yaw_correct'] = imu_result.get('correct', None)
        results['success'] = method_used != 'none'

        # ── Apply corrections ─────────────────────────────────────────
        if results['success'] and bool(self.get_parameter('auto_apply').value):
            self._apply_offset(lidar_yaw_offset_deg)
            results['applied'] = True
            self.get_logger().info(
                f'[Cal] Applied lidar_yaw_offset = {lidar_yaw_offset_deg:.1f}° '
                f'(method: {method_used})')
        else:
            results['applied'] = False

        # Publish result
        msg = String()
        msg.data = json.dumps(results)
        self._result_pub.publish(msg)
        self.get_logger().info(f'[Cal] Result: {json.dumps(results, indent=2)}')

        return results

    def _run_static_calibration(self) -> dict:
        """Run static-only calibration."""
        static_result = self._calibrate_static()

        result = {
            'static': static_result,
            'lidar_yaw_offset_deg': 0.0,
            'method_used': 'none',
            'success': False,
        }

        if static_result.get('success'):
            result['lidar_yaw_offset_deg'] = round(
                static_result['offset_deg'], 1)
            result['method_used'] = 'static'
            result['success'] = True

            if bool(self.get_parameter('auto_apply').value):
                self._apply_offset(static_result['offset_deg'])
                result['applied'] = True
                self.get_logger().info(
                    f'[Cal] Applied lidar_yaw_offset = '
                    f'{static_result["offset_deg"]:.1f}° (static)')
            else:
                result['applied'] = False

        msg = String()
        msg.data = json.dumps(result)
        self._result_pub.publish(msg)
        self.get_logger().info(f'[Cal] Result: {json.dumps(result, indent=2)}')
        return result

    # ── Phase 1: Static LiDAR-ultrasonic correlation ──────────────────

    def _calibrate_static(self) -> dict:
        """
        Cross-correlate ultrasonic distance with LiDAR angular bins.

        The ultrasonic sensor is physically fixed pointing forward on the chassis.
        By comparing its distance reading with the LiDAR's per-angle ranges,
        we find which LiDAR angle corresponds to the physical forward direction.

        Algorithm:
          1. Collect N simultaneous ultrasonic + LiDAR samples.
          2. Build a per-angle average range profile from the LiDAR data.
          3. For each candidate angle θ ∈ [-180°, +180°):
             - Compute the minimum LiDAR range in a ±beam_half_deg window
               centered at θ (mimicking the ultrasonic's wide beam).
             - Score = |min_lidar(θ) − mean_ultrasonic|
          4. The θ with the lowest score is the offset.
        """
        num_samples = int(self.get_parameter('static_samples').value)
        interval = float(self.get_parameter('static_sample_interval').value)
        beam_half = float(self.get_parameter('us_beam_half_deg').value) * DEG_TO_RAD
        tolerance = float(self.get_parameter('us_match_tolerance_m').value)
        us_max = float(self.get_parameter('us_max_range').value)

        # Validate sensor availability
        if self._latest_scan is None:
            return {'success': False, 'error': 'No LiDAR data received'}
        if self._latest_us is None:
            return {'success': False, 'error': 'No ultrasonic data received'}

        # ── Collect samples ────────────────────────────────────────────
        us_samples = []
        scan_angle_ranges = {}  # angle_index → [range values]

        for _ in range(num_samples):
            # Grab current readings
            us = self._latest_us
            scan = self._latest_scan
            if us is None or scan is None:
                time.sleep(interval)
                continue

            if us < us_max:
                us_samples.append(us)

            # Accumulate per-angle ranges
            angle = scan.angle_min
            for i, r in enumerate(scan.ranges):
                if not (math.isinf(r) or math.isnan(r) or r < 0.02):
                    if i not in scan_angle_ranges:
                        scan_angle_ranges[i] = []
                    scan_angle_ranges[i].append(r)
                angle += scan.angle_increment

            time.sleep(interval)

        if len(us_samples) < 3:
            return {
                'success': False,
                'error': f'Too few ultrasonic samples ({len(us_samples)})'
            }

        us_mean = sum(us_samples) / len(us_samples)

        # Need a reference scan for angle geometry
        ref_scan = self._latest_scan
        if ref_scan is None:
            return {'success': False, 'error': 'Lost LiDAR data'}

        # Build averaged range array
        n_rays = len(ref_scan.ranges)
        avg_ranges = [float('inf')] * n_rays
        for i in range(n_rays):
            if i in scan_angle_ranges and scan_angle_ranges[i]:
                avg_ranges[i] = sum(scan_angle_ranges[i]) / len(
                    scan_angle_ranges[i])

        # ── Scan every candidate angle ─────────────────────────────────
        # For each candidate "forward" direction θ, compute what the
        # ultrasonic would read if it pointed at θ (min range in a cone).
        best_score = float('inf')
        best_angle = 0.0
        candidates = []

        for i_center in range(n_rays):
            center_angle = normalize_angle(
                ref_scan.angle_min + i_center * ref_scan.angle_increment)

            # Collect min range in ±beam_half cone around center_angle
            min_in_cone = float('inf')
            for j in range(n_rays):
                j_angle = normalize_angle(
                    ref_scan.angle_min + j * ref_scan.angle_increment)
                delta = abs(normalize_angle(j_angle - center_angle))
                if delta <= beam_half:
                    if avg_ranges[j] < min_in_cone:
                        min_in_cone = avg_ranges[j]

            if math.isinf(min_in_cone):
                continue

            score = abs(min_in_cone - us_mean)
            candidates.append({
                'angle_deg': round(center_angle * RAD_TO_DEG, 1),
                'lidar_range': round(min_in_cone, 3),
                'score': round(score, 4),
            })

            if score < best_score:
                best_score = score
                best_angle = center_angle

        # Dynamic tolerance — at close range the physical mounting offset
        # between the US sensor and LiDAR dominates, so widen tolerance.
        effective_tol = max(tolerance, us_mean * 0.75)

        matched = best_score <= effective_tol
        match_method = 'absolute'

        # Fallback: relative scoring — if this angle is clearly dominant
        # (much lower score than the median), accept it.
        if not matched and len(candidates) >= 10:
            scores = sorted(c['score'] for c in candidates)
            median_score = scores[len(scores) // 2]
            if median_score > 0 and best_score < median_score * 0.4:
                matched = True
                match_method = 'relative'
                self.get_logger().info(
                    f'[Cal/static] Tolerance exceeded but best score '
                    f'({best_score:.3f}m) is clearly dominant '
                    f'(median={median_score:.3f}m), accepting')

        if not matched:
            return {
                'success': False,
                'error': f'No LiDAR angle matched ultrasonic range '
                         f'({us_mean:.3f}m) within tolerance '
                         f'({effective_tol:.3f}m). '
                         f'Best score: {best_score:.3f}m at '
                         f'{round(best_angle * RAD_TO_DEG, 1)}°. '
                         f'Place the robot facing a wall 0.1–2 m away '
                         f'and retry.',
                'us_mean_m': round(us_mean, 3),
                'best_score': round(best_score, 4),
                'best_angle_deg': round(best_angle * RAD_TO_DEG, 1),
                'effective_tolerance_m': round(effective_tol, 3),
            }

        offset_deg = round(best_angle * RAD_TO_DEG, 1)
        self.get_logger().info(
            f'[Cal/static] Ultrasonic = {us_mean:.3f}m, '
            f'best LiDAR match at {offset_deg}° '
            f'(score={best_score:.3f}m, method={match_method})')

        return {
            'success': True,
            'offset_deg': offset_deg,
            'us_mean_m': round(us_mean, 3),
            'match_score': round(best_score, 4),
            'match_method': match_method,
            'confidence': 'high' if best_score <= tolerance else 'medium',
            'interpretation': self._interpret_offset(offset_deg),
        }

    # ── Phase 2: Motion-based forward detection ───────────────────────

    def _calibrate_motion(self) -> dict:
        """
        Drive forward briefly, then compare before/after LiDAR scans.

        When the robot moves in the +X (forward) direction, obstacles in the
        actual forward direction get closer (range decreases) and obstacles
        behind get further (range increases).  The LiDAR angle showing the
        maximum range *decrease* is the true forward direction.

        Algorithm:
          1. Collect N averaged LiDAR scans (before).
          2. Drive at +linear.x for drive_duration seconds, then stop.
          3. Wait for robot to settle.
          4. Collect N averaged LiDAR scans (after).
          5. For each angle: delta = after_range − before_range.
          6. Smooth deltas with an angular window.
          7. Forward = angle of most negative smoothed delta.
        """
        n_samples = int(self.get_parameter('drive_scan_samples').value)
        drive_speed = float(self.get_parameter('drive_speed').value)
        drive_dur = float(self.get_parameter('drive_duration').value)
        settle = float(self.get_parameter('drive_settle').value)
        smooth_half = float(
            self.get_parameter('angle_smooth_window_deg').value) * DEG_TO_RAD

        if self._latest_scan is None:
            return {'success': False, 'error': 'No LiDAR data'}

        # ── Collect before scans ───────────────────────────────────────
        before = self._collect_averaged_scan(n_samples)
        if before is None:
            return {'success': False, 'error': 'Could not collect before-scans'}

        # ── Determine drive direction ──────────────────────────────────
        # If the robot is very close to a wall the collision failsafe will
        # block forward motion.  Drive backward instead and invert the
        # delta interpretation.
        us_range = self._latest_us
        drive_backward = (us_range is not None and us_range < 0.25)
        actual_speed = -drive_speed if drive_backward else drive_speed
        direction = 'backward' if drive_backward else 'forward'
        us_str = f'{us_range:.2f}m' if us_range is not None else 'N/A'

        self.get_logger().info(
            f'[Cal/motion] Driving {direction} at {abs(actual_speed)} m/s '
            f'for {drive_dur}s (US={us_str})')
        self._drive(actual_speed, 0.0, drive_dur)
        time.sleep(settle)

        # ── Collect after scans ────────────────────────────────────────
        after = self._collect_averaged_scan(n_samples)
        if after is None:
            return {'success': False, 'error': 'Could not collect after-scans'}

        # ── Compute range deltas ───────────────────────────────────────
        ref_scan = self._latest_scan
        n_rays = len(ref_scan.ranges)

        if len(before) != n_rays or len(after) != n_rays:
            return {
                'success': False,
                'error': f'Scan size mismatch: before={len(before)}, '
                         f'after={len(after)}, current={n_rays}'
            }

        # delta[i] = after[i] - before[i]; negative = objects got closer
        delta = [0.0] * n_rays
        valid = [False] * n_rays
        for i in range(n_rays):
            b, a = before[i], after[i]
            if not math.isinf(b) and not math.isinf(a):
                delta[i] = a - b
                valid[i] = True

        # ── Angular smoothing ─────────────────────────────────────────
        smoothed = [0.0] * n_rays
        for i in range(n_rays):
            center_angle = ref_scan.angle_min + i * ref_scan.angle_increment
            total = 0.0
            count = 0
            for j in range(n_rays):
                if not valid[j]:
                    continue
                j_angle = ref_scan.angle_min + j * ref_scan.angle_increment
                if abs(normalize_angle(j_angle - center_angle)) <= smooth_half:
                    total += delta[j]
                    count += 1
            if count > 0:
                smoothed[i] = total / count

        # Forward direction detection
        extreme_delta = 0.0
        best_i = 0

        if drive_backward:
            # Drove backward: wall in front moved further → positive delta.
            # Forward = angle with the most POSITIVE smoothed delta.
            for i in range(n_rays):
                if smoothed[i] > extreme_delta:
                    extreme_delta = smoothed[i]
                    best_i = i
        else:
            # Drove forward: wall in front moved closer → negative delta.
            # Forward = angle with the most NEGATIVE smoothed delta.
            for i in range(n_rays):
                if smoothed[i] < extreme_delta:
                    extreme_delta = smoothed[i]
                    best_i = i

        fwd_angle = normalize_angle(
            ref_scan.angle_min + best_i * ref_scan.angle_increment)
        fwd_deg = round(fwd_angle * RAD_TO_DEG, 1)

        # The delta should be meaningful (robot actually moved)
        if abs(extreme_delta) < 0.01:
            return {
                'success': False,
                'error': 'Range change too small — robot may not have moved. '
                         'Ensure wheels have traction and path is clear.',
                'max_delta_m': round(extreme_delta, 4),
                'drove_backward': drive_backward,
            }

        self.get_logger().info(
            f'[Cal/motion] Forward direction at {fwd_deg}° '
            f'(range delta={extreme_delta:.3f}m, drove {direction})')

        return {
            'success': True,
            'forward_angle_deg': fwd_deg,
            'range_delta_m': round(extreme_delta, 3),
            'drove_backward': drive_backward,
            'interpretation': self._interpret_offset(fwd_deg),
        }

    # ── Phase 3: IMU yaw sign check ──────────────────────────────────

    def _calibrate_imu_yaw(self) -> dict:
        """
        Send a brief CCW rotation and check if IMU yaw increases.

        In ROS convention, positive angular.z = CCW when viewed from above,
        and yaw should increase (more positive).
        """
        rotate_speed = float(self.get_parameter('rotate_speed').value)
        rotate_dur = float(self.get_parameter('rotate_duration').value)
        settle = float(self.get_parameter('rotate_settle').value)

        if self._latest_yaw is None:
            return {
                'success': True,
                'correct': None,
                'note': 'No IMU yaw data available — skipping yaw check',
            }

        yaw_before = self._latest_yaw

        # Rotate CCW
        self.get_logger().info(
            f'[Cal/IMU] Rotating CCW at {rotate_speed} rad/s for {rotate_dur}s')
        self._drive(0.0, rotate_speed, rotate_dur)
        time.sleep(settle)

        yaw_after = self._latest_yaw
        delta_yaw = yaw_after - yaw_before

        # Handle wraparound (yaw is in degrees, ±180)
        if delta_yaw > 180:
            delta_yaw -= 360
        elif delta_yaw < -180:
            delta_yaw += 360

        # Expect positive change (CCW = increasing yaw in ROS)
        correct = delta_yaw > 2.0  # at least 2° change in right direction
        ambiguous = abs(delta_yaw) < 2.0

        self.get_logger().info(
            f'[Cal/IMU] Yaw: {yaw_before:.1f}° → {yaw_after:.1f}° '
            f'(delta={delta_yaw:.1f}°, correct={correct})')

        if ambiguous:
            return {
                'success': True,
                'correct': None,
                'delta_deg': round(delta_yaw, 1),
                'note': 'Yaw change too small to determine — '
                        'ensure IMU is connected and robot rotated.',
            }

        result = {
            'success': True,
            'correct': correct,
            'delta_deg': round(delta_yaw, 1),
            'yaw_before_deg': round(yaw_before, 1),
            'yaw_after_deg': round(yaw_after, 1),
        }

        if not correct:
            result['recommendation'] = (
                'IMU yaw sign appears inverted. '
                'Set yaw_gyro_sign: -1.0 in raspbot_hw.yaml '
                'or heading_offset_deg: 180.0 for BNO055.'
            )

        return result

    # ── Helpers ───────────────────────────────────────────────────────

    def _collect_averaged_scan(self, n_samples: int, interval: float = 0.15):
        """Collect N scans and return per-angle average ranges."""
        ref = self._latest_scan
        if ref is None:
            return None

        n_rays = len(ref.ranges)
        totals = [0.0] * n_rays
        counts = [0] * n_rays

        for _ in range(n_samples):
            scan = self._latest_scan
            if scan is None or len(scan.ranges) != n_rays:
                time.sleep(interval)
                continue

            for i, r in enumerate(scan.ranges):
                if not (math.isinf(r) or math.isnan(r) or r < 0.02):
                    totals[i] += r
                    counts[i] += 1
            time.sleep(interval)

        result = [float('inf')] * n_rays
        for i in range(n_rays):
            if counts[i] > 0:
                result[i] = totals[i] / counts[i]

        return result

    def _drive(self, linear_x: float, angular_z: float,
               duration: float) -> None:
        """Send cmd_vel for a specified duration, then stop."""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z

        start = time.monotonic()
        while (time.monotonic() - start) < duration:
            self._cmd_pub.publish(twist)
            time.sleep(0.05)  # 20 Hz cmd_vel

        self._stop_robot()

    def _stop_robot(self) -> None:
        """Publish zero cmd_vel."""
        twist = Twist()
        for _ in range(5):
            self._cmd_pub.publish(twist)
            time.sleep(0.02)

    def _apply_offset(self, offset_deg: float) -> None:
        """Set lidar_yaw_offset on obstacle and SLAM nodes."""
        offset_rad = offset_deg * DEG_TO_RAD
        obstacle_node = str(self.get_parameter('lidar_obstacle_node').value)
        slam_node = str(self.get_parameter('lidar_slam_node').value)

        for node_name in [obstacle_node, slam_node]:
            try:
                cli = self.create_client(
                    rcl_interfaces_srv.SetParameters,
                    f'/{node_name}/set_parameters')
                if cli.wait_for_service(timeout_sec=1.0):
                    from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
                    param = Parameter()
                    param.name = 'lidar_yaw_offset'
                    param.value = ParameterValue()
                    param.value.type = ParameterType.PARAMETER_DOUBLE
                    param.value.double_value = offset_rad
                    req = rcl_interfaces_srv.SetParameters.Request()
                    req.parameters = [param]
                    future = cli.call_async(req)
                    # Wait briefly
                    deadline = time.time() + 2.0
                    while time.time() < deadline and not future.done():
                        time.sleep(0.05)
                    if future.done():
                        self.get_logger().info(
                            f'[Cal] Set lidar_yaw_offset={offset_rad:.4f} '
                            f'on {node_name}')
                    else:
                        self.get_logger().warn(
                            f'[Cal] Timeout setting param on {node_name}')
                else:
                    self.get_logger().warn(
                        f'[Cal] {node_name}/set_parameters not available')
                self.destroy_client(cli)
            except Exception as e:
                self.get_logger().warn(
                    f'[Cal] Could not set param on {node_name}: {e}')

    @staticmethod
    def _interpret_offset(offset_deg: float) -> str:
        """Human-readable interpretation of the angular offset."""
        a = offset_deg
        if abs(a) < 5:
            return 'LiDAR aligned with robot forward (no correction needed)'
        elif abs(a - 180) < 10 or abs(a + 180) < 10:
            return 'LiDAR is 180° rotated (mounted backwards)'
        elif abs(a - 90) < 10:
            return 'LiDAR is ~90° CCW from forward (mounted sideways-left)'
        elif abs(a + 90) < 10:
            return 'LiDAR is ~90° CW from forward (mounted sideways-right)'
        else:
            return f'LiDAR is {a:.0f}° from forward'


# ── Import for parameter setting ──────────────────────────────────────
try:
    from rcl_interfaces import srv as rcl_interfaces_srv
except ImportError:
    rcl_interfaces_srv = None


# ── Entry point ───────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = FrontCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
