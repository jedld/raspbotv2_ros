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

from sensor_msgs.msg import LaserScan, Range, Imu
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


def snap_to_quadrant(deg: float, threshold: float = 15.0) -> float:
    """Snap angle to nearest 90° multiple if within threshold."""
    for q in (-180.0, -90.0, 0.0, 90.0, 180.0):
        if abs(deg - q) < threshold:
            return q
    return deg


# ── Calibration Node ─────────────────────────────────────────────────

class FrontCalibrationNode(Node):
    """Auto-calibrate the robot's front-direction using sensor fusion."""

    def __init__(self):
        super().__init__('front_calibration')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('ultrasonic_topic', 'ultrasonic/range')
        self.declare_parameter('imu_yaw_topic', 'imu/yaw_deg')
        self.declare_parameter('imu_data_topic', 'imu/data')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')

        # Static calibration
        self.declare_parameter('static_samples', 20)         # scan + US samples
        self.declare_parameter('static_sample_interval', 0.12)  # seconds
        self.declare_parameter('us_validation_tolerance_m', 0.30)  # US vs LiDAR
        self.declare_parameter('us_max_range', 3.0)          # ignore US > this
        self.declare_parameter('wall_min_width_deg', 30.0)   # min angular span
        self.declare_parameter('snap_to_90', True)           # snap to nearest 90°
        self.declare_parameter('snap_threshold_deg', 15.0)   # snap if within this

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
        imu_data_topic = str(self.get_parameter('imu_data_topic').value)
        cmd_topic = str(self.get_parameter('cmd_vel_topic').value)

        # ── State ─────────────────────────────────────────────────────
        self._latest_scan = None       # most recent LaserScan
        self._latest_us = None         # most recent ultrasonic range (m)
        self._latest_yaw = None        # most recent IMU yaw (degrees)
        self._latest_imu = None        # most recent Imu message
        self._imu_accel_samples = []   # [(ax, ay, az)] during drive
        self._collect_imu_accel = False
        self._calibrating = False
        self._lock = threading.Lock()

        # ── Subscribers ───────────────────────────────────────────────
        self._scan_sub = self.create_subscription(
            LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data)
        self._us_sub = self.create_subscription(
            Range, us_topic, self._on_us, 10)
        self._yaw_sub = self.create_subscription(
            Float64, imu_topic, self._on_yaw, 10)
        self._imu_sub = self.create_subscription(
            Imu, imu_data_topic, self._on_imu, 10)

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

    def _on_imu(self, msg: Imu) -> None:
        self._latest_imu = msg
        if self._collect_imu_accel:
            ax = msg.linear_acceleration.x
            ay = msg.linear_acceleration.y
            az = msg.linear_acceleration.z
            self._imu_accel_samples.append((ax, ay, az))

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

        # Phase 1: static LiDAR wall-perpendicular detection
        self.get_logger().info('[Cal] Phase 1: Static wall-perpendicular detection')
        static_result = self._calibrate_static()
        results['static'] = static_result

        # Phase 2: motion-based forward detection + IMU mount check
        self.get_logger().info('[Cal] Phase 2: Drive-and-scan + IMU mount detection')
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
        results['imu_mount'] = motion_result.get('imu_mount', {})
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

    # ── Phase 1: Static wall-perpendicular detection ────────────────

    def _calibrate_static(self) -> dict:
        """
        Detect the forward direction by finding the angle of closest
        approach to the wall in the LiDAR scan.

        When the robot faces a flat wall, the LiDAR angle that is
        perpendicular to the wall surface shows the MINIMUM range.
        This angle is the robot's true forward direction in the LiDAR
        frame.  The ultrasonic sensor validates that a wall exists.

        Algorithm:
          1. Collect N LiDAR + ultrasonic samples.
          2. Build per-angle average range profile.
          3. Smooth the profile angularly (reduce noise).
          4. Find the global range minimum — this is the perpendicular
             to the wall, i.e. the forward direction.
          5. Validate: the minimum range should roughly agree with
             ultrasonic (within tolerance, accounting for sensor offsets).
          6. Optionally snap to nearest 90° if close (typical mounts).
        """
        num_samples = int(self.get_parameter('static_samples').value)
        interval = float(self.get_parameter('static_sample_interval').value)
        us_tol = float(self.get_parameter('us_validation_tolerance_m').value)
        us_max = float(self.get_parameter('us_max_range').value)
        wall_min_width = float(
            self.get_parameter('wall_min_width_deg').value) * DEG_TO_RAD
        do_snap = bool(self.get_parameter('snap_to_90').value)
        snap_thresh = float(self.get_parameter('snap_threshold_deg').value)

        # Validate sensor availability
        if self._latest_scan is None:
            return {'success': False, 'error': 'No LiDAR data received'}
        if self._latest_us is None:
            return {'success': False, 'error': 'No ultrasonic data received'}

        # ── Collect samples ────────────────────────────────────────────
        us_samples = []
        scan_angle_ranges = {}  # angle_index → [range values]

        for _ in range(num_samples):
            us = self._latest_us
            scan = self._latest_scan
            if us is None or scan is None:
                time.sleep(interval)
                continue

            if us < us_max:
                us_samples.append(us)

            # Accumulate per-angle ranges
            for i, r in enumerate(scan.ranges):
                if not (math.isinf(r) or math.isnan(r) or r < 0.02):
                    if i not in scan_angle_ranges:
                        scan_angle_ranges[i] = []
                    scan_angle_ranges[i].append(r)

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

        n_rays = len(ref_scan.ranges)
        inc = ref_scan.angle_increment

        # Build averaged range array
        avg_ranges = [float('inf')] * n_rays
        for i in range(n_rays):
            if i in scan_angle_ranges and scan_angle_ranges[i]:
                avg_ranges[i] = (
                    sum(scan_angle_ranges[i]) / len(scan_angle_ranges[i]))

        # ── Angular smoothing of the range profile ─────────────────────
        smooth_half_idx = max(1, int(5.0 * DEG_TO_RAD / inc))
        smoothed = [float('inf')] * n_rays
        for i in range(n_rays):
            total = 0.0
            weight = 0.0
            for j in range(max(0, i - smooth_half_idx),
                           min(n_rays, i + smooth_half_idx + 1)):
                if not math.isinf(avg_ranges[j]):
                    d = abs(j - i)
                    w = 1.0 / (1.0 + d)
                    total += avg_ranges[j] * w
                    weight += w
            if weight > 0:
                smoothed[i] = total / weight

        # ── Find global minimum range (wall perpendicular) ────────────
        min_range = float('inf')
        min_idx = 0
        for i in range(n_rays):
            if smoothed[i] < min_range:
                min_range = smoothed[i]
                min_idx = i

        if math.isinf(min_range):
            return {'success': False, 'error': 'No valid LiDAR data'}

        min_angle_raw = normalize_angle(
            ref_scan.angle_min + min_idx * inc)

        # ── Refine: parabolic interpolation around minimum ─────────────
        refined_angle = min_angle_raw
        if 1 <= min_idx <= n_rays - 2:
            r_prev = smoothed[min_idx - 1]
            r_curr = smoothed[min_idx]
            r_next = smoothed[min_idx + 1]
            if (not math.isinf(r_prev) and not math.isinf(r_next)
                    and r_prev > r_curr and r_next > r_curr):
                a_coef = (r_prev + r_next) / 2.0 - r_curr
                b_coef = (r_next - r_prev) / 2.0
                if abs(a_coef) > 1e-9:
                    x_min = -b_coef / (2.0 * a_coef)
                    x_min = max(-1.0, min(1.0, x_min))
                    refined_angle = normalize_angle(
                        ref_scan.angle_min + (min_idx + x_min) * inc)

        raw_deg = round(refined_angle * RAD_TO_DEG, 1)

        # ── Validate: wall signature check ─────────────────────────────
        wall_half_idx = max(1, int(wall_min_width / 2.0 / inc))
        left_ok = False
        right_ok = False
        for k in range(1, wall_half_idx + 1):
            li = min_idx - k
            ri = min_idx + k
            if 0 <= li < n_rays and not math.isinf(smoothed[li]):
                if smoothed[li] > min_range * 1.01:
                    left_ok = True
            if 0 <= ri < n_rays and not math.isinf(smoothed[ri]):
                if smoothed[ri] > min_range * 1.01:
                    right_ok = True
        wall_valid = left_ok and right_ok

        # ── Validate: ultrasonic cross-check ──────────────────────────
        range_diff = abs(min_range - us_mean)
        us_valid = range_diff <= us_tol

        self.get_logger().info(
            f'[Cal/static] Wall minimum at {raw_deg}° '
            f'(LiDAR={min_range:.3f}m, US={us_mean:.3f}m, '
            f'diff={range_diff:.3f}m, '
            f'wall_shape={wall_valid}, us_valid={us_valid})')

        if not wall_valid:
            indexed = [(smoothed[i], i) for i in range(n_rays)
                       if not math.isinf(smoothed[i])]
            indexed.sort()
            top5 = [{
                'angle_deg': round(normalize_angle(
                    ref_scan.angle_min + idx * inc) * RAD_TO_DEG, 1),
                'range_m': round(rng, 3)
            } for rng, idx in indexed[:5]]

            return {
                'success': False,
                'error': 'No clear wall detected — range minimum does not '
                         'have a wall-shaped profile (increasing range on '
                         'both sides). Ensure robot faces a flat wall.',
                'min_angle_deg': raw_deg,
                'min_range_m': round(min_range, 3),
                'us_mean_m': round(us_mean, 3),
                'top5_minima': top5,
            }

        if not us_valid:
            return {
                'success': False,
                'error': f'LiDAR minimum range ({min_range:.3f}m) differs '
                         f'from ultrasonic ({us_mean:.3f}m) by '
                         f'{range_diff:.3f}m, exceeding tolerance '
                         f'({us_tol:.2f}m). The minimum may not be the '
                         f'wall in front. Try moving closer to the wall.',
                'min_angle_deg': raw_deg,
                'min_range_m': round(min_range, 3),
                'us_mean_m': round(us_mean, 3),
                'range_diff_m': round(range_diff, 3),
            }

        # ── Snap to nearest 90° if configured ─────────────────────────
        offset_deg = raw_deg
        snapped = False
        if do_snap:
            snapped_deg = snap_to_quadrant(raw_deg, snap_thresh)
            if snapped_deg != raw_deg:
                snapped = True
                self.get_logger().info(
                    f'[Cal/static] Snapping {raw_deg}° → {snapped_deg}° '
                    f'(nearest 90° quadrant)')
                offset_deg = snapped_deg

        confidence = 'high'
        if range_diff > us_tol * 0.5:
            confidence = 'medium'

        self.get_logger().info(
            f'[Cal/static] Forward direction: {offset_deg}° '
            f'(raw={raw_deg}°, snapped={snapped}, '
            f'confidence={confidence})')

        return {
            'success': True,
            'offset_deg': offset_deg,
            'raw_offset_deg': raw_deg,
            'snapped': snapped,
            'min_range_m': round(min_range, 3),
            'us_mean_m': round(us_mean, 3),
            'range_diff_m': round(range_diff, 3),
            'confidence': confidence,
            'interpretation': self._interpret_offset(offset_deg),
        }

    # ── Phase 2: Motion-based forward detection + IMU mount check ────

    def _calibrate_motion(self) -> dict:
        """
        Drive forward briefly, compare before/after LiDAR scans, and
        monitor IMU acceleration to detect sideways mounting.

        LiDAR: The angle with the largest range change indicates forward.
        IMU: If the dominant acceleration during forward drive is on the
        Y-axis instead of X-axis, the IMU is mounted sideways.
        """
        n_samples = int(self.get_parameter('drive_scan_samples').value)
        drive_speed = float(self.get_parameter('drive_speed').value)
        drive_dur = float(self.get_parameter('drive_duration').value)
        settle = float(self.get_parameter('drive_settle').value)
        smooth_half = float(
            self.get_parameter('angle_smooth_window_deg').value) * DEG_TO_RAD
        do_snap = bool(self.get_parameter('snap_to_90').value)
        snap_thresh = float(self.get_parameter('snap_threshold_deg').value)

        if self._latest_scan is None:
            return {'success': False, 'error': 'No LiDAR data'}

        # ── Collect stationary IMU baseline ────────────────────────────
        self._imu_accel_samples = []
        self._collect_imu_accel = True
        time.sleep(0.5)
        baseline_samples = list(self._imu_accel_samples)
        self._imu_accel_samples = []

        # ── Collect before scans ───────────────────────────────────────
        before = self._collect_averaged_scan(n_samples)
        if before is None:
            self._collect_imu_accel = False
            return {'success': False, 'error': 'Could not collect before-scans'}

        # ── Determine drive direction ──────────────────────────────────
        us_range = self._latest_us
        drive_backward = (us_range is not None and us_range < 0.25)
        actual_speed = -drive_speed if drive_backward else drive_speed
        direction = 'backward' if drive_backward else 'forward'
        us_str = f'{us_range:.2f}m' if us_range is not None else 'N/A'

        # ── Drive and collect IMU accel ────────────────────────────────
        self._imu_accel_samples = []
        self.get_logger().info(
            f'[Cal/motion] Driving {direction} at {abs(actual_speed)} m/s '
            f'for {drive_dur}s (US={us_str})')
        self._drive(actual_speed, 0.0, drive_dur)
        drive_accel_samples = list(self._imu_accel_samples)
        self._collect_imu_accel = False

        time.sleep(settle)

        # ── Collect after scans ────────────────────────────────────────
        after = self._collect_averaged_scan(n_samples)
        if after is None:
            return {'success': False, 'error': 'Could not collect after-scans'}

        # ── Analyze IMU mount orientation ──────────────────────────────
        imu_mount = self._analyze_imu_mount(
            baseline_samples, drive_accel_samples, drive_backward)

        # ── Compute range deltas ───────────────────────────────────────
        ref_scan = self._latest_scan
        n_rays = len(ref_scan.ranges)
        inc = ref_scan.angle_increment

        if len(before) != n_rays or len(after) != n_rays:
            return {
                'success': False,
                'error': f'Scan size mismatch: before={len(before)}, '
                         f'after={len(after)}, current={n_rays}',
                'imu_mount': imu_mount,
            }

        delta = [0.0] * n_rays
        valid = [False] * n_rays
        for i in range(n_rays):
            b, a = before[i], after[i]
            if not math.isinf(b) and not math.isinf(a):
                delta[i] = a - b
                valid[i] = True

        # ── Angular smoothing ─────────────────────────────────────────
        smooth_half_idx = max(1, int(smooth_half / inc))
        smoothed = [0.0] * n_rays
        for i in range(n_rays):
            total = 0.0
            count = 0
            for j in range(max(0, i - smooth_half_idx),
                           min(n_rays, i + smooth_half_idx + 1)):
                if valid[j]:
                    total += delta[j]
                    count += 1
            if count > 0:
                smoothed[i] = total / count

        # ── Find forward direction ─────────────────────────────────────
        extreme_delta = 0.0
        best_i = 0

        if drive_backward:
            for i in range(n_rays):
                if smoothed[i] > extreme_delta:
                    extreme_delta = smoothed[i]
                    best_i = i
        else:
            for i in range(n_rays):
                if smoothed[i] < extreme_delta:
                    extreme_delta = smoothed[i]
                    best_i = i

        fwd_angle = normalize_angle(
            ref_scan.angle_min + best_i * inc)
        fwd_deg = round(fwd_angle * RAD_TO_DEG, 1)

        if abs(extreme_delta) < 0.01:
            return {
                'success': False,
                'error': 'Range change too small — robot may not have moved. '
                         'Ensure wheels have traction and path is clear.',
                'max_delta_m': round(extreme_delta, 4),
                'drove_backward': drive_backward,
                'imu_mount': imu_mount,
            }

        # Snap to quadrant
        snapped = False
        if do_snap:
            snapped_deg = snap_to_quadrant(fwd_deg, snap_thresh)
            if snapped_deg != fwd_deg:
                snapped = True
                self.get_logger().info(
                    f'[Cal/motion] Snapping {fwd_deg}° → {snapped_deg}°')
                fwd_deg = snapped_deg

        self.get_logger().info(
            f'[Cal/motion] Forward direction at {fwd_deg}° '
            f'(range delta={extreme_delta:.3f}m, drove {direction})')

        return {
            'success': True,
            'forward_angle_deg': fwd_deg,
            'raw_angle_deg': round(fwd_angle * RAD_TO_DEG, 1),
            'snapped': snapped,
            'range_delta_m': round(extreme_delta, 3),
            'drove_backward': drive_backward,
            'imu_mount': imu_mount,
            'interpretation': self._interpret_offset(fwd_deg),
        }

    def _analyze_imu_mount(self, baseline: list, drive: list,
                           drove_backward: bool) -> dict:
        """
        Analyze IMU acceleration during forward drive to detect
        sideways IMU mounting.

        When driving forward (+X), the IMU should show dominant
        acceleration on its X-axis.  If the IMU board is rotated
        (e.g. mounted sideways), the dominant axis will be different.

        The RP2040 firmware auto-orients Z=up via gravity detection,
        so only the X/Y rotation (yaw of IMU vs robot) is unknown.
        """
        if len(drive) < 5:
            return {
                'detected': False,
                'note': 'Insufficient IMU data during drive',
            }

        # Compute mean acceleration during baseline (stationary)
        base_ax = base_ay = 0.0
        if len(baseline) > 3:
            base_ax = sum(s[0] for s in baseline) / len(baseline)
            base_ay = sum(s[1] for s in baseline) / len(baseline)

        # Compute mean acceleration during drive (subtract baseline)
        drive_ax = sum(s[0] for s in drive) / len(drive) - base_ax
        drive_ay = sum(s[1] for s in drive) / len(drive) - base_ay

        # If we drove backward, invert so the analysis is as-if forward
        if drove_backward:
            drive_ax = -drive_ax
            drive_ay = -drive_ay

        accel_mag = math.sqrt(drive_ax**2 + drive_ay**2)

        if accel_mag < 0.05:
            return {
                'detected': False,
                'note': 'Acceleration too small to determine IMU orientation '
                        '— robot may not have moved.',
                'accel_x': round(drive_ax, 3),
                'accel_y': round(drive_ay, 3),
                'accel_mag': round(accel_mag, 3),
            }

        # atan2(y, x) gives the angle of the accel vector in IMU frame
        # If IMU is aligned with robot: angle ≈ 0° (forward = +X)
        # If IMU is 90° CW: forward drive → +Y, so angle ≈ 90°
        # If IMU is 90° CCW: forward drive → -Y, so angle ≈ -90°
        imu_fwd_angle_deg = round(
            math.atan2(drive_ay, drive_ax) * RAD_TO_DEG, 1)

        sideways = abs(imu_fwd_angle_deg) > 30.0
        snapped_mount = snap_to_quadrant(imu_fwd_angle_deg, 20.0)

        recommendations = []
        if abs(snapped_mount) > 10:
            if abs(snapped_mount - 90) < 20:
                recommendations.append(
                    'IMU (RP2040) is mounted ~90° CW from robot forward. '
                    'Set gyro_axis_map: [1, 0, 2] and '
                    'gyro_axis_sign: [-1.0, 1.0, 1.0] in raspbot_hw.yaml, '
                    'or set heading_offset_deg: -90.0')
            elif abs(snapped_mount + 90) < 20:
                recommendations.append(
                    'IMU (RP2040) is mounted ~90° CCW from robot forward. '
                    'Set gyro_axis_map: [1, 0, 2] and '
                    'gyro_axis_sign: [1.0, -1.0, 1.0] in raspbot_hw.yaml, '
                    'or set heading_offset_deg: 90.0')
            elif abs(abs(snapped_mount) - 180) < 20:
                recommendations.append(
                    'IMU (RP2040) is mounted ~180° from robot forward. '
                    'Set heading_offset_deg: 180.0 in raspbot_hw.yaml')
            else:
                recommendations.append(
                    f'IMU (RP2040) appears rotated {snapped_mount:.0f}° '
                    f'from robot forward. Set heading_offset_deg: '
                    f'{-snapped_mount:.0f} in raspbot_hw.yaml')

        result = {
            'detected': True,
            'sideways': sideways,
            'imu_forward_angle_deg': imu_fwd_angle_deg,
            'mount_rotation_deg': round(snapped_mount, 0),
            'raw_accel_x': round(drive_ax, 3),
            'raw_accel_y': round(drive_ay, 3),
            'accel_mag': round(accel_mag, 3),
            'drove_backward': drove_backward,
            'n_samples': len(drive),
        }
        if recommendations:
            result['recommendations'] = recommendations

        self.get_logger().info(
            f'[Cal/IMU mount] accel during drive: '
            f'X={drive_ax:.3f} Y={drive_ay:.3f} m/s² '
            f'(mag={accel_mag:.3f}), '
            f'IMU forward at {imu_fwd_angle_deg}° in IMU frame, '
            f'sideways={sideways}')

        return result

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
