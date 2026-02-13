"""
Lightweight occupancy grid SLAM for Raspbot V2.

Builds a 2-D occupancy grid from LaserScan + Odometry using log-odds
Bresenham ray-casting.  Optionally refines the odometry by correlative
scan-to-map matching, publishing the correction as TF ``map → odom``.

Subscribes:
  /scan          (sensor_msgs/LaserScan)
  /odom          (nav_msgs/Odometry)

Publishes:
  /map           (nav_msgs/OccupancyGrid)
  /slam/active   (std_msgs/Bool)
  TF: map → odom (drift correction; identity when scan-matching disabled)

Services:
  /slam/reset    (Trigger) — clear the map and reset pose correction
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import TransformStamped, Quaternion
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

import tf2_ros

# ── helpers ──────────────────────────────────────────────────────────

DEG_TO_RAD = math.pi / 180.0


def yaw_from_quaternion(q) -> float:
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
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


# ── LiDAR SLAM Node ─────────────────────────────────────────────────

class LidarSlamNode(Node):
    """Lightweight occupancy-grid SLAM with optional scan-matching."""

    _TUNABLE_PARAMS = {
        'slam_update_hz': '_update_hz',
        'slam_map_publish_hz': '_map_publish_hz',
        'slam_scan_match_enable': '_scan_match_enable',
        'slam_min_range': '_min_range',
        'slam_max_range': '_max_range',
        'lidar_yaw_offset': '_yaw_offset',
    }

    def __init__(self):
        super().__init__('lidar_slam')

        # ── Declare parameters ────────────────────────────────────────
        # Grid
        self.declare_parameter('slam_resolution', 0.05)       # m per cell
        self.declare_parameter('slam_width', 200)              # cells
        self.declare_parameter('slam_height', 200)             # cells
        self.declare_parameter('slam_origin_x', -5.0)          # metres
        self.declare_parameter('slam_origin_y', -5.0)

        # Update rates
        self.declare_parameter('slam_update_hz', 2.0)          # grid update rate
        self.declare_parameter('slam_map_publish_hz', 1.0)     # OccupancyGrid publish rate
        self.declare_parameter('slam_tf_publish_hz', 10.0)     # map→odom TF rate

        # Log-odds
        self.declare_parameter('slam_log_odds_occ', 0.85)
        self.declare_parameter('slam_log_odds_free', -0.40)
        self.declare_parameter('slam_log_odds_max', 5.0)
        self.declare_parameter('slam_log_odds_min', -2.0)
        self.declare_parameter('slam_unknown_threshold', 0.15)  # |lo| < this → unknown

        # Scan processing
        self.declare_parameter('slam_min_range', 0.10)         # ignore points closer
        self.declare_parameter('slam_max_range', 8.0)          # ignore points farther
        self.declare_parameter('slam_scan_downsample', 3)      # use every Nth point

        # Scan matching (correlative)
        self.declare_parameter('slam_scan_match_enable', False)
        self.declare_parameter('slam_scan_match_min_scans', 20) # warmup before matching
        self.declare_parameter('slam_scan_match_search_xy', 0.05)    # ±m search window
        self.declare_parameter('slam_scan_match_search_yaw', 0.035)  # ±rad (~2°)
        self.declare_parameter('slam_scan_match_step_xy', 0.01)      # m step
        self.declare_parameter('slam_scan_match_step_yaw', 0.005)    # rad step (~0.3°)
        self.declare_parameter('slam_scan_match_max_correction_xy', 0.02)   # m/update
        self.declare_parameter('slam_scan_match_max_correction_yaw', 0.01)  # rad/update

        # Topics
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('odom_frame_id', 'odom')

        # Angular offset: LiDAR forward vs robot forward (radians, set by calibration)
        self.declare_parameter('lidar_yaw_offset', 0.0)

        # ── Load parameters ───────────────────────────────────────────
        self._resolution = float(self.get_parameter('slam_resolution').value)
        self._width = int(self.get_parameter('slam_width').value)
        self._height = int(self.get_parameter('slam_height').value)
        self._origin_x = float(self.get_parameter('slam_origin_x').value)
        self._origin_y = float(self.get_parameter('slam_origin_y').value)

        self._update_hz = float(self.get_parameter('slam_update_hz').value)
        self._map_publish_hz = float(self.get_parameter('slam_map_publish_hz').value)
        self._tf_publish_hz = float(self.get_parameter('slam_tf_publish_hz').value)

        self._log_occ = float(self.get_parameter('slam_log_odds_occ').value)
        self._log_free = float(self.get_parameter('slam_log_odds_free').value)
        self._log_max = float(self.get_parameter('slam_log_odds_max').value)
        self._log_min = float(self.get_parameter('slam_log_odds_min').value)
        self._unknown_thresh = float(self.get_parameter('slam_unknown_threshold').value)

        self._min_range = float(self.get_parameter('slam_min_range').value)
        self._max_range = float(self.get_parameter('slam_max_range').value)
        self._downsample = max(1, int(self.get_parameter('slam_scan_downsample').value))

        self._scan_match_enable = bool(self.get_parameter('slam_scan_match_enable').value)
        self._scan_match_min_scans = int(self.get_parameter('slam_scan_match_min_scans').value)
        self._search_xy = float(self.get_parameter('slam_scan_match_search_xy').value)
        self._search_yaw = float(self.get_parameter('slam_scan_match_search_yaw').value)
        self._step_xy = float(self.get_parameter('slam_scan_match_step_xy').value)
        self._step_yaw = float(self.get_parameter('slam_scan_match_step_yaw').value)
        self._max_corr_xy = float(self.get_parameter('slam_scan_match_max_correction_xy').value)
        self._max_corr_yaw = float(self.get_parameter('slam_scan_match_max_correction_yaw').value)

        map_topic = str(self.get_parameter('map_topic').value)
        scan_topic = str(self.get_parameter('scan_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        self._map_frame = str(self.get_parameter('map_frame_id').value)
        self._odom_frame = str(self.get_parameter('odom_frame_id').value)

        self._yaw_offset = float(self.get_parameter('lidar_yaw_offset').value)

        # ── Occupancy grid (flat list, log-odds) ──────────────────────
        self._grid = [0.0] * (self._width * self._height)
        self._lock = threading.Lock()

        # ── Odometry state ────────────────────────────────────────────
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._odom_received = False

        # ── Map → odom correction (scan-matching output) ──────────────
        self._corr_x = 0.0
        self._corr_y = 0.0
        self._corr_yaw = 0.0

        # ── Timing ────────────────────────────────────────────────────
        self._update_period = 1.0 / max(self._update_hz, 0.1)
        self._last_update_time = 0.0
        self._scan_count = 0

        # ── Publishers ────────────────────────────────────────────────
        self._map_pub = self.create_publisher(OccupancyGrid, map_topic, 10)
        self._active_pub = self.create_publisher(Bool, 'slam/active', 10)

        # TF broadcaster: map → odom
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ── Subscribers ───────────────────────────────────────────────
        self._scan_sub = self.create_subscription(
            LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data
        )
        self._odom_sub = self.create_subscription(
            Odometry, odom_topic, self._on_odom, 10
        )

        # ── Services ─────────────────────────────────────────────────
        self.create_service(Trigger, 'slam/reset', self._srv_reset)

        # ── Timers ────────────────────────────────────────────────────
        map_period = 1.0 / max(self._map_publish_hz, 0.1)
        self._map_timer = self.create_timer(map_period, self._publish_map)

        tf_period = 1.0 / max(self._tf_publish_hz, 0.1)
        self._tf_timer = self.create_timer(tf_period, self._publish_tf)

        # Periodic active-status heartbeat (2 Hz)
        self._status_timer = self.create_timer(0.5, self._publish_status)

        # Live-tuning
        self.add_on_set_parameters_callback(self._on_param_change)

        self.get_logger().info(
            f'LiDAR SLAM started — grid {self._width}×{self._height} '
            f'@ {self._resolution}m, update {self._update_hz}Hz, '
            f'scan_match={self._scan_match_enable}, '
            f'range [{self._min_range}–{self._max_range}]m'
        )

    # ── Parameter change ──────────────────────────────────────────────

    def _on_param_change(self, params) -> SetParametersResult:
        for p in params:
            attr = self._TUNABLE_PARAMS.get(p.name)
            if attr:
                setattr(self, attr, p.value)
                self.get_logger().info(f'[slam] param {p.name} → {p.value}')
                if p.name == 'slam_update_hz':
                    self._update_period = 1.0 / max(float(p.value), 0.1)
        return SetParametersResult(successful=True)

    # ── Subscriber callbacks ──────────────────────────────────────────

    def _on_odom(self, msg: Odometry) -> None:
        """Track latest odometry pose."""
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        self._odom_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self._odom_received = True

    def _on_scan(self, msg: LaserScan) -> None:
        """Process incoming laser scan — update occupancy grid."""
        if not self._odom_received:
            return

        now = time.monotonic()
        if (now - self._last_update_time) < self._update_period:
            return
        self._last_update_time = now

        # Compute robot pose in map frame: map_pose = correction ⊕ odom_pose
        cos_c = math.cos(self._corr_yaw)
        sin_c = math.sin(self._corr_yaw)
        map_x = self._corr_x + cos_c * self._odom_x - sin_c * self._odom_y
        map_y = self._corr_y + sin_c * self._odom_x + cos_c * self._odom_y
        map_yaw = normalize_angle(self._corr_yaw + self._odom_yaw)

        # Optional: scan-to-map matching for drift correction
        if (self._scan_match_enable
                and self._scan_count >= self._scan_match_min_scans):
            matched_x, matched_y, matched_yaw, score = self._scan_match(
                msg, map_x, map_y, map_yaw
            )
            if score > 0:
                # Limit correction step size for smooth updates
                dx = self._clamp(matched_x - map_x,
                                 -self._max_corr_xy, self._max_corr_xy)
                dy = self._clamp(matched_y - map_y,
                                 -self._max_corr_xy, self._max_corr_xy)
                dyaw = self._clamp(
                    normalize_angle(matched_yaw - map_yaw),
                    -self._max_corr_yaw, self._max_corr_yaw
                )
                # Update correction transform
                self._corr_x += dx
                self._corr_y += dy
                self._corr_yaw = normalize_angle(self._corr_yaw + dyaw)
                # Recompute map pose with updated correction
                cos_c = math.cos(self._corr_yaw)
                sin_c = math.sin(self._corr_yaw)
                map_x = self._corr_x + cos_c * self._odom_x - sin_c * self._odom_y
                map_y = self._corr_y + sin_c * self._odom_x + cos_c * self._odom_y
                map_yaw = normalize_angle(self._corr_yaw + self._odom_yaw)

        # Update occupancy grid
        with self._lock:
            self._update_grid(msg, map_x, map_y, map_yaw)
        self._scan_count += 1

    # ── Grid operations ───────────────────────────────────────────────

    def _world_to_grid(self, wx: float, wy: float):
        """Convert world coordinates to grid cell indices."""
        gx = int((wx - self._origin_x) / self._resolution)
        gy = int((wy - self._origin_y) / self._resolution)
        return gx, gy

    def _in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self._width and 0 <= gy < self._height

    def _update_grid(self, scan: LaserScan,
                     robot_x: float, robot_y: float, robot_yaw: float) -> None:
        """Update occupancy grid with Bresenham ray-casting."""
        rx, ry = self._world_to_grid(robot_x, robot_y)
        angle = scan.angle_min
        w = self._width
        grid = self._grid
        log_occ = self._log_occ
        log_free = self._log_free
        log_max = self._log_max
        log_min = self._log_min
        ds = self._downsample
        min_r = self._min_range
        max_r = self._max_range
        res = self._resolution
        ox = self._origin_x
        oy = self._origin_y

        for i, r in enumerate(scan.ranges):
            if i % ds != 0:
                angle += scan.angle_increment
                continue

            if r < min_r or r > max_r or math.isinf(r) or math.isnan(r):
                angle += scan.angle_increment
                continue

            # Endpoint in map frame (apply yaw offset to correct LiDAR mounting)
            beam_angle = robot_yaw + angle - self._yaw_offset
            ex = robot_x + r * math.cos(beam_angle)
            ey = robot_y + r * math.sin(beam_angle)

            gx = int((ex - ox) / res)
            gy = int((ey - oy) / res)

            # Bresenham ray: mark free cells, then occupied endpoint
            self._trace_ray(rx, ry, gx, gy, grid, w, log_free, log_occ,
                            log_min, log_max)

            angle += scan.angle_increment

    @staticmethod
    def _trace_ray(x0, y0, x1, y1, grid, w, log_free, log_occ,
                   log_min, log_max):
        """Bresenham line from (x0,y0) to (x1,y1).

        Marks traversed cells as free, endpoint cell as occupied.
        Operates directly on the grid list for speed.
        """
        h = len(grid) // w
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        cx, cy = x0, y0
        while True:
            if cx == x1 and cy == y1:
                # Endpoint → occupied
                if 0 <= cx < w and 0 <= cy < h:
                    idx = cy * w + cx
                    v = grid[idx] + log_occ
                    grid[idx] = v if v < log_max else log_max
                break
            else:
                # Free space along the ray
                if 0 <= cx < w and 0 <= cy < h:
                    idx = cy * w + cx
                    v = grid[idx] + log_free
                    grid[idx] = v if v > log_min else log_min

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                cx += sx
            if e2 < dx:
                err += dx
                cy += sy

    # ── Scan matching (correlative) ───────────────────────────────────

    def _scan_match(self, scan: LaserScan,
                    init_x: float, init_y: float, init_yaw: float):
        """Correlative scan-to-map matching.

        Evaluates a grid of candidate poses around the initial estimate.
        Returns (best_x, best_y, best_yaw, best_score).
        """
        # Pre-extract valid scan angles + ranges (downsampled)
        angles = []
        ranges = []
        angle = scan.angle_min
        for i, r in enumerate(scan.ranges):
            if i % self._downsample == 0:
                if self._min_range < r < self._max_range \
                        and not math.isinf(r) and not math.isnan(r):
                    angles.append(angle)
                    ranges.append(r)
            angle += scan.angle_increment

        if not ranges:
            return init_x, init_y, init_yaw, 0.0

        n_xy = max(1, int(self._search_xy / self._step_xy))
        n_yaw = max(1, int(self._search_yaw / self._step_yaw))

        best_score = float('-inf')
        best = (init_x, init_y, init_yaw)

        grid = self._grid
        w = self._width
        h = self._height
        ox = self._origin_x
        oy = self._origin_y
        res = self._resolution

        for dxi in range(-n_xy, n_xy + 1):
            cx = init_x + dxi * self._step_xy
            for dyi in range(-n_xy, n_xy + 1):
                cy = init_y + dyi * self._step_xy
                for dyawi in range(-n_yaw, n_yaw + 1):
                    cyaw = init_yaw + dyawi * self._step_yaw

                    # Score: sum of log-odds at projected scan endpoints
                    score = 0.0
                    for k in range(len(ranges)):
                        beam_a = cyaw + angles[k]
                        ex = cx + ranges[k] * math.cos(beam_a)
                        ey = cy + ranges[k] * math.sin(beam_a)
                        gx = int((ex - ox) / res)
                        gy = int((ey - oy) / res)
                        if 0 <= gx < w and 0 <= gy < h:
                            score += grid[gy * w + gx]

                    if score > best_score:
                        best_score = score
                        best = (cx, cy, cyaw)

        return best[0], best[1], best[2], best_score

    # ── Publishing ────────────────────────────────────────────────────

    def _publish_map(self) -> None:
        """Publish the occupancy grid as nav_msgs/OccupancyGrid."""
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame

        msg.info.resolution = float(self._resolution)
        msg.info.width = self._width
        msg.info.height = self._height
        msg.info.origin.position.x = self._origin_x
        msg.info.origin.position.y = self._origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        thresh = self._unknown_thresh
        with self._lock:
            # Convert log-odds → occupancy value  (-1 unknown, 0–100 probability)
            data = []
            for lo in self._grid:
                if abs(lo) < thresh:
                    data.append(-1)
                else:
                    # Probability = 1 / (1 + e^{-lo})  →  scale to 0–100
                    p = 1.0 / (1.0 + math.exp(-lo))
                    data.append(int(p * 100.0))

        msg.data = data
        self._map_pub.publish(msg)

    def _publish_tf(self) -> None:
        """Publish TF map → odom (drift-correction transform)."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._map_frame
        t.child_frame_id = self._odom_frame
        t.transform.translation.x = self._corr_x
        t.transform.translation.y = self._corr_y
        t.transform.translation.z = 0.0
        t.transform.rotation = quaternion_from_yaw(self._corr_yaw)
        self._tf_broadcaster.sendTransform(t)

    def _publish_status(self) -> None:
        b = Bool()
        b.data = True
        self._active_pub.publish(b)

    # ── Service handlers ──────────────────────────────────────────────

    def _srv_reset(self, req, resp):
        """Clear the occupancy grid and reset the correction transform."""
        with self._lock:
            for i in range(len(self._grid)):
                self._grid[i] = 0.0
        self._corr_x = 0.0
        self._corr_y = 0.0
        self._corr_yaw = 0.0
        self._scan_count = 0
        resp.success = True
        resp.message = 'SLAM map cleared, correction reset'
        self.get_logger().info('[slam] map reset')
        return resp

    # ── Util ──────────────────────────────────────────────────────────

    @staticmethod
    def _clamp(v: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, v))


# ── Entry point ───────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = LidarSlamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
