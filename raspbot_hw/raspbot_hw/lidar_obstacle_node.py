"""
LiDAR-based 360° obstacle avoidance for Raspbot V2.

Processes LaserScan data into angular zone distances for safe navigation.
Publishes a ``sensor_msgs/Range`` on the front zone for seamless integration
with the motor driver's existing collision failsafe, and comprehensive zone
data for the web UI.

Angular zones (configurable, degrees from robot forward):
  - front:   ±front_half_angle  (default ±30°)
  - left:    +30° to +150°
  - right:   −30° to −150°
  - rear:    ±(180° − front_half_angle)

Subscribes:
  /scan                     (sensor_msgs/LaserScan)

Publishes:
  /lidar/front_range        (sensor_msgs/Range)    — min distance in front zone
  /lidar/left_range         (sensor_msgs/Range)    — min distance in left zone
  /lidar/right_range        (sensor_msgs/Range)    — min distance in right zone
  /lidar/rear_range         (sensor_msgs/Range)    — min distance in rear zone
  /lidar_obstacle/active    (std_msgs/Bool)        — true when front zone blocked
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult

from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Bool

# ── helpers ──────────────────────────────────────────────────────────

DEG_TO_RAD = math.pi / 180.0


def normalize_angle(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


# ── LiDAR Obstacle Node ─────────────────────────────────────────────

class LidarObstacleNode(Node):
    """360° LiDAR obstacle detection with zone-based avoidance."""

    _TUNABLE_PARAMS = {
        'front_half_angle_deg': '_front_half_angle',
        'front_stop_distance': '_front_stop_dist',
        'front_slow_distance': '_front_slow_dist',
        'side_stop_distance': '_side_stop_dist',
        'side_slow_distance': '_side_slow_dist',
        'rear_stop_distance': '_rear_stop_dist',
        'rear_slow_distance': '_rear_slow_dist',
        'lidar_yaw_offset': '_yaw_offset',
    }

    def __init__(self):
        super().__init__('lidar_obstacle')

        # ── Declare parameters ────────────────────────────────────────
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('publish_hz', 10.0)

        # Zone definitions (angles in degrees, measured from forward)
        self.declare_parameter('front_half_angle_deg', 30.0)     # front: ±30°
        self.declare_parameter('side_angle_start_deg', 30.0)     # sides: 30°–150°
        self.declare_parameter('side_angle_end_deg', 150.0)

        # Distance thresholds
        self.declare_parameter('front_stop_distance', 0.20)      # hard stop (m)
        self.declare_parameter('front_slow_distance', 0.50)      # slow-down start (m)
        self.declare_parameter('side_stop_distance', 0.15)
        self.declare_parameter('side_slow_distance', 0.30)
        self.declare_parameter('rear_stop_distance', 0.15)
        self.declare_parameter('rear_slow_distance', 0.30)

        # Range message parameters
        self.declare_parameter('min_range', 0.05)
        self.declare_parameter('max_range', 12.0)
        self.declare_parameter('frame_id', 'laser_frame')

        # Angular offset: LiDAR forward vs robot forward (radians, set by calibration)
        self.declare_parameter('lidar_yaw_offset', 0.0)

        # ── Load parameters ───────────────────────────────────────────
        scan_topic = str(self.get_parameter('scan_topic').value)
        publish_hz = float(self.get_parameter('publish_hz').value)

        self._front_half_angle = float(
            self.get_parameter('front_half_angle_deg').value) * DEG_TO_RAD
        self._side_start = float(
            self.get_parameter('side_angle_start_deg').value) * DEG_TO_RAD
        self._side_end = float(
            self.get_parameter('side_angle_end_deg').value) * DEG_TO_RAD

        self._front_stop_dist = float(self.get_parameter('front_stop_distance').value)
        self._front_slow_dist = float(self.get_parameter('front_slow_distance').value)
        self._side_stop_dist = float(self.get_parameter('side_stop_distance').value)
        self._side_slow_dist = float(self.get_parameter('side_slow_distance').value)
        self._rear_stop_dist = float(self.get_parameter('rear_stop_distance').value)
        self._rear_slow_dist = float(self.get_parameter('rear_slow_distance').value)

        self._min_range = float(self.get_parameter('min_range').value)
        self._max_range = float(self.get_parameter('max_range').value)
        self._frame_id = str(self.get_parameter('frame_id').value)

        self._yaw_offset = float(self.get_parameter('lidar_yaw_offset').value)

        # ── Zone state ────────────────────────────────────────────────
        self._front_min = float('inf')
        self._left_min = float('inf')
        self._right_min = float('inf')
        self._rear_min = float('inf')
        self._front_blocked = False  # True when front zone ≤ stop distance
        self._last_scan_time = 0.0

        # ── Publishers ────────────────────────────────────────────────
        self._front_pub = self.create_publisher(Range, 'lidar/front_range', 10)
        self._left_pub = self.create_publisher(Range, 'lidar/left_range', 10)
        self._right_pub = self.create_publisher(Range, 'lidar/right_range', 10)
        self._rear_pub = self.create_publisher(Range, 'lidar/rear_range', 10)
        self._active_pub = self.create_publisher(Bool, 'lidar_obstacle/active', 10)

        # ── Subscriber ────────────────────────────────────────────────
        self._scan_sub = self.create_subscription(
            LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data
        )

        # ── Publish timer ─────────────────────────────────────────────
        period = 1.0 / max(publish_hz, 1.0)
        self._timer = self.create_timer(period, self._publish)

        # Live-tuning
        self.add_on_set_parameters_callback(self._on_param_change)

        self.get_logger().info(
            f'LiDAR obstacle avoidance started — '
            f'front ±{self._front_half_angle / DEG_TO_RAD:.0f}°, '
            f'stop={self._front_stop_dist:.2f}m, '
            f'slow={self._front_slow_dist:.2f}m'
        )

    # ── Parameter change ──────────────────────────────────────────────

    def _on_param_change(self, params) -> SetParametersResult:
        for p in params:
            attr = self._TUNABLE_PARAMS.get(p.name)
            if attr:
                val = p.value
                # Convert degree params to radians internally
                if p.name == 'front_half_angle_deg':
                    val = float(val) * DEG_TO_RAD
                setattr(self, attr, val)
                self.get_logger().info(f'[lidar_obstacle] param {p.name} → {p.value}')
        return SetParametersResult(successful=True)

    # ── Scan callback ─────────────────────────────────────────────────

    def _on_scan(self, msg: LaserScan) -> None:
        """Process LaserScan into zone minimum distances."""
        front_min = float('inf')
        left_min = float('inf')
        right_min = float('inf')
        rear_min = float('inf')

        front_half = self._front_half_angle
        side_start = self._side_start
        side_end = self._side_end
        yaw_offset = self._yaw_offset

        angle = msg.angle_min
        for r in msg.ranges:
            if math.isinf(r) or math.isnan(r) or r < self._min_range:
                angle += msg.angle_increment
                continue

            # Apply yaw offset: rotate scan into robot frame
            a = normalize_angle(angle - yaw_offset)
            abs_a = abs(a)

            if abs_a <= front_half:
                # Front zone
                if r < front_min:
                    front_min = r
            elif abs_a >= math.pi - front_half:
                # Rear zone
                if r < rear_min:
                    rear_min = r
            elif a > 0 and side_start <= a <= side_end:
                # Left zone
                if r < left_min:
                    left_min = r
            elif a < 0 and side_start <= abs_a <= side_end:
                # Right zone
                if r < right_min:
                    right_min = r

            angle += msg.angle_increment

        self._front_min = front_min
        self._left_min = left_min
        self._right_min = right_min
        self._rear_min = rear_min
        self._front_blocked = (front_min <= self._front_stop_dist)
        self._last_scan_time = time.monotonic()

    # ── Publish ───────────────────────────────────────────────────────

    def _publish(self) -> None:
        """Publish zone ranges and obstacle status."""
        # Don't publish if no scan data in last 2 seconds
        if (time.monotonic() - self._last_scan_time) > 2.0:
            return

        stamp = self.get_clock().now().to_msg()

        # Front range (for motor driver collision failsafe integration)
        self._pub_range(self._front_pub, stamp, self._front_min, math.pi / 3.0)
        # Side & rear ranges
        self._pub_range(self._left_pub, stamp, self._left_min, math.pi / 1.5)
        self._pub_range(self._right_pub, stamp, self._right_min, math.pi / 1.5)
        self._pub_range(self._rear_pub, stamp, self._rear_min, math.pi / 3.0)

        # Obstacle active status
        b = Bool()
        b.data = self._front_blocked
        self._active_pub.publish(b)

    def _pub_range(self, pub, stamp, distance: float, fov: float) -> None:
        """Publish a sensor_msgs/Range message."""
        msg = Range()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.radiation_type = Range.INFRARED  # closest to LiDAR
        msg.field_of_view = fov
        msg.min_range = self._min_range
        msg.max_range = self._max_range
        if math.isinf(distance):
            msg.range = self._max_range
        else:
            msg.range = max(self._min_range, min(distance, self._max_range))
        pub.publish(msg)


# ── Entry point ───────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
