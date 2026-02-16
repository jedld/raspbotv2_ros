import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Bool, Float64, Int32

from .i2c_car import I2CCar

DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver')

        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_addr', 0x16)
        self.declare_parameter('i2c_protocol', 'auto')
        self.declare_parameter('i2c_required', True)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('max_pwm', 100)
        self.declare_parameter('min_pwm', 0)  # 0 disables; helps overcome static friction
        self.declare_parameter('max_linear_velocity', 0.35)
        self.declare_parameter('wheel_separation', 0.18)
        self.declare_parameter('write_hz', 20.0)  # I2C write rate (lower reduces bus load)
        self.declare_parameter('drive_mode', 'differential')  # differential|mecanum
        self.declare_parameter('wheelbase', 0.18)  # m (front-to-rear)
        self.declare_parameter('track_width', 0.18)  # m (left-to-right)

        # pi5 protocol motor IDs (defaults match vendor docs: 0=L1,1=L2,2=R1,3=R2)
        # We assume L1=front_left, L2=rear_left, R1=front_right, R2=rear_right.
        self.declare_parameter('motor_id_fl', 0)
        self.declare_parameter('motor_id_rl', 1)
        self.declare_parameter('motor_id_fr', 2)
        self.declare_parameter('motor_id_rr', 3)
        self.declare_parameter('invert_fl', False)
        self.declare_parameter('invert_fr', False)
        self.declare_parameter('invert_rl', False)
        self.declare_parameter('invert_rr', False)
        self.declare_parameter('cmd_vel_timeout_sec', 0.5)
        self.declare_parameter('idle_stop_period_sec', 1.0)
        # Optional: brief boost when starting from rest to overcome static friction.
        # 0 disables.
        self.declare_parameter('startup_kick_pwm', 0)
        self.declare_parameter('startup_kick_duration_sec', 0.15)

        # ── Gyro heading-hold parameters ──────────────────────────────
        self.declare_parameter('gyro_heading_hold_enable', True)
        # Proportional gain: PWM correction per (°/s of yaw drift).
        # Positive drift → slow down right wheels / speed up left wheels.
        self.declare_parameter('gyro_heading_hold_gain', 2.0)
        # Max PWM correction applied by heading-hold (prevents wild swings)
        self.declare_parameter('gyro_heading_hold_max_correction', 30.0)
        # IMU topic to subscribe to for gyro data
        self.declare_parameter('imu_topic', 'imu/data')
        # Dead-band: don't correct if cmd_vel angular.z is above this (rad/s)
        # i.e. heading-hold only engages when the user is driving "straight"
        self.declare_parameter('gyro_heading_hold_deadband_rad', 0.05)

        # Turning gate for heading-hold: disable heading-hold when any turn command is present,
        # and keep it disabled briefly after turning to avoid fighting rotate-to-angle near target.
        self.declare_parameter('gyro_heading_hold_turn_disable_rad', 0.005)
        self.declare_parameter('gyro_heading_hold_turn_cooldown_sec', 0.25)

        # Extended PID gains (upgrade P-only heading-hold to full PID)
        self.declare_parameter('gyro_heading_hold_ki', 1.5)
        self.declare_parameter('gyro_heading_hold_kd', 0.8)
        self.declare_parameter('gyro_heading_hold_integral_max', 20.0)
        # Strafe-specific stabilizers
        # Small deadband on the PWM correction during pure strafe helps prevent
        # tiny gyro-noise-driven corrections from modulating lateral speed.
        self.declare_parameter('gyro_heading_hold_strafe_correction_deadband_pwm', 0.0)
        # Optional output smoothing during pure strafe (PWM per second). 0 disables.
        self.declare_parameter('pwm_slew_rate_strafe', 0.0)
        self.declare_parameter('imu_yaw_topic', 'imu/yaw_deg')
        # Correction sign: set to -1.0 if the heading-hold makes drift WORSE.
        # This flips the entire PID output without touching IMU axis config.
        self.declare_parameter('heading_hold_sign', 1.0)
        # Diagnostic log interval (seconds). 0 disables.
        self.declare_parameter('heading_hold_diag_interval_sec', 2.0)

        # Adaptive motor trim: learns L/R asymmetry from sustained corrections
        self.declare_parameter('motor_trim_adapt_enable', True)
        self.declare_parameter('motor_trim_adapt_rate', 0.10)  # PWM/s
        self.declare_parameter('motor_trim_max', 15.0)

        # Per-motor manual trim offsets (added to PWM; positive = more PWM)
        self.declare_parameter('trim_fl', 0.0)
        self.declare_parameter('trim_fr', 0.0)
        self.declare_parameter('trim_rl', 0.0)
        self.declare_parameter('trim_rr', 0.0)

        # Lateral drift correction via accelerometer (mecanum only)
        self.declare_parameter('lateral_drift_correction_enable', False)
        self.declare_parameter('lateral_drift_gain', 3.0)
        self.declare_parameter('lateral_drift_max_correction', 8.0)
        self.declare_parameter('lateral_drift_alpha', 0.05)

        # ── Collision failsafe (ultrasonic) ───────────────────────────
        # Enabled by default.  Blocks forward motion when ultrasonic range
        # drops below collision_stop_distance_m.  Between stop and slow
        # distances, forward speed is linearly scaled down.
        self.declare_parameter('collision_failsafe_enable', True)
        self.declare_parameter('collision_stop_distance_m', 0.15)   # hard stop
        self.declare_parameter('collision_slow_distance_m', 0.35)   # begin slowdown
        self.declare_parameter('collision_ultrasonic_topic', 'ultrasonic/range')
        self.declare_parameter('collision_failsafe_timeout_sec', 1.0)  # stale data timeout

        # ── LiDAR front-zone obstacle integration ────────────────────
        # Complements ultrasonic with 360° LiDAR data from lidar_obstacle_node.
        # Uses the minimum of ultrasonic and lidar front range for collision checks.
        self.declare_parameter('lidar_front_range_topic', 'lidar/front_range')
        self.declare_parameter('lidar_collision_enable', True)
        self.declare_parameter('lidar_collision_timeout_sec', 1.0)

        # ── Backward collision via LiDAR rear zone ──────────────────
        # Blocks backward motion when LiDAR rear range is too short.
        self.declare_parameter('backward_collision_enable', True)
        self.declare_parameter('backward_stop_distance_m', 0.15)
        self.declare_parameter('backward_slow_distance_m', 0.30)
        self.declare_parameter('lidar_rear_range_topic', 'lidar/rear_range')

        # ── Lateral collision via LiDAR side zones ──────────────────
        # Blocks strafe motion toward an obstacle detected by the LiDAR
        # left / right zone (mecanum mode only).
        self.declare_parameter('lateral_collision_enable', True)
        self.declare_parameter('lateral_stop_distance_m', 0.12)
        self.declare_parameter('lateral_slow_distance_m', 0.25)
        self.declare_parameter('lidar_left_range_topic', 'lidar/left_range')
        self.declare_parameter('lidar_right_range_topic', 'lidar/right_range')

        # ── Cliff / edge failsafe (line-tracker) ───────────────────────
        # Uses the downward-facing IR reflectance sensors (line trackers) to
        # detect when the floor disappears (stairs, table edge, hole).
        # Sensor value 1 = no floor / edge detected, 0 = floor present.
        # Blocks forward motion when ANY sensor sees no floor.
        self.declare_parameter('cliff_failsafe_enable', True)
        self.declare_parameter('cliff_tracking_topic', 'tracking/state')
        self.declare_parameter('cliff_failsafe_timeout_sec', 1.0)  # stale data timeout
        # Bit mask: which sensors to use for cliff detection.
        # Default 0b1111 = all four (L1, L2, R1, R2).
        # Set to e.g. 0b0101 (5) to use only L1 + R1 (outer sensors).
        self.declare_parameter('cliff_sensor_mask', 15)

        i2c_bus = int(self.get_parameter('i2c_bus').value)
        i2c_addr = int(self.get_parameter('i2c_addr').value)
        i2c_protocol = str(self.get_parameter('i2c_protocol').value)
        i2c_required = bool(self.get_parameter('i2c_required').value)
        dry_run = bool(self.get_parameter('dry_run').value)

        self._max_pwm = int(self.get_parameter('max_pwm').value)
        self._min_pwm = int(self.get_parameter('min_pwm').value)
        self._max_linear = float(self.get_parameter('max_linear_velocity').value)
        self._wheel_sep = float(self.get_parameter('wheel_separation').value)

        self._write_hz = float(self.get_parameter('write_hz').value)
        self._min_write_period = 1.0 / max(self._write_hz, 1e-3)

        self._drive_mode = str(self.get_parameter('drive_mode').value).strip().lower()
        self._wheelbase = float(self.get_parameter('wheelbase').value)
        self._track_width = float(self.get_parameter('track_width').value)

        self._motor_id_fl = int(self.get_parameter('motor_id_fl').value)
        self._motor_id_fr = int(self.get_parameter('motor_id_fr').value)
        self._motor_id_rl = int(self.get_parameter('motor_id_rl').value)
        self._motor_id_rr = int(self.get_parameter('motor_id_rr').value)

        self._invert_fl = bool(self.get_parameter('invert_fl').value)
        self._invert_fr = bool(self.get_parameter('invert_fr').value)
        self._invert_rl = bool(self.get_parameter('invert_rl').value)
        self._invert_rr = bool(self.get_parameter('invert_rr').value)
        self._timeout = float(self.get_parameter('cmd_vel_timeout_sec').value)
        self._idle_stop_period = float(self.get_parameter('idle_stop_period_sec').value)

        self._startup_kick_pwm = int(self.get_parameter('startup_kick_pwm').value)
        self._startup_kick_duration = float(self.get_parameter('startup_kick_duration_sec').value)
        self._kick_until_time = 0.0

        # ── Gyro heading-hold state ──────────────────────────────────
        self._heading_hold_enable = bool(self.get_parameter('gyro_heading_hold_enable').value)
        self._heading_hold_gain = float(self.get_parameter('gyro_heading_hold_gain').value)
        self._heading_hold_max = float(self.get_parameter('gyro_heading_hold_max_correction').value)
        self._heading_hold_deadband = float(self.get_parameter('gyro_heading_hold_deadband_rad').value)
        self._heading_hold_turn_disable_rad = float(self.get_parameter('gyro_heading_hold_turn_disable_rad').value)
        self._heading_hold_turn_cooldown_sec = float(self.get_parameter('gyro_heading_hold_turn_cooldown_sec').value)
        self._imu_topic = str(self.get_parameter('imu_topic').value)
        self._gyro_yaw_rate = 0.0  # latest gyro angular_velocity.z (rad/s)
        self._gyro_correction = 0.0  # current correction PWM delta
        self._cmd_angular_z = 0.0  # latest commanded angular.z from cmd_vel
        self._cmd_vx = 0.0  # latest commanded linear.x from cmd_vel
        self._cmd_vy = 0.0  # latest commanded linear.y from cmd_vel
        self._last_turn_cmd_time = 0.0  # time.time() when we last saw a non-trivial angular.z command
        self._imu_calibrated = False  # gated until imu/calibrated is True
        self._imu_has_quaternion = False  # True when BNO055 fused orientation is available
        # Drive direction tracking: detect transitions between forward/strafe/diagonal
        # to reset PID state that is mode-specific (integral, adapted trim).
        self._prev_drive_dir = 'stopped'  # 'stopped' | 'forward' | 'strafe' | 'diagonal'

        # PID extended state
        self._heading_hold_ki = float(self.get_parameter('gyro_heading_hold_ki').value)
        self._heading_hold_kd = float(self.get_parameter('gyro_heading_hold_kd').value)
        self._heading_integral_max = float(self.get_parameter('gyro_heading_hold_integral_max').value)
        self._heading_hold_strafe_corr_deadband_pwm = float(
            self.get_parameter('gyro_heading_hold_strafe_correction_deadband_pwm').value
        )
        self._pwm_slew_rate_strafe = float(self.get_parameter('pwm_slew_rate_strafe').value)
        self._heading_hold_sign = float(self.get_parameter('heading_hold_sign').value)
        self._heading_integral = 0.0
        self._heading_target_deg = None  # captured on transition to straight driving
        self._imu_yaw_deg = 0.0
        self._last_heading_hold_time = None
        # Diagnostic logging
        self._diag_interval = float(self.get_parameter('heading_hold_diag_interval_sec').value)
        self._last_diag_time = 0.0
        self._heading_error_deg = 0.0  # stashed for diagnostics

        # Adaptive motor trim
        self._trim_adapt_enable = bool(self.get_parameter('motor_trim_adapt_enable').value)
        self._trim_adapt_rate = float(self.get_parameter('motor_trim_adapt_rate').value)
        self._trim_max = float(self.get_parameter('motor_trim_max').value)
        self._adapted_trim = 0.0  # learned L-R bias (pos = left needs more PWM)

        # Per-motor manual trim
        self._trim_fl = float(self.get_parameter('trim_fl').value)
        self._trim_fr = float(self.get_parameter('trim_fr').value)
        self._trim_rl = float(self.get_parameter('trim_rl').value)
        self._trim_rr = float(self.get_parameter('trim_rr').value)

        # Lateral drift correction
        self._lateral_drift_enable = bool(self.get_parameter('lateral_drift_correction_enable').value)
        self._lateral_drift_gain = float(self.get_parameter('lateral_drift_gain').value)
        self._lateral_drift_max = float(self.get_parameter('lateral_drift_max_correction').value)
        self._lateral_drift_alpha = float(self.get_parameter('lateral_drift_alpha').value)
        self._lateral_accel_filtered = 0.0
        self._lateral_correction = 0.0

        # ── Collision failsafe state ──────────────────────────────────
        self._collision_enable = bool(self.get_parameter('collision_failsafe_enable').value)
        self._collision_stop_dist = float(self.get_parameter('collision_stop_distance_m').value)
        self._collision_slow_dist = float(self.get_parameter('collision_slow_distance_m').value)
        self._collision_topic = str(self.get_parameter('collision_ultrasonic_topic').value)
        self._collision_timeout = float(self.get_parameter('collision_failsafe_timeout_sec').value)
        self._collision_distance = float('inf')  # latest ultrasonic reading (m)
        self._collision_last_time = 0.0
        self._collision_active = False  # True when forward motion is being blocked
        self._collision_logged = False  # avoid log spam

        # ── LiDAR front-zone collision state ──────────────────────────
        self._lidar_collision_enable = bool(self.get_parameter('lidar_collision_enable').value)
        self._lidar_front_topic = str(self.get_parameter('lidar_front_range_topic').value)
        self._lidar_collision_timeout = float(self.get_parameter('lidar_collision_timeout_sec').value)
        self._lidar_front_distance = float('inf')
        self._lidar_front_last_time = 0.0

        # ── Backward collision state ──────────────────────────────────
        self._backward_collision_enable = bool(self.get_parameter('backward_collision_enable').value)
        self._backward_stop_dist = float(self.get_parameter('backward_stop_distance_m').value)
        self._backward_slow_dist = float(self.get_parameter('backward_slow_distance_m').value)
        self._lidar_rear_topic = str(self.get_parameter('lidar_rear_range_topic').value)
        self._lidar_rear_distance = float('inf')
        self._lidar_rear_last_time = 0.0
        self._backward_active = False
        self._backward_logged = False

        # ── Lateral collision state ───────────────────────────────────
        self._lateral_collision_enable = bool(self.get_parameter('lateral_collision_enable').value)
        self._lateral_stop_dist = float(self.get_parameter('lateral_stop_distance_m').value)
        self._lateral_slow_dist = float(self.get_parameter('lateral_slow_distance_m').value)
        self._lidar_left_topic = str(self.get_parameter('lidar_left_range_topic').value)
        self._lidar_right_topic = str(self.get_parameter('lidar_right_range_topic').value)
        self._lidar_left_distance = float('inf')
        self._lidar_right_distance = float('inf')
        self._lidar_left_last_time = 0.0
        self._lidar_right_last_time = 0.0
        self._lateral_active = False
        self._lateral_logged = False

        # ── Cliff failsafe state ──────────────────────────────────────
        self._cliff_enable = bool(self.get_parameter('cliff_failsafe_enable').value)
        self._cliff_topic = str(self.get_parameter('cliff_tracking_topic').value)
        self._cliff_timeout = float(self.get_parameter('cliff_failsafe_timeout_sec').value)
        self._cliff_sensor_mask = int(self.get_parameter('cliff_sensor_mask').value) & 0x0F
        self._cliff_state = 0  # latest tracking/state (4-bit)
        self._cliff_last_time = 0.0
        self._cliff_active = False
        self._cliff_logged = False

        self._car = I2CCar(i2c_bus=i2c_bus, i2c_addr=i2c_addr, dry_run=dry_run, protocol=i2c_protocol)

        self._last_cmd_time = None
        self._last_idle_stop_time = 0.0
        self._last_write_time = 0.0
        self._zero_stop_sent = False  # guard against repeated _car.stop() I2C writes
        self._left_pwm = 0.0
        self._right_pwm = 0.0

        self._fl_pwm = 0.0
        self._fr_pwm = 0.0
        self._rl_pwm = 0.0
        self._rr_pwm = 0.0

        # Last PWMs sent to hardware (for optional slew limiting)
        self._sent_fl_pwm = 0.0
        self._sent_fr_pwm = 0.0
        self._sent_rl_pwm = 0.0
        self._sent_rr_pwm = 0.0

        self._prev_nonzero = False

        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)
        self._timer = self.create_timer(0.02, self._tick)  # 50Hz

        # ── Collision failsafe subscription & publisher ───────────────
        self._collision_enable_sub = self.create_subscription(
            Bool, 'collision_failsafe/enable', self._on_collision_enable, 10
        )
        if self._collision_enable:
            self.create_subscription(
                Range, self._collision_topic, self._on_ultrasonic, 10
            )
            self._collision_active_pub = self.create_publisher(
                Bool, 'collision_failsafe/active', 10
            )
            self.get_logger().info(
                f'Collision failsafe ENABLED (stop={self._collision_stop_dist:.2f}m, '
                f'slow={self._collision_slow_dist:.2f}m, topic={self._collision_topic})'
            )
        else:
            self._collision_active_pub = self.create_publisher(
                Bool, 'collision_failsafe/active', 10
            )
            self.create_subscription(
                Range, self._collision_topic, self._on_ultrasonic, 10
            )
            self.get_logger().info('Collision failsafe DISABLED (can be enabled via topic or param)')

        # ── LiDAR front-zone collision subscription ─────────────────
        self.create_subscription(
            Range, self._lidar_front_topic, self._on_lidar_front_range, 10
        )
        if self._lidar_collision_enable:
            self.get_logger().info(
                f'LiDAR collision integration ENABLED (topic={self._lidar_front_topic})'
            )

        # ── LiDAR rear/side collision subscriptions ─────────────────
        self.create_subscription(
            Range, self._lidar_rear_topic, self._on_lidar_rear_range, 10
        )
        self.create_subscription(
            Range, self._lidar_left_topic, self._on_lidar_left_range, 10
        )
        self.create_subscription(
            Range, self._lidar_right_topic, self._on_lidar_right_range, 10
        )
        if self._backward_collision_enable:
            self.get_logger().info(
                f'Backward collision ENABLED (stop={self._backward_stop_dist:.2f}m, '
                f'slow={self._backward_slow_dist:.2f}m, topic={self._lidar_rear_topic})'
            )
        if self._lateral_collision_enable:
            self.get_logger().info(
                f'Lateral collision ENABLED (stop={self._lateral_stop_dist:.2f}m, '
                f'slow={self._lateral_slow_dist:.2f}m)'
            )

        # ── Cliff failsafe subscription & publisher ───────────────────
        self._cliff_enable_sub = self.create_subscription(
            Bool, 'cliff_failsafe/enable', self._on_cliff_enable, 10
        )
        self._cliff_active_pub = self.create_publisher(
            Bool, 'cliff_failsafe/active', 10
        )
        self.create_subscription(
            Int32, self._cliff_topic, self._on_tracking_state, 10
        )
        if self._cliff_enable:
            self.get_logger().info(
                f'Cliff failsafe ENABLED (topic={self._cliff_topic}, '
                f'mask=0b{self._cliff_sensor_mask:04b})'
            )
        else:
            self.get_logger().info('Cliff failsafe DISABLED (can be enabled via topic or param)')

        # ── IMU subscriptions for heading-hold ────────────────────────
        if self._heading_hold_enable:
            self.create_subscription(Imu, self._imu_topic, self._on_imu, 10)
            self.create_subscription(Bool, 'imu/calibrated', self._on_imu_calibrated, 10)
            imu_yaw_topic = str(self.get_parameter('imu_yaw_topic').value)
            self.create_subscription(Float64, imu_yaw_topic, self._on_imu_yaw, 10)
            self.get_logger().info(
                f'PID heading-hold ENABLED (Kp={self._heading_hold_gain}, '
                f'Ki={self._heading_hold_ki}, Kd={self._heading_hold_kd}, '
                f'sign={self._heading_hold_sign:+.0f}, '
                f'max_corr={self._heading_hold_max}, deadband={self._heading_hold_deadband} rad/s). '
                f'Waiting for IMU calibration...'
            )
            if self._trim_adapt_enable:
                self.get_logger().info(
                    f'Adaptive motor trim ENABLED (rate={self._trim_adapt_rate} PWM/s, '
                    f'max={self._trim_max})'
                )
            if any(abs(t) > 0.01 for t in (self._trim_fl, self._trim_fr, self._trim_rl, self._trim_rr)):
                self.get_logger().info(
                    f'Manual motor trim: FL={self._trim_fl:+.1f} FR={self._trim_fr:+.1f} '
                    f'RL={self._trim_rl:+.1f} RR={self._trim_rr:+.1f}'
                )

        # Live-tuning: allow key parameters to be changed at runtime
        self.add_on_set_parameters_callback(self._on_param_change)

        if dry_run:
            self.get_logger().warn('Motor driver running in dry_run mode (no I2C writes).')
        else:
            try:
                self._car.stop()
            except Exception as e:
                msg = f'I2C motor controller not reachable (bus={i2c_bus}, addr=0x{i2c_addr:02x}): {e!r}'
                if i2c_required:
                    raise RuntimeError(msg) from e
                self.get_logger().error(msg)

        self.get_logger().info(
            f'I2C motor driver ready (bus={i2c_bus}, addr=0x{i2c_addr:02x}, max_pwm={self._max_pwm})'
        )

        if self._car.protocol == 'pi5':
            self.get_logger().info(
                'Motor mapping (pi5): '
                f'FL=id{self._motor_id_fl} inv={self._invert_fl}, '
                f'FR=id{self._motor_id_fr} inv={self._invert_fr}, '
                f'RL=id{self._motor_id_rl} inv={self._invert_rl}, '
                f'RR=id{self._motor_id_rr} inv={self._invert_rr}'
            )

        if self._drive_mode not in {'differential', 'mecanum'}:
            self.get_logger().warn(
                f"Unknown drive_mode '{self._drive_mode}', falling back to differential."
            )
            self._drive_mode = 'differential'

        if self._drive_mode == 'mecanum' and self._car.protocol != 'pi5':
            self.get_logger().warn(
                'drive_mode=mecanum requested but I2C protocol is not pi5; '
                'strafe will be ignored and controller will behave like differential.'
            )

    # ── Live parameter tuning ────────────────────────────────────────
    _TUNABLE_PARAMS = {
        'heading_hold_sign': '_heading_hold_sign',
        'gyro_heading_hold_gain': '_heading_hold_gain',
        'gyro_heading_hold_ki': '_heading_hold_ki',
        'gyro_heading_hold_kd': '_heading_hold_kd',
        'gyro_heading_hold_max_correction': '_heading_hold_max',
        'gyro_heading_hold_deadband_rad': '_heading_hold_deadband',
        'gyro_heading_hold_turn_disable_rad': '_heading_hold_turn_disable_rad',
        'gyro_heading_hold_turn_cooldown_sec': '_heading_hold_turn_cooldown_sec',
        'gyro_heading_hold_strafe_correction_deadband_pwm': '_heading_hold_strafe_corr_deadband_pwm',
        'pwm_slew_rate_strafe': '_pwm_slew_rate_strafe',
        'trim_fl': '_trim_fl',
        'trim_fr': '_trim_fr',
        'trim_rl': '_trim_rl',
        'trim_rr': '_trim_rr',
        'collision_stop_distance_m': '_collision_stop_dist',
        'collision_slow_distance_m': '_collision_slow_dist',
        'lateral_drift_gain': '_lateral_drift_gain',
        'lateral_drift_max_correction': '_lateral_drift_max',
        'lateral_drift_alpha': '_lateral_drift_alpha',
    }
    _TUNABLE_BOOL_PARAMS = {
        'collision_failsafe_enable': '_collision_enable',
        'cliff_failsafe_enable': '_cliff_enable',
        'lateral_drift_correction_enable': '_lateral_drift_enable',
    }

    def _on_param_change(self, params) -> SetParametersResult:
        for p in params:
            attr = self._TUNABLE_PARAMS.get(p.name)
            if attr is not None:
                setattr(self, attr, float(p.value))
                self.get_logger().info(f'[live-tune] {p.name} = {p.value}')
            bool_attr = self._TUNABLE_BOOL_PARAMS.get(p.name)
            if bool_attr is not None:
                setattr(self, bool_attr, bool(p.value))
                self.get_logger().info(f'[live-tune] {p.name} = {p.value}')
        return SetParametersResult(successful=True)

    def _on_imu_calibrated(self, msg: Bool) -> None:
        was = self._imu_calibrated
        self._imu_calibrated = msg.data
        if msg.data and not was:
            self.get_logger().info('IMU calibrated — PID heading-hold is now active')
        elif not msg.data and was:
            self.get_logger().warn('IMU calibration lost — heading-hold suspended')
            self._gyro_correction = 0.0
            self._heading_integral = 0.0
            self._heading_target_deg = None
            self._last_heading_hold_time = None
            self._lateral_accel_filtered = 0.0
            self._lateral_correction = 0.0

    def _on_imu_yaw(self, msg: Float64) -> None:
        """Receive integrated heading from IMU node."""
        self._imu_yaw_deg = msg.data

    def _on_imu(self, msg: Imu) -> None:
        """PID heading-hold + lateral drift correction using IMU data.

        When BNO055 is present, orientation_covariance[0] >= 0 and the
        linear_acceleration is already gravity-subtracted (true inertial).
        When LSM6DSOX-only, covariance[0] == -1 and linear_acceleration
        includes gravity — the lateral correction uses higher filtering.
        """
        self._gyro_yaw_rate = msg.angular_velocity.z  # rad/s
        accel_y = msg.linear_acceleration.y            # lateral m/s²
        # Track whether we have fused orientation (BNO055) for later use
        self._imu_has_quaternion = msg.orientation_covariance[0] >= 0.0

        # ── Gate: wait for calibration ────────────────────────────────
        if not self._imu_calibrated:
            self._gyro_correction = 0.0
            self._heading_integral = 0.0
            self._heading_target_deg = None
            self._lateral_accel_filtered = 0.0
            self._lateral_correction = 0.0
            return

        now = time.time()

        # ── Gate: user is intentionally turning ───────────────────────
        recently_turning = (now - self._last_turn_cmd_time) < self._heading_hold_turn_cooldown_sec
        if abs(self._cmd_angular_z) > self._heading_hold_turn_disable_rad or recently_turning:
            self._gyro_correction = 0.0
            # Preserve integral — motor asymmetry is valid across headings
            self._heading_target_deg = None
            self._last_heading_hold_time = None
            self._lateral_correction = 0.0
            return

        # ── Gate: robot isn't moving ──────────────────────────────────
        if not self._is_any_motion_commanded():
            self._gyro_correction = 0.0
            # Preserve integral — resumes instantly on next forward command
            self._heading_target_deg = None
            self._last_heading_hold_time = None
            self._lateral_correction = 0.0
            return

        # ── Capture heading target on transition to straight driving ──
        if self._heading_target_deg is None:
            self._heading_target_deg = self._imu_yaw_deg
            self._last_heading_hold_time = now
            self.get_logger().debug(
                f'Heading hold locked: target={self._heading_target_deg:.1f}°'
            )
            return

        dt = now - self._last_heading_hold_time if self._last_heading_hold_time else 0.02
        dt = min(dt, 0.1)  # safety cap
        self._last_heading_hold_time = now

        # ── Heading error (shortest path, degrees) ────────────────────
        error = self._heading_target_deg - self._imu_yaw_deg
        while error > 180.0:
            error -= 360.0
        while error < -180.0:
            error += 360.0

        # Pure strafe (vy only) behaves differently than forward driving:
        # - heading-hold is still useful to prevent yaw rotation
        # - but the forward-drive integral / trim can cause wobble
        is_strafing = abs(self._cmd_vy) > 0.01 and abs(self._cmd_vx) < 0.01
        pid_scale = 0.6 if is_strafing else 1.0

        # ── PID terms ─────────────────────────────────────────────────
        # P: proportional to heading position error
        p_term = self._heading_hold_gain * error * pid_scale

        # I: accumulates to learn constant motor asymmetry
        # Freeze the integrator during pure strafe to avoid oscillation from
        # mode-specific biases. Direction-change handling already resets it.
        if not is_strafing:
            self._heading_integral += error * dt
            self._heading_integral = clamp(
                self._heading_integral,
                -self._heading_integral_max,
                self._heading_integral_max,
            )
            i_term = self._heading_hold_ki * self._heading_integral
        else:
            i_term = 0.0

        # D: damps oscillation using gyro yaw rate
        yaw_rate_dps = self._gyro_yaw_rate * RAD_TO_DEG
        d_term = -self._heading_hold_kd * yaw_rate_dps * pid_scale

        # Apply correction sign (flip entire PID if IMU axis is inverted)
        raw_correction = self._heading_hold_sign * (p_term + i_term + d_term)
        if (
            is_strafing
            and self._heading_hold_strafe_corr_deadband_pwm > 0.0
            and abs(raw_correction) < self._heading_hold_strafe_corr_deadband_pwm
        ):
            raw_correction = 0.0
        self._gyro_correction = clamp(
            raw_correction, -self._heading_hold_max, self._heading_hold_max
        )
        self._heading_error_deg = error  # stash for diagnostics

        # ── Adaptive trim: slowly extract DC bias from sustained correction
        # Only adapt during forward/backward driving — strafe has different
        # motor roles and would pollute the forward-drive trim.
        total_corr = self._gyro_correction + self._adapted_trim
        if self._trim_adapt_enable and abs(total_corr) > 0.5 and not is_strafing:
            adapt_step = self._trim_adapt_rate * dt
            if total_corr > 0.5:
                self._adapted_trim = min(
                    self._adapted_trim + adapt_step, self._trim_max
                )
            elif total_corr < -0.5:
                self._adapted_trim = max(
                    self._adapted_trim - adapt_step, -self._trim_max
                )

        # ── Periodic diagnostic logging ───────────────────────────────
        if self._diag_interval > 0.0 and (now - self._last_diag_time) >= self._diag_interval:
            self._last_diag_time = now
            self.get_logger().info(
                f'[heading-hold] err={error:+.1f}° P={p_term:+.1f} '
                f'I={i_term:+.1f} D={d_term:+.1f} → corr={self._gyro_correction:+.1f} '
                f'trim={self._adapted_trim:+.1f} '
                f'yaw={self._imu_yaw_deg:.1f}° target={self._heading_target_deg:.1f}° '
                f'sign={self._heading_hold_sign:+.0f} '
                f'dir={self._prev_drive_dir} lat={self._lateral_correction:+.1f}'
            )

        # ── Lateral drift correction (accelerometer, mecanum) ─────────
        # When BNO055 is active, linear_acceleration is gravity-subtracted
        # so we can use a tighter threshold and more responsive filtering.
        # Lateral drift correction should not run during commanded strafe,
        # since accel_y will contain the intended lateral acceleration.
        is_forwardish = abs(self._cmd_vx) > 0.01
        if self._lateral_drift_enable and is_forwardish and not is_strafing:
            if getattr(self, '_imu_has_quaternion', False):
                # BNO055: gravity already removed → cleaner signal, respond faster
                alpha = max(self._lateral_drift_alpha, 0.25)
                threshold = 0.08  # m/s²
            else:
                # LSM6DSOX: raw accel includes gravity → heavier filtering
                alpha = min(self._lateral_drift_alpha, 0.05)
                threshold = 0.15  # m/s²
            self._lateral_accel_filtered = (
                alpha * accel_y + (1.0 - alpha) * self._lateral_accel_filtered
            )
            if abs(self._lateral_accel_filtered) > threshold:
                raw_lat = -self._lateral_drift_gain * self._lateral_accel_filtered
                self._lateral_correction = clamp(
                    raw_lat, -self._lateral_drift_max, self._lateral_drift_max
                )
            else:
                self._lateral_correction = 0.0
        else:
            self._lateral_accel_filtered = 0.0
            self._lateral_correction = 0.0

    # ── Collision failsafe ─────────────────────────────────────────
    def _on_ultrasonic(self, msg: Range) -> None:
        """Update obstacle distance from ultrasonic sensor."""
        d = msg.range
        if math.isnan(d) or math.isinf(d):
            self._collision_distance = float('inf')
        else:
            self._collision_distance = max(d, 0.0)
        self._collision_last_time = time.time()
    def _on_lidar_front_range(self, msg: Range) -> None:
        """Update front obstacle distance from LiDAR obstacle node."""
        d = msg.range
        if math.isnan(d) or math.isinf(d):
            self._lidar_front_distance = float('inf')
        else:
            self._lidar_front_distance = max(d, 0.0)
        self._lidar_front_last_time = time.time()

    def _on_lidar_rear_range(self, msg: Range) -> None:
        d = msg.range
        if math.isnan(d) or math.isinf(d):
            self._lidar_rear_distance = float('inf')
        else:
            self._lidar_rear_distance = max(d, 0.0)
        self._lidar_rear_last_time = time.time()

    def _on_lidar_left_range(self, msg: Range) -> None:
        d = msg.range
        if math.isnan(d) or math.isinf(d):
            self._lidar_left_distance = float('inf')
        else:
            self._lidar_left_distance = max(d, 0.0)
        self._lidar_left_last_time = time.time()

    def _on_lidar_right_range(self, msg: Range) -> None:
        d = msg.range
        if math.isnan(d) or math.isinf(d):
            self._lidar_right_distance = float('inf')
        else:
            self._lidar_right_distance = max(d, 0.0)
        self._lidar_right_last_time = time.time()

    def _on_collision_enable(self, msg: Bool) -> None:
        """Enable/disable collision failsafe via topic (from web UI)."""
        was = self._collision_enable
        self._collision_enable = msg.data
        if msg.data and not was:
            self.get_logger().info(
                f'Collision failsafe ENABLED via topic '
                f'(stop={self._collision_stop_dist:.2f}m, slow={self._collision_slow_dist:.2f}m)'
            )
        elif not msg.data and was:
            self.get_logger().info('Collision failsafe DISABLED via topic')
            self._collision_active = False
            self._collision_logged = False
            if self._collision_active_pub:
                b = Bool()
                b.data = False
                self._collision_active_pub.publish(b)
    def _apply_collision_failsafe(self, vx: float) -> float:
        """Clamp forward velocity based on ultrasonic distance.

        Returns modified vx.  Backward motion (vx < 0) is never blocked.
        """
        if not self._collision_enable:
            if self._collision_active:
                self._collision_active = False
                self._collision_logged = False
                if self._collision_active_pub:
                    b = Bool()
                    b.data = False
                    self._collision_active_pub.publish(b)
            return vx

        # If ultrasonic data is stale, don't block (sensor might be offline)
        if (time.time() - self._collision_last_time) > self._collision_timeout:
            if self._collision_active:
                self._collision_active = False
                self._collision_logged = False
                if self._collision_active_pub:
                    b = Bool()
                    b.data = False
                    self._collision_active_pub.publish(b)
            return vx

        # Only restrict forward motion (positive vx)
        if vx <= 0.0:
            was_active = self._collision_active
            self._collision_active = False
            if was_active:
                self._collision_logged = False
                if self._collision_active_pub:
                    b = Bool()
                    b.data = False
                    self._collision_active_pub.publish(b)
            return vx

        # Use minimum of ultrasonic and lidar front distance
        dist = self._collision_distance
        if self._lidar_collision_enable:
            lidar_stale = (time.time() - self._lidar_front_last_time) > self._lidar_collision_timeout
            if not lidar_stale:
                dist = min(dist, self._lidar_front_distance)
        stop_d = self._collision_stop_dist
        slow_d = self._collision_slow_dist

        if dist <= stop_d:
            # Hard stop — too close
            if not self._collision_logged:
                self.get_logger().warn(
                    f'[collision-failsafe] STOP — obstacle at {dist:.2f}m '
                    f'(threshold {stop_d:.2f}m)'
                )
                self._collision_logged = True
            if not self._collision_active:
                self._collision_active = True
                if self._collision_active_pub:
                    b = Bool()
                    b.data = True
                    self._collision_active_pub.publish(b)
            return 0.0
        elif dist < slow_d:
            # Linear slow-down zone
            scale = (dist - stop_d) / (slow_d - stop_d)
            if not self._collision_active:
                self._collision_active = True
                if self._collision_active_pub:
                    b = Bool()
                    b.data = True
                    self._collision_active_pub.publish(b)
            if not self._collision_logged:
                self.get_logger().info(
                    f'[collision-failsafe] slowing — obstacle at {dist:.2f}m '
                    f'(scale {scale:.0%})'
                )
                self._collision_logged = True
            return vx * scale
        else:
            # Clear
            was_active = self._collision_active
            self._collision_active = False
            if was_active:
                self._collision_logged = False
                self.get_logger().info('[collision-failsafe] clear')
                if self._collision_active_pub:
                    b = Bool()
                    b.data = False
                    self._collision_active_pub.publish(b)
            elif self._collision_logged:
                self._collision_logged = False
            return vx

    # ── Backward collision failsafe (LiDAR rear zone) ────────────────

    def _apply_backward_failsafe(self, vx: float) -> float:
        """Clamp backward velocity based on LiDAR rear range.

        Only restricts backward motion (vx < 0).
        """
        if not self._backward_collision_enable or vx >= 0.0:
            if self._backward_active:
                self._backward_active = False
                self._backward_logged = False
            return vx

        # Stale data — don't block
        if (time.time() - self._lidar_rear_last_time) > self._lidar_collision_timeout:
            if self._backward_active:
                self._backward_active = False
                self._backward_logged = False
            return vx

        dist = self._lidar_rear_distance
        stop_d = self._backward_stop_dist
        slow_d = self._backward_slow_dist

        if dist <= stop_d:
            if not self._backward_logged:
                self.get_logger().warn(
                    f'[backward-failsafe] STOP — rear obstacle at {dist:.2f}m'
                )
                self._backward_logged = True
            self._backward_active = True
            return 0.0
        elif dist < slow_d:
            scale = (dist - stop_d) / (slow_d - stop_d)
            self._backward_active = True
            if not self._backward_logged:
                self.get_logger().info(
                    f'[backward-failsafe] slowing — rear obstacle at {dist:.2f}m '
                    f'(scale {scale:.0%})'
                )
                self._backward_logged = True
            return vx * scale  # vx is negative, so this slows the backward speed
        else:
            if self._backward_active:
                self._backward_active = False
                self._backward_logged = False
                self.get_logger().info('[backward-failsafe] clear')
            return vx

    # ── Lateral collision failsafe (LiDAR left/right zones) ──────────

    def _apply_lateral_failsafe(self, vy: float) -> float:
        """Clamp lateral (strafe) velocity based on LiDAR side ranges.

        vy > 0 → moving left (check left range),
        vy < 0 → moving right (check right range).
        """
        if not self._lateral_collision_enable or abs(vy) < 1e-4:
            if self._lateral_active:
                self._lateral_active = False
                self._lateral_logged = False
            return vy

        if vy > 0.0:
            # Strafing left — check left range
            if (time.time() - self._lidar_left_last_time) > self._lidar_collision_timeout:
                return vy
            dist = self._lidar_left_distance
        else:
            # Strafing right — check right range
            if (time.time() - self._lidar_right_last_time) > self._lidar_collision_timeout:
                return vy
            dist = self._lidar_right_distance

        stop_d = self._lateral_stop_dist
        slow_d = self._lateral_slow_dist

        if dist <= stop_d:
            if not self._lateral_logged:
                side = 'left' if vy > 0 else 'right'
                self.get_logger().warn(
                    f'[lateral-failsafe] STOP — {side} obstacle at {dist:.2f}m'
                )
                self._lateral_logged = True
            self._lateral_active = True
            return 0.0
        elif dist < slow_d:
            scale = (dist - stop_d) / (slow_d - stop_d)
            self._lateral_active = True
            if not self._lateral_logged:
                side = 'left' if vy > 0 else 'right'
                self.get_logger().info(
                    f'[lateral-failsafe] slowing — {side} obstacle at {dist:.2f}m '
                    f'(scale {scale:.0%})'
                )
                self._lateral_logged = True
            return vy * scale
        else:
            if self._lateral_active:
                self._lateral_active = False
                self._lateral_logged = False
                self.get_logger().info('[lateral-failsafe] clear')
            return vy

    # ── Cliff / edge failsafe (line-tracker sensors) ─────────────────

    def _on_tracking_state(self, msg: Int32) -> None:
        """Receive 4-bit line-tracker state (bit=1 → sensor sees no floor)."""
        self._cliff_state = int(msg.data) & 0x0F
        self._cliff_last_time = time.time()

    def _on_cliff_enable(self, msg: Bool) -> None:
        """Enable/disable cliff failsafe via topic (from web UI)."""
        was = self._cliff_enable
        self._cliff_enable = msg.data
        if msg.data and not was:
            self.get_logger().info(
                f'Cliff failsafe ENABLED via topic '
                f'(mask=0b{self._cliff_sensor_mask:04b})'
            )
        elif not msg.data and was:
            self.get_logger().info('Cliff failsafe DISABLED via topic')
            self._cliff_active = False
            self._cliff_logged = False
            b = Bool()
            b.data = False
            self._cliff_active_pub.publish(b)

    def _apply_cliff_failsafe(self, vx: float) -> float:
        """Block forward motion when line-tracker sensors detect no floor.

        Returns modified vx.  Backward motion (vx < 0) is never blocked,
        allowing the robot to reverse away from an edge.
        """
        if not self._cliff_enable:
            if self._cliff_active:
                self._cliff_active = False
                self._cliff_logged = False
                b = Bool()
                b.data = False
                self._cliff_active_pub.publish(b)
            return vx

        # If tracking data is stale, don't block (sensor might be offline)
        if (time.time() - self._cliff_last_time) > self._cliff_timeout:
            if self._cliff_active:
                self._cliff_active = False
                self._cliff_logged = False
                b = Bool()
                b.data = False
                self._cliff_active_pub.publish(b)
            return vx

        # Only restrict forward motion (positive vx)
        if vx <= 0.0:
            was_active = self._cliff_active
            self._cliff_active = False
            if was_active:
                self._cliff_logged = False
                b = Bool()
                b.data = False
                self._cliff_active_pub.publish(b)
            return vx

        # Check if ANY masked sensor sees no floor (bit = 0 means no floor).
        # Sensors read 1 when floor is present, 0 when no floor / edge.
        missing = (~self._cliff_state) & self._cliff_sensor_mask
        if missing:
            if not self._cliff_logged:
                # Decode which sensors see no floor (bit=0)
                names = []
                if missing & 0x01:
                    names.append('L1')
                if missing & 0x02:
                    names.append('L2')
                if missing & 0x04:
                    names.append('R1')
                if missing & 0x08:
                    names.append('R2')
                self.get_logger().warn(
                    f'[cliff-failsafe] STOP — no floor detected by sensor(s): '
                    f'{", ".join(names)} (state=0b{self._cliff_state:04b})'
                )
                self._cliff_logged = True
            if not self._cliff_active:
                self._cliff_active = True
                b = Bool()
                b.data = True
                self._cliff_active_pub.publish(b)
            return 0.0
        else:
            # All masked sensors see floor — clear
            was_active = self._cliff_active
            self._cliff_active = False
            if was_active:
                self._cliff_logged = False
                self.get_logger().info('[cliff-failsafe] clear — floor detected')
                b = Bool()
                b.data = False
                self._cliff_active_pub.publish(b)
            elif self._cliff_logged:
                self._cliff_logged = False
            return vx

    def _on_cmd_vel(self, msg: Twist) -> None:
        vx = float(msg.linear.x)
        vy = float(getattr(msg.linear, 'y', 0.0))
        wz = float(msg.angular.z)
        self._cmd_angular_z = wz  # stash for heading-hold
        self._cmd_vx = vx
        self._cmd_vy = vy

        if abs(wz) > self._heading_hold_turn_disable_rad:
            self._last_turn_cmd_time = time.time()

        # ── Detect drive-direction transitions ────────────────────────
        # Reset heading-hold PID state when switching between forward and
        # strafe because the integral / adapted-trim from one mode is not
        # valid in the other.
        has_vx = abs(vx) > 0.01
        has_vy = abs(vy) > 0.01
        if has_vx and has_vy:
            new_dir = 'diagonal'
        elif has_vx:
            new_dir = 'forward'
        elif has_vy:
            new_dir = 'strafe'
        else:
            new_dir = 'stopped'

        if new_dir != self._prev_drive_dir and self._prev_drive_dir != 'stopped' and new_dir != 'stopped':
            # Direction changed while moving — reset PID to avoid carryover
            self._heading_target_deg = None
            self._last_heading_hold_time = None
            self._heading_integral = 0.0
            self.get_logger().debug(
                f'Drive direction change ({self._prev_drive_dir}→{new_dir}): '
                f'heading-hold PID reset'
            )
        self._prev_drive_dir = new_dir

        # ── Apply collision failsafe before kinematics ────────────────
        vx = self._apply_collision_failsafe(vx)

        # ── Apply backward collision failsafe (LiDAR rear) ───────────
        vx = self._apply_backward_failsafe(vx)

        # ── Apply cliff failsafe (line-tracker edge detection) ────────
        vx = self._apply_cliff_failsafe(vx)

        # ── Apply lateral collision failsafe (LiDAR left/right) ──────
        vy = self._apply_lateral_failsafe(vy)

        if self._max_linear <= 0.0:
            self._left_pwm = 0.0
            self._right_pwm = 0.0
            self._fl_pwm = 0.0
            self._fr_pwm = 0.0
            self._rl_pwm = 0.0
            self._rr_pwm = 0.0
            self._last_cmd_time = time.time()
            return

        if self._drive_mode == 'mecanum' and self._car.protocol == 'pi5':
            # ROS base_link: x forward, y left, z up.
            lx = 0.5 * float(self._wheelbase if self._wheelbase > 0.0 else self._wheel_sep)
            ly = 0.5 * float(self._track_width if self._track_width > 0.0 else self._wheel_sep)
            k = (lx + ly)

            # Wheel "linear" commands (m/s at wheel perimeter), scaled to max_linear_velocity.
            v_fl = vx - vy - (k * wz)
            v_fr = vx + vy + (k * wz)
            v_rl = vx + vy - (k * wz)
            v_rr = vx - vy + (k * wz)

            max_abs = max(abs(v_fl), abs(v_fr), abs(v_rl), abs(v_rr), 1e-6)
            if max_abs > self._max_linear:
                scale = self._max_linear / max_abs
                v_fl *= scale
                v_fr *= scale
                v_rl *= scale
                v_rr *= scale

            fl_pwm = v_fl / self._max_linear * self._max_pwm
            fr_pwm = v_fr / self._max_linear * self._max_pwm
            rl_pwm = v_rl / self._max_linear * self._max_pwm
            rr_pwm = v_rr / self._max_linear * self._max_pwm

            if self._invert_fl:
                fl_pwm *= -1.0
            if self._invert_fr:
                fr_pwm *= -1.0
            if self._invert_rl:
                rl_pwm *= -1.0
            if self._invert_rr:
                rr_pwm *= -1.0

            self._fl_pwm = clamp(fl_pwm, -self._max_pwm, self._max_pwm)
            self._fr_pwm = clamp(fr_pwm, -self._max_pwm, self._max_pwm)
            self._rl_pwm = clamp(rl_pwm, -self._max_pwm, self._max_pwm)
            self._rr_pwm = clamp(rr_pwm, -self._max_pwm, self._max_pwm)

            if self._min_pwm > 0:
                self._fl_pwm = self._apply_min_pwm(self._fl_pwm)
                self._fr_pwm = self._apply_min_pwm(self._fr_pwm)
                self._rl_pwm = self._apply_min_pwm(self._rl_pwm)
                self._rr_pwm = self._apply_min_pwm(self._rr_pwm)
        else:
            # Differential drive kinematics (vy ignored)
            v_l = vx - (wz * self._wheel_sep / 2.0)
            v_r = vx + (wz * self._wheel_sep / 2.0)

            self._left_pwm = clamp(v_l / self._max_linear * self._max_pwm, -self._max_pwm, self._max_pwm)
            self._right_pwm = clamp(v_r / self._max_linear * self._max_pwm, -self._max_pwm, self._max_pwm)

            if self._min_pwm > 0:
                self._left_pwm = self._apply_min_pwm(self._left_pwm)
                self._right_pwm = self._apply_min_pwm(self._right_pwm)

        self._last_cmd_time = time.time()

        # ── Immediately clear heading-hold when stopped ─────────────
        # This prevents the next _tick() from applying stale corrections
        # to zero PWMs before _on_imu has a chance to gate them.
        if not self._is_any_motion_commanded():
            self._gyro_correction = 0.0
            self._heading_target_deg = None
            self._last_heading_hold_time = None
            self._lateral_correction = 0.0

        # If we were stopped and are now commanded to move, start a brief kick window.
        now = self._last_cmd_time
        if self._startup_kick_pwm > 0 and self._startup_kick_duration > 0.0:
            if not self._prev_nonzero and self._is_any_motion_commanded():
                self._kick_until_time = now + self._startup_kick_duration
            self._prev_nonzero = self._is_any_motion_commanded()

    def _is_any_motion_commanded(self) -> bool:
        if self._drive_mode == 'mecanum' and self._car.protocol == 'pi5':
            return any(
                abs(x) > 1e-6
                for x in (self._fl_pwm, self._fr_pwm, self._rl_pwm, self._rr_pwm)
            )
        return abs(self._left_pwm) > 1e-6 or abs(self._right_pwm) > 1e-6

    def _apply_startup_kick(self, pwm: float) -> float:
        if self._startup_kick_pwm <= 0:
            return pwm
        if pwm == 0.0:
            return 0.0
        sign = 1.0 if pwm > 0.0 else -1.0
        mag = abs(pwm)
        if mag < float(self._startup_kick_pwm):
            return sign * float(self._startup_kick_pwm)
        return pwm

    def _apply_min_pwm(self, pwm: float) -> float:
        if pwm == 0.0:
            return 0.0
        sign = 1.0 if pwm > 0.0 else -1.0
        mag = abs(pwm)
        if 0.0 < mag < float(self._min_pwm):
            return sign * float(self._min_pwm)
        return pwm

    def _slew_limit_pwm(self, desired: float, previous: float, max_delta: float) -> float:
        if max_delta <= 0.0:
            return desired
        delta = desired - previous
        if delta > max_delta:
            return previous + max_delta
        if delta < -max_delta:
            return previous - max_delta
        return desired

    def _tick(self) -> None:
        now = time.time()
        if self._last_cmd_time is None:
            if self._idle_stop_period > 0.0 and (now - self._last_idle_stop_time) > self._idle_stop_period:
                try:
                    self._car.stop()
                except Exception as e:
                    self.get_logger().warn(f'Failed to stop motors (idle): {e!r}')
                self._last_idle_stop_time = now
            return

        if (now - self._last_cmd_time) > self._timeout:
            if self._idle_stop_period > 0.0 and (now - self._last_idle_stop_time) > self._idle_stop_period:
                try:
                    self._car.stop()
                except Exception as e:
                    self.get_logger().warn(f'Failed to stop motors (timeout): {e!r}')
                self._last_idle_stop_time = now
            return

        # Rate-limit I2C writes to reduce bus load and keep wheel updates consistent.
        if (now - self._last_write_time) < self._min_write_period:
            return

        # During the kick window, force a slightly higher PWM to help all wheels start together.
        kick_active = self._startup_kick_pwm > 0 and self._startup_kick_duration > 0.0 and now <= self._kick_until_time

        # ── Hard stop: when ALL base PWMs are zero, send an explicit stop ──
        # This prevents residual heading-hold / trim from spinning motors.
        # Guard: only send the stop I2C command once to avoid wasting bus bandwidth.
        if not self._is_any_motion_commanded():
            if not self._zero_stop_sent:
                try:
                    self._car.stop()
                except Exception as e:
                    self.get_logger().warn(f'Failed to stop motors (zero cmd): {e!r}')
                self._zero_stop_sent = True
                self._last_write_time = now
            return

        # We have active motion — clear the stop guard.
        self._zero_stop_sent = False

        try:
            if self._drive_mode == 'mecanum' and self._car.protocol == 'pi5':
                fl = self._apply_startup_kick(self._fl_pwm) if kick_active else self._fl_pwm
                fr = self._apply_startup_kick(self._fr_pwm) if kick_active else self._fr_pwm
                rl = self._apply_startup_kick(self._rl_pwm) if kick_active else self._rl_pwm
                rr = self._apply_startup_kick(self._rr_pwm) if kick_active else self._rr_pwm

                # ── IMU corrections ───────────────────────────────────
                if self._heading_hold_enable:
                    # PID heading-hold + adaptive trim (yaw correction)
                    # During pure strafe, the forward-drive adapted trim is not
                    # valid (different motor roles), so scale it down.
                    is_strafing = abs(self._cmd_vy) > 0.01 and abs(self._cmd_vx) < 0.01
                    trim_scale = 0.2 if is_strafing else 1.0
                    c = self._gyro_correction + self._adapted_trim * trim_scale
                    if abs(c) > 0.01:
                        fl = clamp(fl + c, -self._max_pwm, self._max_pwm)
                        rl = clamp(rl + c, -self._max_pwm, self._max_pwm)
                        fr = clamp(fr - c, -self._max_pwm, self._max_pwm)
                        rr = clamp(rr - c, -self._max_pwm, self._max_pwm)

                    # Lateral drift correction (mecanum strafe offset)
                    lc = self._lateral_correction
                    if abs(lc) > 0.01:
                        fl = clamp(fl - lc, -self._max_pwm, self._max_pwm)
                        fr = clamp(fr + lc, -self._max_pwm, self._max_pwm)
                        rl = clamp(rl + lc, -self._max_pwm, self._max_pwm)
                        rr = clamp(rr - lc, -self._max_pwm, self._max_pwm)

                # Per-motor manual trim (always applied)
                fl = clamp(fl + self._trim_fl, -self._max_pwm, self._max_pwm)
                fr = clamp(fr + self._trim_fr, -self._max_pwm, self._max_pwm)
                rl = clamp(rl + self._trim_rl, -self._max_pwm, self._max_pwm)
                rr = clamp(rr + self._trim_rr, -self._max_pwm, self._max_pwm)

                # Optional: strafe-only output smoothing to reduce speed pulsing
                if is_strafing and self._pwm_slew_rate_strafe > 0.0:
                    dt_send = (
                        self._min_write_period
                        if self._last_write_time <= 0.0
                        else clamp(now - self._last_write_time, 0.0, 0.2)
                    )
                    max_delta = self._pwm_slew_rate_strafe * dt_send
                    fl = self._slew_limit_pwm(fl, self._sent_fl_pwm, max_delta)
                    fr = self._slew_limit_pwm(fr, self._sent_fr_pwm, max_delta)
                    rl = self._slew_limit_pwm(rl, self._sent_rl_pwm, max_delta)
                    rr = self._slew_limit_pwm(rr, self._sent_rr_pwm, max_delta)

                self._car.set_motor_pwms(
                    fl,
                    fr,
                    rl,
                    rr,
                    max_pwm=self._max_pwm,
                    motor_id_fl=self._motor_id_fl,
                    motor_id_fr=self._motor_id_fr,
                    motor_id_rl=self._motor_id_rl,
                    motor_id_rr=self._motor_id_rr,
                )

                self._sent_fl_pwm = fl
                self._sent_fr_pwm = fr
                self._sent_rl_pwm = rl
                self._sent_rr_pwm = rr
            else:
                left = self._apply_startup_kick(self._left_pwm) if kick_active else self._left_pwm
                right = self._apply_startup_kick(self._right_pwm) if kick_active else self._right_pwm

                # ── IMU corrections (differential) ────────────────────
                if self._heading_hold_enable:
                    c = self._gyro_correction + self._adapted_trim
                    if abs(c) > 0.01:
                        left = clamp(left + c, -self._max_pwm, self._max_pwm)
                        right = clamp(right - c, -self._max_pwm, self._max_pwm)

                # Per-motor trim (applied as left/right offset)
                lt = 0.5 * (self._trim_fl + self._trim_rl)
                rt = 0.5 * (self._trim_fr + self._trim_rr)
                if abs(lt) > 0.01 or abs(rt) > 0.01:
                    left = clamp(left + lt, -self._max_pwm, self._max_pwm)
                    right = clamp(right + rt, -self._max_pwm, self._max_pwm)

                self._car.set_wheel_pwms(left, right, max_pwm=self._max_pwm)
            self._last_write_time = now
        except Exception as e:
            self.get_logger().error(f'I2C write failed: {e!r}')

    def destroy_node(self):
        try:
            self._car.stop()
            self._car.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
