import math
import threading
import time
from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from urllib.parse import parse_qs, urlparse
from typing import Optional
import hashlib
import base64
import struct
import socket

import numpy as np

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Vector3, Twist
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import BatteryState, CompressedImage, Imu, LaserScan, Range
from std_msgs.msg import Bool, Empty, Float32, Float64, Int32, Int32MultiArray, String, UInt8MultiArray
from std_srvs.srv import Trigger
import os


# ── Load static web files ─────────────────────────────────────────
_STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")


def _build_index_html():
    with open(os.path.join(_STATIC_DIR, "body.html"), "r") as _f:
        body = _f.read()
    return (
        "<!doctype html>\n"
        '<html lang="en">\n'
        "<head>\n"
        '  <meta charset="utf-8" />\n'
        '  <meta name="viewport" content="width=device-width, initial-scale=1" />\n'
        "  <title>Raspbot Camera</title>\n"
        '  <link rel="stylesheet" href="/static/style.css">\n'
        "</head>\n"
        "<body>\n"
        + body + "\n"
        '<script defer src="https://unpkg.com/react@18/umd/react.production.min.js"></script>\n'
        '<script defer src="https://unpkg.com/react-dom@18/umd/react-dom.production.min.js"></script>\n'
        '<script defer src="/static/layout_react.js"></script>\n'
        '<script defer src="/static/app.js"></script>\n'
        "</body>\n"
        "</html>\n"
    )


INDEX_HTML = _build_index_html()


# ── WebSocket helpers (RFC 6455) ──────────────────────────────────

_WS_MAGIC = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


def _ws_accept_key(key: str) -> str:
    return base64.b64encode(
        hashlib.sha1((key.strip() + _WS_MAGIC).encode()).digest()
    ).decode()


def _recv_exact(sock, n: int) -> Optional[bytes]:
    """Read exactly *n* bytes from *sock*. Returns None on disconnect."""
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


def _ws_send(sock, data: bytes, opcode: int = 0x01) -> None:
    """Send a single WebSocket frame (server → client, unmasked)."""
    hdr = bytes([0x80 | opcode])
    length = len(data)
    if length < 126:
        hdr += bytes([length])
    elif length < 65536:
        hdr += bytes([126]) + struct.pack("!H", length)
    else:
        hdr += bytes([127]) + struct.pack("!Q", length)
    sock.sendall(hdr + data)


def _ws_recv(sock) -> Optional[tuple]:
    """Read one WebSocket frame. Returns (opcode, payload) or None on close/error."""
    hdr = _recv_exact(sock, 2)
    if hdr is None:
        return None
    opcode = hdr[0] & 0x0F
    masked = bool(hdr[1] & 0x80)
    length = hdr[1] & 0x7F
    if length == 126:
        raw = _recv_exact(sock, 2)
        if raw is None:
            return None
        length = struct.unpack("!H", raw)[0]
    elif length == 127:
        raw = _recv_exact(sock, 8)
        if raw is None:
            return None
        length = struct.unpack("!Q", raw)[0]
    mask = _recv_exact(sock, 4) if masked else None
    if masked and mask is None:
        return None
    payload = _recv_exact(sock, length) if length > 0 else b""
    if payload is None:
        return None
    if masked and mask:
        payload = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
    return (opcode, payload)


@dataclass
class LatestFrame:
    jpeg: Optional[bytes] = None
    stamp_monotonic: float = 0.0


class FrameBuffer:
    def __init__(self) -> None:
        self._latest = LatestFrame()
        self._cv = threading.Condition()

    def set(self, jpeg: bytes) -> None:
        now = time.monotonic()
        with self._cv:
            self._latest = LatestFrame(jpeg=jpeg, stamp_monotonic=now)
            self._cv.notify_all()

    def get_latest(self) -> LatestFrame:
        with self._cv:
            return self._latest

    def wait_for_newer(self, last_stamp: float, timeout: float) -> LatestFrame:
        # If last_stamp is 0, the client just connected. Return whatever we have immediately.
        # This fixes LCP/start-up delay.
        if last_stamp == 0.0:
            with self._cv:
                return self._latest

        end = time.monotonic() + max(timeout, 0.0)
        with self._cv:
            while self._latest.stamp_monotonic <= last_stamp:
                remaining = end - time.monotonic()
                if remaining <= 0:
                    break
                self._cv.wait(timeout=remaining)
            return self._latest


class AudioBuffer:
    """Thread-safe buffer for streaming audio PCM chunks to HTTP clients."""

    def __init__(self, max_chunks: int = 128) -> None:
        self._chunks: list = []   # list of (seq, bytes)
        self._seq = 0
        self._max = max_chunks
        self._cv = threading.Condition()

    def push(self, data: bytes) -> None:
        with self._cv:
            self._chunks.append((self._seq, data))
            self._seq += 1
            if len(self._chunks) > self._max:
                self._chunks = self._chunks[-self._max:]
            self._cv.notify_all()

    def get_newer(self, after_seq: int, timeout: float = 1.0):
        """Return (latest_seq, concatenated_bytes) for chunks after *after_seq*."""
        end = time.monotonic() + timeout
        with self._cv:
            while True:
                result = b''
                latest_seq = after_seq
                for seq, data in self._chunks:
                    if seq > after_seq:
                        result += data
                        latest_seq = seq
                if result:
                    return (latest_seq, result)
                remaining = end - time.monotonic()
                if remaining <= 0:
                    return (after_seq, b'')
                self._cv.wait(timeout=remaining)


class WebVideoNode(Node):
    def __init__(self, frame_buffer: FrameBuffer, front_frame_buffer: FrameBuffer = None, audio_buffer: AudioBuffer = None, depth_frame_buffer: FrameBuffer = None) -> None:
        super().__init__("web_video")

        self.declare_parameter("topic", "image_raw/compressed")
        self.declare_parameter("front_camera_topic", "front_camera/compressed")
        self.declare_parameter("bind", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter("fps_limit", 6.0)
        self.declare_parameter("depth_fps_limit", 4.0)
        self.declare_parameter("http_backlog", 64)

        self.declare_parameter("gimbal_topic", "camera_gimbal/command_deg")
        self.declare_parameter("pan_min_deg", 0.0)
        self.declare_parameter("pan_max_deg", 180.0)
        self.declare_parameter("tilt_min_deg", 0.0)
        self.declare_parameter("tilt_max_deg", 110.0)
        self.declare_parameter("pan_neutral_deg", 90.0)
        self.declare_parameter("tilt_neutral_deg", 45.0)

        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("snapshot_dir", "~/Pictures/raspbot")
        # Max speeds (scaled by incoming -1..1 commands from the web UI).
        self.declare_parameter("max_linear_mps", 0.25)
        self.declare_parameter("max_angular_rps", 1.2)
        # If no cmd_vel received for this duration, publish a stop once.
        self.declare_parameter("cmd_timeout_sec", 0.5)

        # Detection/Tracking integration
        self.declare_parameter("detections_topic", "detections/json")
        self.declare_parameter("tracking_enable_topic", "tracking/enable")
        self.declare_parameter("tracking_config_topic", "tracking/config")
        self.declare_parameter("tracking_select_topic", "tracking/select_id")
        self.declare_parameter("follow_enable_topic", "follow/enable")
        self.declare_parameter("follow_target_area_topic", "follow/target_area")
        self.declare_parameter("follow_max_linear_topic", "follow/max_linear")
        self.declare_parameter("lightbar_command_topic", "lightbar/command")

        self._frame_buffer = frame_buffer
        self._front_frame_buffer = front_frame_buffer
        self._frame_count = 0
        self._front_frame_count = 0
        self._last_frame_monotonic = 0.0

        self._pan_min = float(self.get_parameter("pan_min_deg").value)
        self._pan_max = float(self.get_parameter("pan_max_deg").value)
        self._tilt_min = float(self.get_parameter("tilt_min_deg").value)
        self._tilt_max = float(self.get_parameter("tilt_max_deg").value)
        self._pan_neutral = float(self.get_parameter("pan_neutral_deg").value)
        self._tilt_neutral = float(self.get_parameter("tilt_neutral_deg").value)

        self._pan_deg = self._pan_neutral
        self._tilt_deg = self._tilt_neutral
        self._last_gimbal_monotonic = 0.0

        self._max_linear_mps = float(self.get_parameter("max_linear_mps").value)
        self._max_angular_rps = float(self.get_parameter("max_angular_rps").value)
        self._cmd_timeout_sec = float(self.get_parameter("cmd_timeout_sec").value)
        self._cmd_last_rx_monotonic = 0.0
        self._cmd_last_sent_monotonic = 0.0
        self._cmd_linear_x = 0.0
        self._cmd_linear_y = 0.0
        self._cmd_angular_z = 0.0

        topic = str(self.get_parameter("topic").value)
        self.get_logger().info(f"Subscribing to {topic}")

        # Use sensor-data QoS for better interoperability with camera publishers.
        self._sub = self.create_subscription(CompressedImage, topic, self._on_img, qos_profile_sensor_data)

        # Front camera (Pi Camera Module 3)
        front_topic = str(self.get_parameter("front_camera_topic").value)
        if self._front_frame_buffer is not None:
            self._front_sub = self.create_subscription(
                CompressedImage, front_topic, self._on_front_img, qos_profile_sensor_data
            )
            self.get_logger().info(f"Front camera subscribing to {front_topic}")
        else:
            self._front_sub = None

        gimbal_topic = str(self.get_parameter("gimbal_topic").value)
        self._gimbal_pub = self.create_publisher(Vector3, gimbal_topic, 10)
        self.get_logger().info(f"Gimbal control topic: {gimbal_topic} (x=pan_deg, y=tilt_deg)")

        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.get_logger().info(
            f"Drive control topic: {cmd_vel_topic} (Twist linear.x, angular.z), max_linear={self._max_linear_mps} m/s, max_angular={self._max_angular_rps} rad/s"
        )
        # Watchdog to stop if browser disconnects / stops sending.
        self._cmd_timer = self.create_timer(0.1, self._cmd_watchdog)

        detections_topic = str(self.get_parameter("detections_topic").value)
        self._latest_detections_json = None
        self._last_detections_monotonic = 0.0
        self._detections_sub = self.create_subscription(String, detections_topic, self._on_detections, 10)
        self.get_logger().info(f"Detections topic: {detections_topic} (std_msgs/String JSON)")

        # Face-recognition enriched detections (preferred over raw hailo detections)
        face_det_topic = "face_recognition/detections"
        self._face_detections_json = None
        self._face_det_monotonic = 0.0
        self._face_det_sub = self.create_subscription(String, face_det_topic, self._on_face_detections, 10)
        self.get_logger().info(f"Face detections topic: {face_det_topic}")

        # Face database for management API
        from raspbot_web_video.face_db import FaceDB as _FaceDB
        self._face_db = _FaceDB()
        self.get_logger().info(f"Face DB loaded: {self._face_db.db_path} ({self._face_db.num_faces} faces)")

        tracking_enable_topic = str(self.get_parameter("tracking_enable_topic").value)
        self._tracking_enabled = False
        self._tracking_pub = self.create_publisher(Bool, tracking_enable_topic, 10)
        self.get_logger().info(f"Tracking enable topic: {tracking_enable_topic} (std_msgs/Bool)")

        tracking_config_topic = str(self.get_parameter("tracking_config_topic").value)
        self._tracking_pan_sign = -1
        self._tracking_tilt_sign = -1
        self._tracking_config_pub = self.create_publisher(Int32MultiArray, tracking_config_topic, 10)
        self.get_logger().info(
            f"Tracking config topic: {tracking_config_topic} (std_msgs/Int32MultiArray [pan_sign, tilt_sign])"
        )

        tracking_select_topic = str(self.get_parameter("tracking_select_topic").value)
        self._selected_person_id = -1
        self._tracking_select_pub = self.create_publisher(Int32, tracking_select_topic, 10)
        self.get_logger().info(f"Tracking select topic: {tracking_select_topic} (std_msgs/Int32)")

        follow_enable_topic = str(self.get_parameter("follow_enable_topic").value)
        self._follow_enabled = False
        self._follow_pub = self.create_publisher(Bool, follow_enable_topic, 10)
        self.get_logger().info(f"Follow enable topic: {follow_enable_topic} (std_msgs/Bool)")

        follow_target_area_topic = str(self.get_parameter("follow_target_area_topic").value)
        self._follow_target_bbox_area = 0.04
        self._follow_target_area_pub = self.create_publisher(Float64, follow_target_area_topic, 10)

        follow_max_linear_topic = str(self.get_parameter("follow_max_linear_topic").value)
        self._follow_max_linear = 0.30
        self._follow_max_linear_pub = self.create_publisher(Float64, follow_max_linear_topic, 10)

        # Runtime-tunable strafe gain and gyro damping
        self.declare_parameter("follow_strafe_gain_topic", "follow/strafe_gain")
        self.declare_parameter("follow_gyro_damping_topic", "follow/gyro_damping")
        follow_strafe_gain_topic = str(self.get_parameter("follow_strafe_gain_topic").value)
        follow_gyro_damping_topic = str(self.get_parameter("follow_gyro_damping_topic").value)
        self._follow_strafe_gain = 0.50
        self._follow_gyro_damping = 0.15
        self._follow_strafe_gain_pub = self.create_publisher(Float64, follow_strafe_gain_topic, 10)
        self._follow_gyro_damping_pub = self.create_publisher(Float64, follow_gyro_damping_topic, 10)

        lightbar_command_topic = str(self.get_parameter("lightbar_command_topic").value)
        self._lightbar_pub = self.create_publisher(String, lightbar_command_topic, 10)
        self.get_logger().info(f"Lightbar command topic: {lightbar_command_topic} (std_msgs/String JSON)")

        # ── IMU integration ───────────────────────────────────────────
        self.declare_parameter("imu_data_topic", "imu/data")
        self.declare_parameter("imu_yaw_topic", "imu/yaw_deg")
        self.declare_parameter("imu_calibrated_topic", "imu/calibrated")
        self.declare_parameter("imu_calibrate_topic", "imu/calibrate")
        self.declare_parameter("imu_temperature_topic", "imu/temperature")
        self.declare_parameter("imu_mic_topic", "imu/mic_level")
        # Rotate-to-angle controller params
        # NOTE: P and D are normalised — error ÷ ramp_deg, rate ÷ max_rate_dps
        # so gains are directly comparable (1.0 = full output at boundary).
        self.declare_parameter("rotate_kp", 2.5)       # proportional gain (normalised error)
        self.declare_parameter("rotate_ki", 0.5)       # integral gain (overcomes friction near target)
        self.declare_parameter("rotate_kd", 0.5)       # derivative gain (normalised gyro rate)
        self.declare_parameter("rotate_ramp_deg", 30.0) # error angle at which P-term reaches kp
        self.declare_parameter("rotate_tolerance_deg", 2.0)
        self.declare_parameter("rotate_settle_sec", 0.4)
        self.declare_parameter("rotate_integral_max", 0.5)  # anti-windup cap (normalised·s)
        self.declare_parameter("rotate_min_output", 0.06)   # minimum |angular_unit| to overcome motor deadband

        imu_data_topic = str(self.get_parameter("imu_data_topic").value)
        imu_yaw_topic = str(self.get_parameter("imu_yaw_topic").value)
        imu_cal_topic = str(self.get_parameter("imu_calibrated_topic").value)
        imu_calibrate_topic = str(self.get_parameter("imu_calibrate_topic").value)
        imu_temp_topic = str(self.get_parameter("imu_temperature_topic").value)
        imu_mic_topic = str(self.get_parameter("imu_mic_topic").value)

        self._imu_accel = [0.0, 0.0, 0.0]
        self._imu_gyro = [0.0, 0.0, 0.0]
        self._imu_pitch_deg = 0.0
        self._imu_roll_deg = 0.0
        self._imu_has_quaternion = False
        self._imu_temperature = 0.0
        self._imu_mic_level = 0
        self._imu_yaw_deg = 0.0
        self._imu_calibrated = False
        self._imu_last_monotonic = 0.0

        # Rotate-to-angle controller state
        self._rotate_active = False
        self._rotate_target_deg = None
        self._rotate_speed = 0.5
        self._rotate_kp = float(self.get_parameter("rotate_kp").value)
        self._rotate_ki = float(self.get_parameter("rotate_ki").value)
        self._rotate_kd = float(self.get_parameter("rotate_kd").value)
        self._rotate_ramp_deg = float(self.get_parameter("rotate_ramp_deg").value)
        self._rotate_tolerance = float(self.get_parameter("rotate_tolerance_deg").value)
        self._rotate_settle_sec = float(self.get_parameter("rotate_settle_sec").value)
        self._rotate_integral_max = float(self.get_parameter("rotate_integral_max").value)
        self._rotate_min_output = float(self.get_parameter("rotate_min_output").value)
        self._rotate_in_tolerance_since = None
        self._rotate_integral = 0.0
        self._rotate_prev_error = None
        self._rotate_last_time = 0.0
        self._rotate_last_output = 0.0

        self._imu_sub = self.create_subscription(Imu, imu_data_topic, self._on_imu_data, 10)
        self._imu_yaw_sub = self.create_subscription(Float64, imu_yaw_topic, self._on_imu_yaw, 10)
        self._imu_cal_sub = self.create_subscription(Bool, imu_cal_topic, self._on_imu_cal, 10)
        self._imu_temp_sub = self.create_subscription(Float32, imu_temp_topic, self._on_imu_temp, 10)
        self._imu_mic_sub = self.create_subscription(Int32, imu_mic_topic, self._on_imu_mic, 10)
        self._imu_calibrate_pub = self.create_publisher(Empty, imu_calibrate_topic, 10)

        # Rotate controller timer (20 Hz)
        self._rotate_timer = self.create_timer(0.1, self._rotate_tick)

        self.get_logger().info(f"IMU topics: data={imu_data_topic}, yaw={imu_yaw_topic}, cal={imu_cal_topic}")

        # ── BNO055 calibration integration ─────────────────────────────
        self.declare_parameter("imu_calibration_topic", "imu/calibration")
        self.declare_parameter("imu_save_cal_topic", "imu/save_cal")
        imu_calibration_topic = str(self.get_parameter("imu_calibration_topic").value)
        imu_save_cal_topic = str(self.get_parameter("imu_save_cal_topic").value)

        self._bno_cal = {'sys': 0, 'gyro': 0, 'accel': 0, 'mag': 0}
        self._bno_cal_available = False

        self._bno_cal_sub = self.create_subscription(
            String, imu_calibration_topic, self._on_bno_cal, 10
        )
        self._imu_save_cal_pub = self.create_publisher(Empty, imu_save_cal_topic, 10)

        # Auto-calibration state machine
        # States: 'idle', 'spin_mag', 'tilt_accel', 'still_gyro', 'wait_sys', 'saving', 'done', 'failed'
        self._autocal_state = 'idle'
        self._autocal_phase_start = 0.0
        self._autocal_status_msg = ''
        self._autocal_spin_dir = 1.0
        self._autocal_tilt_phase = 0  # sub-phase for tilt sequence
        self._autocal_tilt_start = 0.0
        self._autocal_timer = self.create_timer(0.1, self._autocal_tick)  # 10 Hz

        self.get_logger().info(
            f"BNO055 calibration topics: detail={imu_calibration_topic}, "
            f"save={imu_save_cal_topic}"
        )

        # ── Audio streaming ─────────────────────────────────────────
        self.declare_parameter("imu_audio_topic", "imu/audio")
        self.declare_parameter("imu_audio_enable_topic", "imu/audio_enable")

        imu_audio_topic = str(self.get_parameter("imu_audio_topic").value)
        imu_audio_enable_topic = str(self.get_parameter("imu_audio_enable_topic").value)

        self._audio_buffer = audio_buffer
        self._audio_enabled = False

        self._audio_sub = self.create_subscription(
            UInt8MultiArray, imu_audio_topic, self._on_audio, 10
        )
        self._audio_enable_pub = self.create_publisher(Bool, imu_audio_enable_topic, 10)
        self.get_logger().info(f"Audio topics: data={imu_audio_topic}, enable={imu_audio_enable_topic}")

        # ── Depth estimation integration ─────────────────────────────
        self.declare_parameter("depth_colorized_topic", "depth/colorized/compressed")
        self.declare_parameter("depth_enable_topic", "depth/enable")

        depth_colorized_topic = str(self.get_parameter("depth_colorized_topic").value)
        depth_enable_topic = str(self.get_parameter("depth_enable_topic").value)

        self._depth_frame_buffer = depth_frame_buffer
        self._depth_frame_count = 0
        self._depth_enabled = True

        if self._depth_frame_buffer is not None:
            self._depth_sub = self.create_subscription(
                CompressedImage, depth_colorized_topic, self._on_depth_img, qos_profile_sensor_data
            )
            self.get_logger().info(f"Depth colorized subscribing to {depth_colorized_topic}")
        else:
            self._depth_sub = None

        self._depth_enable_pub = self.create_publisher(Bool, depth_enable_topic, 10)
        self.get_logger().info(f"Depth enable topic: {depth_enable_topic}")

        # ── Battery monitor integration ───────────────────────────────
        self.declare_parameter("battery_state_topic", "battery/state")
        battery_topic = str(self.get_parameter("battery_state_topic").value)

        self._battery_voltage = 0.0
        self._battery_percentage = 0.0  # 0–100
        self._battery_present = False
        self._battery_last_monotonic = 0.0

        self._battery_sub = self.create_subscription(
            BatteryState, battery_topic, self._on_battery_state, 10
        )
        self.get_logger().info(f"Battery state topic: {battery_topic}")

        # ── Collision failsafe integration ────────────────────────────
        self.declare_parameter("collision_failsafe_enable_topic", "collision_failsafe/enable")
        self.declare_parameter("collision_failsafe_active_topic", "collision_failsafe/active")
        self.declare_parameter("ultrasonic_range_topic", "ultrasonic/range")

        cf_enable_topic = str(self.get_parameter("collision_failsafe_enable_topic").value)
        cf_active_topic = str(self.get_parameter("collision_failsafe_active_topic").value)
        us_range_topic = str(self.get_parameter("ultrasonic_range_topic").value)

        self._collision_failsafe_enabled = True   # mirrors motor_driver default
        self._collision_failsafe_active = False
        self._collision_distance_m = float('inf')

        self._cf_enable_pub = self.create_publisher(Bool, cf_enable_topic, 10)
        self._cf_active_sub = self.create_subscription(
            Bool, cf_active_topic, self._on_collision_active, 10
        )
        self._us_range_sub = self.create_subscription(
            Range, us_range_topic, self._on_ultrasonic_range, 10
        )
        self.get_logger().info(
            f"Collision failsafe topics: enable={cf_enable_topic}, "
            f"active={cf_active_topic}, range={us_range_topic}"
        )

        # ── Cliff failsafe integration ────────────────────────────────
        self.declare_parameter("cliff_failsafe_enable_topic", "cliff_failsafe/enable")
        self.declare_parameter("cliff_failsafe_active_topic", "cliff_failsafe/active")
        self.declare_parameter("cliff_tracking_state_topic", "tracking/state")

        cl_enable_topic = str(self.get_parameter("cliff_failsafe_enable_topic").value)
        cl_active_topic = str(self.get_parameter("cliff_failsafe_active_topic").value)
        cl_tracking_topic = str(self.get_parameter("cliff_tracking_state_topic").value)

        self._cliff_failsafe_enabled = True   # mirrors motor_driver default
        self._cliff_failsafe_active = False
        self._cliff_sensor_state = 0

        self._cl_enable_pub = self.create_publisher(Bool, cl_enable_topic, 10)
        self._cl_active_sub = self.create_subscription(
            Bool, cl_active_topic, self._on_cliff_active, 10
        )
        self._cl_tracking_sub = self.create_subscription(
            Int32, cl_tracking_topic, self._on_cliff_tracking, 10
        )
        self.get_logger().info(
            f"Cliff failsafe topics: enable={cl_enable_topic}, "
            f"active={cl_active_topic}, tracking={cl_tracking_topic}"
        )

        # ── Odometry integration ──────────────────────────────────────
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("odom_path_topic", "odom/path")
        self.declare_parameter("odom_recording_topic", "odom/recording")
        self.declare_parameter("odom_returning_topic", "odom/returning")

        odom_topic = str(self.get_parameter("odom_topic").value)
        odom_path_topic = str(self.get_parameter("odom_path_topic").value)
        odom_recording_topic = str(self.get_parameter("odom_recording_topic").value)
        odom_returning_topic = str(self.get_parameter("odom_returning_topic").value)

        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw_deg = 0.0
        self._odom_vx = 0.0
        self._odom_vy = 0.0
        self._odom_wz = 0.0
        self._odom_last_monotonic = 0.0
        self._odom_recording = False
        self._odom_returning = False
        self._odom_stuck = False
        self._odom_navigating = False
        self._odom_nav_wp_idx = 0
        self._odom_nav_wp_total = 0
        self._odom_nav_target_x = 0.0
        self._odom_nav_target_y = 0.0
        self._odom_path_xy: list = []  # [(x, y), ...] from Path messages
        self._odom_trail_xy: list = []  # live breadcrumb trail from Odometry

        self._odom_sub = self.create_subscription(
            Odometry, odom_topic, self._on_odom, 10
        )
        self._odom_path_sub = self.create_subscription(
            Path, odom_path_topic, self._on_odom_path, 10
        )
        self._odom_recording_sub = self.create_subscription(
            Bool, odom_recording_topic, self._on_odom_recording, 10
        )
        self._odom_returning_sub = self.create_subscription(
            Bool, odom_returning_topic, self._on_odom_returning, 10
        )
        self._odom_stuck_sub = self.create_subscription(
            Bool, 'odom/stuck', self._on_odom_stuck, 10
        )
        self._odom_nav_status_sub = self.create_subscription(
            String, 'odom/nav_status', self._on_odom_nav_status, 10
        )

        # Service clients for odometry services
        self._odom_set_origin_cli = self.create_client(Trigger, 'odom/set_origin')
        self._odom_start_recording_cli = self.create_client(Trigger, 'odom/start_recording')
        self._odom_stop_recording_cli = self.create_client(Trigger, 'odom/stop_recording')
        self._odom_return_to_origin_cli = self.create_client(Trigger, 'odom/return_to_origin')
        self._odom_cancel_return_cli = self.create_client(Trigger, 'odom/cancel_return')
        self._odom_cancel_navigate_cli = self.create_client(Trigger, 'odom/cancel_navigate')

        # Publisher for navigate-to-waypoint commands
        from geometry_msgs.msg import Point
        self._odom_navigate_to_pub = self.create_publisher(Point, 'odom/navigate_to', 10)
        # Publisher for multi-waypoint path navigation
        self._odom_navigate_path_pub = self.create_publisher(Path, 'odom/navigate_path', 10)

        self.get_logger().info(
            f"Odometry topics: odom={odom_topic}, path={odom_path_topic}, "
            f"recording={odom_recording_topic}, returning={odom_returning_topic}"
        )

        # ── LiDAR scan integration ────────────────────────────────────
        self.declare_parameter("scan_topic", "scan")
        scan_topic = str(self.get_parameter("scan_topic").value)

        self._scan_angles: list = []     # angle for each ray (radians)
        self._scan_ranges: list = []     # range for each ray (metres)
        self._scan_angle_min = 0.0
        self._scan_angle_max = 0.0
        self._scan_range_min = 0.0
        self._scan_range_max = 12.0
        self._scan_last_monotonic = 0.0

        self._scan_sub = self.create_subscription(
            LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data
        )
        self.get_logger().info(f"LiDAR scan topic: {scan_topic}")

        # ── SLAM map integration ──────────────────────────────────────
        self.declare_parameter("map_topic", "map")
        map_topic = str(self.get_parameter("map_topic").value)

        self._map_data: list = []       # raw occupancy data (-1, 0–100)
        self._map_width = 0
        self._map_height = 0
        self._map_resolution = 0.05
        self._map_origin_x = 0.0
        self._map_origin_y = 0.0
        self._map_last_monotonic = 0.0
        self._map_dirty = False
        self._map_b64_cache = ''

        self._map_sub = self.create_subscription(
            OccupancyGrid, map_topic, self._on_map, 10
        )
        self.get_logger().info(f"SLAM map topic: {map_topic}")

        # ── SLAM service clients ──────────────────────────────────────
        self._slam_reset_cli = self.create_client(Trigger, 'slam/reset')

        # ── Front calibration service clients ─────────────────────────
        self._cal_front_cli = self.create_client(Trigger, 'calibrate_front')
        self._cal_static_cli = self.create_client(Trigger, 'calibrate_front_static')

        # ── LiDAR obstacle zone ranges ────────────────────────────────
        self.declare_parameter("lidar_front_range_topic", "lidar/front_range")
        self.declare_parameter("lidar_left_range_topic", "lidar/left_range")
        self.declare_parameter("lidar_right_range_topic", "lidar/right_range")
        self.declare_parameter("lidar_rear_range_topic", "lidar/rear_range")

        self._lidar_front_range = float('inf')
        self._lidar_left_range = float('inf')
        self._lidar_right_range = float('inf')
        self._lidar_rear_range = float('inf')
        self._lidar_zones_last = 0.0

        self.create_subscription(
            Range, str(self.get_parameter("lidar_front_range_topic").value),
            self._on_lidar_front, 10
        )
        self.create_subscription(
            Range, str(self.get_parameter("lidar_left_range_topic").value),
            self._on_lidar_left, 10
        )
        self.create_subscription(
            Range, str(self.get_parameter("lidar_right_range_topic").value),
            self._on_lidar_right, 10
        )
        self.create_subscription(
            Range, str(self.get_parameter("lidar_rear_range_topic").value),
            self._on_lidar_rear, 10
        )

    @staticmethod
    def _clamp(x: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, x))

    def set_gimbal(self, pan_deg: float, tilt_deg: float) -> None:
        pan = self._clamp(float(pan_deg), self._pan_min, self._pan_max)
        tilt = self._clamp(float(tilt_deg), self._tilt_min, self._tilt_max)

        msg = Vector3()
        msg.x = float(pan)
        msg.y = float(tilt)
        msg.z = 0.0
        self._gimbal_pub.publish(msg)

        self._pan_deg = float(pan)
        self._tilt_deg = float(tilt)
        self._last_gimbal_monotonic = time.monotonic()

    def center_gimbal(self) -> None:
        self.set_gimbal(self._pan_neutral, self._tilt_neutral)

    def set_cmd_vel(self, linear_x_unit: float, angular_z_unit: float, linear_y_unit: float = 0.0) -> None:
        # Inputs from UI are in approx [-1..1]. Clamp and scale.
        lin_u = self._clamp(float(linear_x_unit), -1.0, 1.0)
        lat_u = self._clamp(float(linear_y_unit), -1.0, 1.0)
        ang_u = self._clamp(float(angular_z_unit), -1.0, 1.0)

        self._cmd_linear_x = float(lin_u) * float(self._max_linear_mps)
        self._cmd_linear_y = float(lat_u) * float(self._max_linear_mps)
        self._cmd_angular_z = float(ang_u) * float(self._max_angular_rps)

        msg = Twist()
        msg.linear.x = float(self._cmd_linear_x)
        msg.linear.y = float(self._cmd_linear_y)
        msg.angular.z = float(self._cmd_angular_z)
        self._cmd_pub.publish(msg)

        now = time.monotonic()
        self._cmd_last_rx_monotonic = now
        self._cmd_last_sent_monotonic = now

    def stop_cmd_vel(self) -> None:
        self._cmd_linear_x = 0.0
        self._cmd_linear_y = 0.0
        self._cmd_angular_z = 0.0
        msg = Twist()
        self._cmd_pub.publish(msg)
        now = time.monotonic()
        # Consider ourselves idle after an explicit stop so the watchdog doesn't
        # keep re-sending stop commands forever.
        self._cmd_last_rx_monotonic = 0.0
        self._cmd_last_sent_monotonic = now

    def _cmd_watchdog(self) -> None:
        if self._cmd_timeout_sec <= 0.0:
            return
        now = time.monotonic()
        if self._cmd_last_rx_monotonic <= 0.0:
            return
        if (now - self._cmd_last_rx_monotonic) > self._cmd_timeout_sec:
            # Only publish stop once per timeout episode.
            if (now - self._cmd_last_sent_monotonic) > self._cmd_timeout_sec:
                self.stop_cmd_vel()

    def _on_img(self, msg: CompressedImage) -> None:
        # Typically msg.format is 'jpeg'; we still just pass through the data.
        if not msg.data:
            return
        self._frame_buffer.set(bytes(msg.data))
        self._frame_count += 1
        self._last_frame_monotonic = time.monotonic()
        if self._frame_count == 1:
            try:
                fmt = getattr(msg, 'format', '')
            except Exception:
                fmt = ''
            self.get_logger().info(f"First frame received (format={fmt}, bytes={len(msg.data)})")

    def _on_front_img(self, msg: CompressedImage) -> None:
        if not msg.data or self._front_frame_buffer is None:
            return
        self._front_frame_buffer.set(bytes(msg.data))
        self._front_frame_count += 1
        if self._front_frame_count == 1:
            try:
                fmt = getattr(msg, 'format', '')
            except Exception:
                fmt = ''
            self.get_logger().info(f"First front camera frame received (format={fmt}, bytes={len(msg.data)})")

    def status_dict(self) -> dict:
        now = time.monotonic()
        age_s = None
        if self._last_frame_monotonic > 0.0:
            age_s = max(0.0, now - self._last_frame_monotonic)
        latest = self._frame_buffer.get_latest()
        size = len(latest.jpeg) if latest.jpeg is not None else 0
        return {
            'frame_count': int(self._frame_count),
            'last_frame_age_s': age_s,
            'latest_jpeg_bytes': int(size),
            'front_camera': {
                'frame_count': int(self._front_frame_count),
                'available': self._front_frame_buffer is not None,
            },
            'cmd_vel': {
                'max_linear_mps': float(self._max_linear_mps),
                'max_angular_rps': float(self._max_angular_rps),
                'timeout_sec': float(self._cmd_timeout_sec),
                'state': {
                    'linear_x': float(self._cmd_linear_x),
                    'linear_y': float(self._cmd_linear_y),
                    'angular_z': float(self._cmd_angular_z),
                    'last_cmd_age_s': (None if self._cmd_last_rx_monotonic <= 0.0 else max(0.0, now - self._cmd_last_rx_monotonic)),
                },
            },
            'gimbal': {
                'limits': {
                    'pan_min_deg': float(self._pan_min),
                    'pan_max_deg': float(self._pan_max),
                    'tilt_min_deg': float(self._tilt_min),
                    'tilt_max_deg': float(self._tilt_max),
                },
                'neutral': {
                    'pan_neutral_deg': float(self._pan_neutral),
                    'tilt_neutral_deg': float(self._tilt_neutral),
                },
                'state': {
                    'pan_deg': float(self._pan_deg),
                    'tilt_deg': float(self._tilt_deg),
                    'last_cmd_age_s': (None if self._last_gimbal_monotonic <= 0.0 else max(0.0, now - self._last_gimbal_monotonic)),
                },
            },
            'detections': {
                'last_age_s': (None if self._last_detections_monotonic <= 0.0 else max(0.0, now - self._last_detections_monotonic)),
            },
            'tracking': {
                'enabled': bool(self._tracking_enabled),
                'pan_sign': int(self._tracking_pan_sign),
                'tilt_sign': int(self._tracking_tilt_sign),
                'selected_person_id': int(self._selected_person_id),
            },
            'follow': {
                'enabled': bool(self._follow_enabled),
                'target_bbox_area': float(self._follow_target_bbox_area),
                'max_linear': float(self._follow_max_linear),
                'strafe_gain': float(self._follow_strafe_gain),
                'gyro_damping': float(self._follow_gyro_damping),
            },
            'collision_failsafe': {
                'enabled': bool(self._collision_failsafe_enabled),
                'active': bool(self._collision_failsafe_active),
                'distance_m': (
                    round(float(self._collision_distance_m), 3)
                    if self._collision_distance_m != float('inf')
                    and self._collision_distance_m == self._collision_distance_m
                    else None
                ),
            },
            'cliff_failsafe': {
                'enabled': bool(self._cliff_failsafe_enabled),
                'active': bool(self._cliff_failsafe_active),
                'sensor_state': int(self._cliff_sensor_state),
            },
            'odometry': {
                'x': round(self._odom_x, 4),
                'y': round(self._odom_y, 4),
                'yaw_deg': round(self._odom_yaw_deg, 2),
                'recording': bool(self._odom_recording),
                'returning': bool(self._odom_returning),
                'stuck': bool(self._odom_stuck),
                'navigating': bool(self._odom_navigating),
            },
            'battery': {
                'present': bool(self._battery_present),
                'voltage': round(float(self._battery_voltage), 3),
                'percentage': round(float(self._battery_percentage), 1),
                'last_update_age_s': (
                    round(max(0.0, now - self._battery_last_monotonic), 1)
                    if self._battery_last_monotonic > 0.0 else None
                ),
            },
        }

    def _on_detections(self, msg: String) -> None:
        if not msg.data:
            return
        self._latest_detections_json = msg.data
        self._last_detections_monotonic = time.monotonic()

    def _on_face_detections(self, msg: String) -> None:
        if not msg.data:
            return
        self._face_detections_json = msg.data
        self._face_det_monotonic = time.monotonic()

    def get_latest_detections(self) -> dict:
        # Prefer face-recognition enriched detections if recent (< 2s)
        now = time.monotonic()
        if self._face_detections_json and (now - self._face_det_monotonic) < 2.0:
            try:
                return json.loads(self._face_detections_json)
            except Exception:
                pass
        # Always return valid JSON.
        if not self._latest_detections_json:
            return {"image_width": None, "image_height": None, "detections": []}
        try:
            return json.loads(self._latest_detections_json)
        except Exception:
            return {"error": "invalid_detections_json", "detections": []}

    def get_latest_detections_json(self) -> str:
        """Return raw JSON string for detections to avoid repetitive parsing/serialization."""
        now = time.monotonic()
        if self._face_detections_json and (now - self._face_det_monotonic) < 2.0:
            return self._face_detections_json
        if self._latest_detections_json:
            return self._latest_detections_json
        return '{"image_width": null, "image_height": null, "detections": []}'

    # ------------------------------------------------------------------
    # Face DB management (delegated to FaceDB instance)
    # ------------------------------------------------------------------
    def face_list(self) -> list:
        return self._face_db.get_all_faces()

    def face_rename(self, face_id: int, name: str) -> bool:
        ok = self._face_db.update_face_name(face_id, name)
        if ok:
            self._face_db._rebuild_centroids()
        return ok

    def face_delete(self, face_id: int) -> bool:
        return self._face_db.delete_face(face_id)

    def face_merge(self, keep_id: int, merge_id: int) -> bool:
        return self._face_db.merge_faces(keep_id, merge_id)

    def face_clear(self) -> int:
        return self._face_db.clear_all()

    def face_thumbnail(self, face_id: int):
        return self._face_db.get_thumbnail(face_id)

    def set_tracking_enabled(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        self._tracking_pub.publish(msg)
        self._tracking_enabled = bool(enabled)

    def set_follow_enabled(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        self._follow_pub.publish(msg)
        self._follow_enabled = bool(enabled)

    def set_follow_config(self, target_bbox_area: float = None, max_linear: float = None,
                         strafe_gain: float = None, gyro_damping: float = None) -> None:
        if target_bbox_area is not None:
            val = max(0.01, min(1.0, float(target_bbox_area)))
            self._follow_target_bbox_area = val
            msg = Float64()
            msg.data = val
            self._follow_target_area_pub.publish(msg)
        if max_linear is not None:
            val = max(0.05, min(1.0, float(max_linear)))
            self._follow_max_linear = val
            msg = Float64()
            msg.data = val
            self._follow_max_linear_pub.publish(msg)
        if strafe_gain is not None:
            val = max(0.0, min(2.0, float(strafe_gain)))
            self._follow_strafe_gain = val
            msg = Float64()
            msg.data = val
            self._follow_strafe_gain_pub.publish(msg)
        if gyro_damping is not None:
            val = max(0.0, min(1.0, float(gyro_damping)))
            self._follow_gyro_damping = val
            msg = Float64()
            msg.data = val
            self._follow_gyro_damping_pub.publish(msg)

    def take_snapshot(self) -> dict:
        """Grab the latest full-resolution JPEG and save to disk. Returns {filename, path}."""
        from datetime import datetime

        latest = self._frame_buffer.get_latest()
        if latest.jpeg is None or len(latest.jpeg) == 0:
            raise RuntimeError("No frame available")

        snap_dir = str(self.get_parameter("snapshot_dir").value)
        snap_dir = os.path.expanduser(snap_dir)
        os.makedirs(snap_dir, exist_ok=True)

        ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]  # trim to milliseconds
        filename = f"raspbot_{ts}.jpg"
        filepath = os.path.join(snap_dir, filename)

        with open(filepath, "wb") as f:
            f.write(latest.jpeg)

        self.get_logger().info(f"Snapshot saved: {filepath} ({len(latest.jpeg)} bytes)")
        return {"filename": filename, "path": filepath, "size": len(latest.jpeg)}

    @staticmethod
    def _norm_sign(x: int) -> int:
        try:
            v = int(x)
        except Exception:
            v = 1
        return -1 if v < 0 else 1

    def set_tracking_config(self, pan_sign: Optional[int] = None, tilt_sign: Optional[int] = None) -> None:
        if pan_sign is not None:
            self._tracking_pan_sign = self._norm_sign(pan_sign)
        if tilt_sign is not None:
            self._tracking_tilt_sign = self._norm_sign(tilt_sign)

        msg = Int32MultiArray()
        msg.data = [int(self._tracking_pan_sign), int(self._tracking_tilt_sign)]
        self._tracking_config_pub.publish(msg)

    def set_person_select(self, person_id: int) -> None:
        """Select a specific person by track_id for exclusive tracking/following.
        -1 clears the selection (auto mode)."""
        self._selected_person_id = int(person_id)
        msg = Int32()
        msg.data = int(person_id)
        self._tracking_select_pub.publish(msg)
        self.get_logger().info(f"Person selection: {'cleared' if person_id < 0 else f'track_id={person_id}'}")

    def set_lightbar_command(self, cmd: dict) -> None:
        """Forward a JSON lightbar command to the lightbar/command topic."""
        msg = String()
        msg.data = json.dumps(cmd)
        self._lightbar_pub.publish(msg)

    # ── IMU callbacks ─────────────────────────────────────────────────

    def _on_imu_data(self, msg: Imu) -> None:
        self._imu_accel = [
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ]
        self._imu_gyro = [
            msg.angular_velocity.x * 57.2957795,  # rad/s → °/s for display
            msg.angular_velocity.y * 57.2957795,
            msg.angular_velocity.z * 57.2957795,
        ]
        self._imu_last_monotonic = time.monotonic()

        # ── Extract pitch / roll for the 3D cube ─────────────────────
        # orientation_covariance[0] >= 0  →  BNO055 quaternion valid
        # orientation_covariance[0] == -1 →  LSM6DSOX only (no orientation)
        if msg.orientation_covariance[0] >= 0.0:
            qw = msg.orientation.w
            qx = msg.orientation.x
            qy = msg.orientation.y
            qz = msg.orientation.z
            # Project world "up" into the body frame via the quaternion
            # rotation matrix.  This gives an accelerometer-equivalent
            # gravity reading (positive Z = up) regardless of sensor
            # mounting orientation — exactly what the 3D cube needs.
            #   accel_eq = −row₂(R)   where R is the body→world matrix
            ax_eq = 2.0 * (qw * qy - qx * qz)
            ay_eq = -2.0 * (qw * qx + qy * qz)
            az_eq = 2.0 * (qx * qx + qy * qy) - 1.0
            denom_p = ay_eq * ay_eq + az_eq * az_eq
            raw_pitch = math.degrees(
                math.atan2(ax_eq, math.sqrt(denom_p))
            ) if denom_p > 1e-6 else 0.0
            raw_roll = math.degrees(
                math.atan2(-ay_eq, az_eq)
            ) if (abs(az_eq) > 1e-6 or abs(ay_eq) > 1e-6) else 0.0
            # Compensate for BNO055 mounted upside-down (az_eq < 0 ≈ Z-down).
            # When inverted, raw_roll reads ≈ ±180° on a flat surface.
            if az_eq < -0.3:  # sensor Z axis pointing downward
                raw_pitch = -raw_pitch
                if raw_roll > 0:
                    raw_roll -= 180.0
                else:
                    raw_roll += 180.0
            self._imu_pitch_deg = raw_pitch
            self._imu_roll_deg = raw_roll
            self._imu_has_quaternion = True
        else:
            # Fallback: derive pitch/roll from raw accelerometer
            ax = msg.linear_acceleration.x
            ay = msg.linear_acceleration.y
            az = msg.linear_acceleration.z
            self._imu_pitch_deg = math.degrees(
                math.atan2(ax, math.sqrt(ay * ay + az * az))
            ) if (ay * ay + az * az) > 0.001 else 0.0
            self._imu_roll_deg = math.degrees(
                math.atan2(-ay, az)
            ) if abs(az) > 0.001 else 0.0
            self._imu_has_quaternion = False

    def _on_imu_yaw(self, msg: Float64) -> None:
        self._imu_yaw_deg = msg.data

    def _on_imu_cal(self, msg: Bool) -> None:
        self._imu_calibrated = msg.data

    def _on_imu_temp(self, msg: Float32) -> None:
        self._imu_temperature = msg.data

    def _on_imu_mic(self, msg: Int32) -> None:
        self._imu_mic_level = msg.data

    def _on_battery_state(self, msg: BatteryState) -> None:
        self._battery_voltage = msg.voltage
        self._battery_percentage = msg.percentage * 100.0  # BatteryState uses 0.0–1.0
        self._battery_present = msg.present
        self._battery_last_monotonic = time.monotonic()

    def _on_bno_cal(self, msg: String) -> None:
        """Receive BNO055 calibration JSON from imu/calibration."""
        try:
            data = json.loads(msg.data)
            self._bno_cal = {
                'sys': int(data.get('sys', 0)),
                'gyro': int(data.get('gyro', 0)),
                'accel': int(data.get('accel', 0)),
                'mag': int(data.get('mag', 0)),
            }
            self._bno_cal_available = True
        except Exception:
            pass

    # ── BNO055 auto-calibration state machine ─────────────────────

    _AUTOCAL_SPIN_SPEED = 0.35      # rad/s — slow rotation
    _AUTOCAL_SPIN_TIME = 20.0       # seconds — full slow spin for mag
    _AUTOCAL_TILT_SPEED = 0.25      # m/s — gentle forward/back tilt
    _AUTOCAL_TILT_TIME = 3.0        # seconds per tilt direction
    _AUTOCAL_STILL_TIME = 4.0       # seconds — hold still for gyro
    _AUTOCAL_WAIT_SYS_TIME = 10.0   # seconds — wait for sys to converge
    _AUTOCAL_TIMEOUT = 90.0         # overall timeout

    def start_autocal(self) -> dict:
        """Begin the BNO055 auto-calibration routine."""
        if not self._bno_cal_available:
            return {'ok': False, 'error': 'BNO055 not available'}
        if self._autocal_state not in ('idle', 'done', 'failed'):
            return {'ok': False, 'error': f'Already running ({self._autocal_state})'}
        self._autocal_state = 'spin_mag'
        self._autocal_phase_start = time.monotonic()
        self._autocal_status_msg = 'Starting magnetometer calibration…'
        self._autocal_spin_dir = 1.0
        self._autocal_tilt_phase = 0
        self.get_logger().info('BNO055 auto-calibration started')
        return {'ok': True, 'state': 'spin_mag'}

    def stop_autocal(self) -> None:
        """Cancel auto-calibration and stop motors."""
        if self._autocal_state not in ('idle', 'done', 'failed'):
            self.get_logger().info('BNO055 auto-calibration cancelled')
        self._autocal_state = 'idle'
        self._autocal_status_msg = ''
        self.stop_cmd_vel()

    def get_autocal_state(self) -> dict:
        """Return current auto-cal status for the UI."""
        return {
            'state': self._autocal_state,
            'message': self._autocal_status_msg,
            'bno_cal': dict(self._bno_cal),
            'bno_available': self._bno_cal_available,
        }

    def _autocal_tick(self) -> None:
        """10 Hz state machine for BNO055 auto-calibration."""
        state = self._autocal_state
        if state == 'idle' or state == 'done' or state == 'failed':
            return

        now = time.monotonic()
        elapsed = now - self._autocal_phase_start
        cal = self._bno_cal

        # Overall timeout
        if state != 'saving' and elapsed > self._AUTOCAL_TIMEOUT:
            self._autocal_state = 'failed'
            self._autocal_status_msg = (
                f'Timeout — cal: sys={cal["sys"]} gyro={cal["gyro"]} '
                f'accel={cal["accel"]} mag={cal["mag"]}. '
                'Try again or calibrate manually.'
            )
            self.stop_cmd_vel()
            self.get_logger().warn(f'Auto-cal timeout: {self._autocal_status_msg}')
            return

        # ── Phase 1: Slow spin for magnetometer ──────────────────
        if state == 'spin_mag':
            if cal['mag'] >= 3:
                self.get_logger().info(f'Mag calibrated (mag={cal["mag"]}) — moving to accel phase')
                self.stop_cmd_vel()
                self._autocal_state = 'tilt_accel'
                self._autocal_phase_start = now
                self._autocal_tilt_phase = 0
                self._autocal_tilt_start = now
                return
            # Alternate spin direction every 8 seconds
            spin_cycle = int(elapsed / 8.0)
            self._autocal_spin_dir = 1.0 if (spin_cycle % 2 == 0) else -1.0
            msg = Twist()
            msg.angular.z = self._autocal_spin_dir * self._AUTOCAL_SPIN_SPEED
            self._cmd_pub.publish(msg)
            self._cmd_last_rx_monotonic = now
            self._cmd_last_sent_monotonic = now
            progress = min(100, int(elapsed / self._AUTOCAL_SPIN_TIME * 100))
            self._autocal_status_msg = (
                f'Spinning for magnetometer… mag={cal["mag"]}/3 ({progress}%)'
            )
            # Time-based fallback: move on even if mag hasn't reached 3
            if elapsed >= self._AUTOCAL_SPIN_TIME:
                self.get_logger().info(f'Spin phase done (mag={cal["mag"]}) — moving to accel')
                self.stop_cmd_vel()
                self._autocal_state = 'tilt_accel'
                self._autocal_phase_start = now
                self._autocal_tilt_phase = 0
                self._autocal_tilt_start = now
            return

        # ── Phase 2: Gentle tilts for accelerometer ──────────────
        if state == 'tilt_accel':
            if cal['accel'] >= 3:
                self.get_logger().info(f'Accel calibrated (accel={cal["accel"]}) — moving to gyro phase')
                self.stop_cmd_vel()
                self._autocal_state = 'still_gyro'
                self._autocal_phase_start = now
                return
            tilt_elapsed = now - self._autocal_tilt_start
            # Sub-phases: forward, back, left, right, stop (each TILT_TIME seconds)
            directions = [
                ('Forward tilt…', 0.15, 0.0, 0.0),
                ('Backward tilt…', -0.15, 0.0, 0.0),
                ('Left strafe…', 0.0, 0.15, 0.0),
                ('Right strafe…', 0.0, -0.15, 0.0),
                ('Pause…', 0.0, 0.0, 0.0),
            ]
            if self._autocal_tilt_phase >= len(directions):
                # All tilt sub-phases done
                self.get_logger().info(f'Tilt phase done (accel={cal["accel"]}) — moving to gyro')
                self.stop_cmd_vel()
                self._autocal_state = 'still_gyro'
                self._autocal_phase_start = now
                return
            label, lx, ly, az = directions[self._autocal_tilt_phase]
            msg = Twist()
            msg.linear.x = float(lx)
            msg.linear.y = float(ly)
            msg.angular.z = float(az)
            self._cmd_pub.publish(msg)
            self._cmd_last_rx_monotonic = now
            self._cmd_last_sent_monotonic = now
            self._autocal_status_msg = (
                f'{label} accel={cal["accel"]}/3 '
                f'(step {self._autocal_tilt_phase + 1}/{len(directions)})'
            )
            if tilt_elapsed >= self._AUTOCAL_TILT_TIME:
                self._autocal_tilt_phase += 1
                self._autocal_tilt_start = now
            return

        # ── Phase 3: Hold still for gyroscope ────────────────────
        if state == 'still_gyro':
            if cal['gyro'] >= 3:
                self.get_logger().info(f'Gyro calibrated (gyro={cal["gyro"]}) — waiting for sys')
                self._autocal_state = 'wait_sys'
                self._autocal_phase_start = now
                return
            # Just keep still
            self._autocal_status_msg = (
                f'Holding still for gyroscope… gyro={cal["gyro"]}/3'
            )
            if elapsed >= self._AUTOCAL_STILL_TIME:
                self.get_logger().info(f'Still phase done (gyro={cal["gyro"]}) — waiting for sys')
                self._autocal_state = 'wait_sys'
                self._autocal_phase_start = now
            return

        # ── Phase 4: Wait for system calibration to converge ─────
        if state == 'wait_sys':
            all_ok = (cal['sys'] >= 1 and cal['gyro'] >= 2
                      and cal['accel'] >= 1 and cal['mag'] >= 1)
            fully_cal = (cal['sys'] >= 3 and cal['gyro'] >= 3
                         and cal['accel'] >= 3 and cal['mag'] >= 3)
            self._autocal_status_msg = (
                f'Waiting for fusion… sys={cal["sys"]}/3 '
                f'gyro={cal["gyro"]}/3 accel={cal["accel"]}/3 '
                f'mag={cal["mag"]}/3'
            )
            if all_ok or fully_cal or elapsed >= self._AUTOCAL_WAIT_SYS_TIME:
                self.get_logger().info(
                    f'Auto-cal complete — sys={cal["sys"]} gyro={cal["gyro"]} '
                    f'accel={cal["accel"]} mag={cal["mag"]} — saving…'
                )
                self._autocal_state = 'saving'
                self._autocal_phase_start = now
                # Send save command
                save_msg = Empty()
                self._imu_save_cal_pub.publish(save_msg)
            return

        # ── Phase 5: Save and finish ─────────────────────────────
        if state == 'saving':
            if elapsed >= 1.0:
                self._autocal_state = 'done'
                self._autocal_status_msg = (
                    f'✅ Calibration saved! sys={cal["sys"]} '
                    f'gyro={cal["gyro"]} accel={cal["accel"]} '
                    f'mag={cal["mag"]}'
                )
                self.get_logger().info(f'Auto-cal done: {self._autocal_status_msg}')
            else:
                self._autocal_status_msg = 'Saving calibration to flash…'
            return

    def get_imu_dict(self) -> dict:
        now = time.monotonic()
        age = None
        if self._imu_last_monotonic > 0.0:
            age = max(0.0, now - self._imu_last_monotonic)
        return {
            'accel_x': round(self._imu_accel[0], 3),
            'accel_y': round(self._imu_accel[1], 3),
            'accel_z': round(self._imu_accel[2], 3),
            'gyro_x': round(self._imu_gyro[0], 2),
            'gyro_y': round(self._imu_gyro[1], 2),
            'gyro_z': round(self._imu_gyro[2], 2),
            'temperature': round(self._imu_temperature, 1),
            'mic_level': int(self._imu_mic_level),
            'yaw_deg': round(self._imu_yaw_deg, 2),
            'pitch_deg': round(self._imu_pitch_deg, 2),
            'roll_deg': round(self._imu_roll_deg, 2),
            'has_quaternion': bool(self._imu_has_quaternion),
            'calibrated': bool(self._imu_calibrated),
            'last_age_s': round(age, 3) if age is not None else None,
            'rotate_active': bool(self._rotate_active),
            'rotate_target_deg': self._rotate_target_deg,
            'bno_cal': dict(self._bno_cal),
            'bno_available': self._bno_cal_available,
            'autocal': self.get_autocal_state(),
        }

    def imu_calibrate(self) -> None:
        """Trigger gyro re-calibration on the Arduino."""
        msg = Empty()
        self._imu_calibrate_pub.publish(msg)
        self.get_logger().info("IMU calibration requested via web UI")

    # ── Audio streaming ─────────────────────────────────────────

    def _on_audio(self, msg: UInt8MultiArray) -> None:
        if self._audio_buffer is not None and msg.data:
            self._audio_buffer.push(bytes(msg.data))

    def audio_enable(self) -> None:
        msg = Bool()
        msg.data = True
        self._audio_enable_pub.publish(msg)
        self._audio_enabled = True
        self.get_logger().info("Audio streaming enabled via web UI")

    def audio_disable(self) -> None:
        msg = Bool()
        msg.data = False
        self._audio_enable_pub.publish(msg)
        self._audio_enabled = False
        self.get_logger().info("Audio streaming disabled via web UI")

    # ── Depth estimation control ──────────────────────────────────

    def _on_depth_img(self, msg: CompressedImage) -> None:
        if not msg.data or self._depth_frame_buffer is None:
            return
        self._depth_frame_buffer.set(bytes(msg.data))
        self._depth_frame_count += 1
        if self._depth_frame_count == 1:
            self.get_logger().info(f"First depth frame received (bytes={len(msg.data)})")

    def depth_enable(self) -> None:
        msg = Bool()
        msg.data = True
        self._depth_enable_pub.publish(msg)
        self._depth_enabled = True
        self.get_logger().info("Depth estimation enabled via web UI")

    def depth_disable(self) -> None:
        msg = Bool()
        msg.data = False
        self._depth_enable_pub.publish(msg)
        self._depth_enabled = False
        self.get_logger().info("Depth estimation disabled via web UI")

    # ── Collision failsafe control ────────────────────────────────────

    def _on_collision_active(self, msg: Bool) -> None:
        self._collision_failsafe_active = msg.data

    def _on_ultrasonic_range(self, msg: Range) -> None:
        import math
        d = msg.range
        if math.isnan(d) or math.isinf(d):
            self._collision_distance_m = float('inf')
        else:
            self._collision_distance_m = max(d, 0.0)

    def collision_failsafe_enable(self) -> None:
        msg = Bool()
        msg.data = True
        self._cf_enable_pub.publish(msg)
        self._collision_failsafe_enabled = True
        self.get_logger().info("Collision failsafe enabled via web UI")

    def collision_failsafe_disable(self) -> None:
        msg = Bool()
        msg.data = False
        self._cf_enable_pub.publish(msg)
        self._collision_failsafe_enabled = False
        self._collision_failsafe_active = False
        self.get_logger().info("Collision failsafe disabled via web UI")

    # ── Cliff failsafe control ────────────────────────────────────

    def _on_cliff_active(self, msg: Bool) -> None:
        self._cliff_failsafe_active = msg.data

    def _on_cliff_tracking(self, msg: Int32) -> None:
        self._cliff_sensor_state = msg.data

    def cliff_failsafe_enable(self) -> None:
        msg = Bool()
        msg.data = True
        self._cl_enable_pub.publish(msg)
        self._cliff_failsafe_enabled = True
        self.get_logger().info("Cliff failsafe enabled via web UI")

    def cliff_failsafe_disable(self) -> None:
        msg = Bool()
        msg.data = False
        self._cl_enable_pub.publish(msg)
        self._cliff_failsafe_enabled = False
        self._cliff_failsafe_active = False
        self.get_logger().info("Cliff failsafe disabled via web UI")

    # ── Odometry integration ──────────────────────────────────────

    def _on_odom(self, msg: Odometry) -> None:
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        self._odom_vx = msg.twist.twist.linear.x
        self._odom_vy = msg.twist.twist.linear.y
        self._odom_wz = msg.twist.twist.angular.z
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._odom_yaw_deg = math.degrees(math.atan2(siny, cosy))
        self._odom_last_monotonic = time.monotonic()
        # Live breadcrumb trail (keep every ~5cm or so, cap at 2000 points)
        trail = self._odom_trail_xy
        if not trail or (
            (self._odom_x - trail[-1][0]) ** 2 +
            (self._odom_y - trail[-1][1]) ** 2 > 0.0025
        ):
            trail.append((self._odom_x, self._odom_y))
            if len(trail) > 2000:
                self._odom_trail_xy = trail[-2000:]

    def _on_odom_path(self, msg: Path) -> None:
        pts = []
        for ps in msg.poses:
            pts.append((ps.pose.position.x, ps.pose.position.y))
        self._odom_path_xy = pts

    def _on_odom_recording(self, msg: Bool) -> None:
        self._odom_recording = msg.data

    def _on_odom_returning(self, msg: Bool) -> None:
        self._odom_returning = msg.data

    def _on_odom_stuck(self, msg: Bool) -> None:
        self._odom_stuck = msg.data

    def _on_odom_nav_status(self, msg: String) -> None:
        # e.g. "navigating 2/5", "nav: arrived", "nav: cancelled", "idle"
        txt = msg.data
        self._odom_navigating = txt.startswith('navigating')
        # Parse waypoint progress: "navigating 2/5"
        if txt.startswith('navigating') and '/' in txt:
            try:
                parts = txt.split()[-1].split('/')
                self._odom_nav_wp_idx = int(parts[0])
                self._odom_nav_wp_total = int(parts[1])
            except Exception:
                pass
        elif not self._odom_navigating:
            self._odom_nav_wp_idx = 0
            self._odom_nav_wp_total = 0

    def get_odom_dict(self) -> dict:
        now = time.monotonic()
        age = None
        if self._odom_last_monotonic > 0.0:
            age = round(max(0.0, now - self._odom_last_monotonic), 3)
        return {
            'x': round(self._odom_x, 4),
            'y': round(self._odom_y, 4),
            'yaw_deg': round(self._odom_yaw_deg, 2),
            'vx': round(self._odom_vx, 3),
            'vy': round(self._odom_vy, 3),
            'wz': round(self._odom_wz, 3),
            'recording': bool(self._odom_recording),
            'returning': bool(self._odom_returning),
            'stuck': bool(self._odom_stuck),
            'navigating': bool(self._odom_navigating),
            'nav_wp_idx': self._odom_nav_wp_idx,
            'nav_wp_total': self._odom_nav_wp_total,
            'trail': self._downsample_trail(200),  # downsampled for network efficiency
            'path': self._odom_path_xy,
            'last_age_s': age,
        }

    def _downsample_trail(self, max_points: int = 200) -> list:
        trail = self._odom_trail_xy
        n = len(trail)
        if n <= max_points:
            return list(trail)
        # Always keep first and last, evenly sample the rest
        step = (n - 1) / (max_points - 1)
        return [trail[int(i * step)] for i in range(max_points)]

    def _call_trigger_service(self, client, name: str) -> dict:
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=0.3):
                self.get_logger().warn(f"[odom] service {name} not available")
                return {'ok': False, 'message': f'Service {name} not available'}
        req = Trigger.Request()
        future = client.call_async(req)
        # Poll for completion — the MultiThreadedExecutor handles the callback
        deadline = time.monotonic() + 2.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if future.done() and future.result() is not None:
            result = future.result()
            return {'ok': result.success, 'message': result.message}
        return {'ok': False, 'message': 'Service call timed out'}

    def odom_set_origin(self) -> dict:
        self._odom_trail_xy.clear()
        self._odom_path_xy.clear()
        return self._call_trigger_service(self._odom_set_origin_cli, 'odom/set_origin')

    def odom_start_recording(self) -> dict:
        return self._call_trigger_service(self._odom_start_recording_cli, 'odom/start_recording')

    def odom_stop_recording(self) -> dict:
        return self._call_trigger_service(self._odom_stop_recording_cli, 'odom/stop_recording')

    def odom_return_to_origin(self) -> dict:
        return self._call_trigger_service(self._odom_return_to_origin_cli, 'odom/return_to_origin')

    def odom_cancel_return(self) -> dict:
        return self._call_trigger_service(self._odom_cancel_return_cli, 'odom/cancel_return')

    def odom_navigate_to(self, x: float, y: float) -> dict:
        from geometry_msgs.msg import Point
        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = 0.0
        self._odom_navigate_to_pub.publish(msg)
        self.get_logger().info(f"[odom] Published navigate_to ({x:.3f}, {y:.3f})")
        return {'ok': True, 'message': f'Navigating to ({x:.2f}, {y:.2f})'}

    def odom_navigate_path(self, waypoints: list) -> dict:
        from nav_msgs.msg import Path as NavPath
        from geometry_msgs.msg import PoseStamped
        msg = NavPath()
        msg.header.frame_id = 'odom'
        for wp in waypoints:
            ps = PoseStamped()
            ps.pose.position.x = float(wp['x'])
            ps.pose.position.y = float(wp['y'])
            msg.poses.append(ps)
        self._odom_navigate_path_pub.publish(msg)
        self.get_logger().info(f"[odom] Published navigate_path with {len(waypoints)} waypoints")
        return {'ok': True, 'message': f'Navigating {len(waypoints)}-waypoint path'}

    def odom_cancel_navigate(self) -> dict:
        return self._call_trigger_service(self._odom_cancel_navigate_cli, 'odom/cancel_navigate')

    # ── LiDAR scan integration ────────────────────────────────────

    def _on_scan(self, msg: LaserScan) -> None:
        n = len(msg.ranges)
        # Vectorize angle generation
        self._scan_angles = np.linspace(msg.angle_min, msg.angle_max, n)
        # Store ranges as numpy array, handling infinities if needed (though JS handles null/inf often)
        # but for safety, let's keep them as is, just numpy-ified.
        # Note: msg.ranges is a tuple/array.array usually.
        self._scan_ranges = np.array(msg.ranges, dtype=np.float32)

        self._scan_angle_min = msg.angle_min
        self._scan_angle_max = msg.angle_max
        self._scan_range_min = msg.range_min
        self._scan_range_max = msg.range_max
        self._scan_last_monotonic = time.monotonic()

    def get_scan_dict(self) -> dict:
        now = time.monotonic()
        age = None
        if self._scan_last_monotonic > 0.0:
            age = round(max(0.0, now - self._scan_last_monotonic), 3)

        # Optimize: Use numpy for fast rounding if available, else fallback
        # (Though we expect numpy to be available in this environment)
        if self._scan_ranges is not None and len(self._scan_ranges) > 0:
            # Check if it's already a numpy array (from _on_scan optimization)
            if isinstance(self._scan_ranges, np.ndarray):
                ranges = np.round(self._scan_ranges, 4).tolist()
            else:
                 ranges = [round(r, 4) for r in self._scan_ranges]
        else:
             ranges = []

        if self._scan_angles is not None and len(self._scan_angles) > 0:
             if isinstance(self._scan_angles, np.ndarray):
                 angles = np.round(self._scan_angles, 4).tolist()
             else:
                 angles = [round(a, 4) for a in self._scan_angles]
        else:
             angles = []

        return {
            'angles': angles,
            'ranges': ranges,
            'angle_min': round(self._scan_angle_min, 4),
            'angle_max': round(self._scan_angle_max, 4),
            'range_min': round(self._scan_range_min, 4),
            'range_max': round(self._scan_range_max, 4),
            'last_age_s': age,
        }

    # ── SLAM map callbacks ────────────────────────────────────────────

    def _on_map(self, msg: OccupancyGrid) -> None:
        """Store latest occupancy grid for /api/map endpoint."""
        # Store as numpy array immediately for efficiency
        self._map_data = np.array(msg.data, dtype=np.int8)
        self._map_width = msg.info.width
        self._map_height = msg.info.height
        self._map_resolution = msg.info.resolution
        self._map_origin_x = msg.info.origin.position.x
        self._map_origin_y = msg.info.origin.position.y
        self._map_last_monotonic = time.monotonic()
        self._map_dirty = True

    def get_map_dict(self) -> dict:
        """Build map data dict for /api/map endpoint."""
        now = time.monotonic()
        if self._map_data is None or self._map_last_monotonic == 0.0:
            return {'available': False}

        # Only re-encode when map data changes
        if self._map_dirty:
            import base64
            # Vectorized implementation using numpy
            # Map data is int8 (-1..100).
            # Logic: 255 if v < 0 else min(v, 100)
            
            # Ensure it is a numpy array (it should be from _on_map)
            arr = self._map_data
            if not isinstance(arr, np.ndarray):
                arr = np.array(arr, dtype=np.int8)
            
            # Mask for unknown (-1) -> 255 (uint8)
            # We can use a temporary buffer or create a new one.
            # Output needs to be uint8 for base64
            
            # 1. Create output array initialized to same values
            # Cast to int16 temporarily to avoid overflow if needed, or just smart masking
            # Actually, standard map is -1 (unknown), 0-100 (prob).
            # We want -1 -> 255. 0-100 -> 0-100.
            
            # Use np.where is robust.
            # unknown_mask = (arr < 0)
            # occupied_mask = (arr > 100) # Should not happen in standard map, but clean it.
            
            # Create a clean uint8 array
            # -1 becomes 255 when cast to uint8? No, -1 is 255 in int8 two's complement, 
            # but we need to be careful.
            # Let's use explicit logic for clarity and speed.
            
            processed = np.copy(arr)
            
            # Clamp 0..100
            # Note: -1 is < 0, so it's not affected by min(100) if we did that.
            # But we want -1 -> 255.
            
            # Fast way:
            # 1. replace -1 with 255? 
            # In int8, -1 is signed. 255 is not representable.
            # So we must cast to uint8 first? 
            # If we cast -1 (int8) to uint8, it becomes 255!
            # If we cast 100 (int8) to uint8, it becomes 100.
            # So simple cast might work for -1 case!
            
            # However, we also want `min(v, 100)`.
            # If v is 120 (invalid prob), it should be 100.
            
            # Safer approach:
            out = np.zeros_like(arr, dtype=np.uint8)
            
            # Copy valid data: 0 <= v <= 100
            valid_mask = (arr >= 0) & (arr <= 100)
            out[valid_mask] = arr[valid_mask]
            
            # Handle > 100 (clamp to 100)
            high_mask = (arr > 100)
            out[high_mask] = 100
            
            # Handle < 0 (unknown) -> 255
            # Already zeros? No, we need 255.
            unknown_mask = (arr < 0)
            out[unknown_mask] = 255
            
            # Convert to bytes
            data_bytes = out.tobytes()
            
            self._map_b64_cache = base64.b64encode(data_bytes).decode('ascii')
            self._map_dirty = False

        age = round(max(0.0, now - self._map_last_monotonic), 3)
        return {
            'available': True,
            'width': self._map_width,
            'height': self._map_height,
            'resolution': round(self._map_resolution, 4),
            'origin_x': round(self._map_origin_x, 4),
            'origin_y': round(self._map_origin_y, 4),
            'data': self._map_b64_cache,
            'age_s': age,
        }

    def slam_reset(self) -> dict:
        """Call slam/reset service."""
        if not self._slam_reset_cli.wait_for_service(timeout_sec=0.5):
            return {'success': False, 'message': 'SLAM service not available'}
        req = Trigger.Request()
        future = self._slam_reset_cli.call_async(req)
        deadline = time.time() + 2.0
        while time.time() < deadline:
            if future.done():
                result = future.result()
                return {'success': result.success, 'message': result.message}
            time.sleep(0.02)
        return {'success': False, 'message': 'Timeout waiting for slam/reset'}

    # ── Front calibration service calls ────────────────────────────────

    def calibrate_front(self) -> dict:
        """Call calibrate_front (full) service."""
        if not self._cal_front_cli.wait_for_service(timeout_sec=0.5):
            return {'success': False, 'message': 'Calibration service not available'}
        req = Trigger.Request()
        future = self._cal_front_cli.call_async(req)
        deadline = time.time() + 30.0
        while time.time() < deadline:
            if future.done():
                result = future.result()
                return {'success': result.success, 'message': result.message}
            time.sleep(0.05)
        return {'success': False, 'message': 'Timeout waiting for calibration'}

    def calibrate_front_static(self) -> dict:
        """Call calibrate_front_static service."""
        if not self._cal_static_cli.wait_for_service(timeout_sec=0.5):
            return {'success': False, 'message': 'Static calibration service not available'}
        req = Trigger.Request()
        future = self._cal_static_cli.call_async(req)
        deadline = time.time() + 15.0
        while time.time() < deadline:
            if future.done():
                result = future.result()
                return {'success': result.success, 'message': result.message}
            time.sleep(0.05)
        return {'success': False, 'message': 'Timeout waiting for static calibration'}

    # ── LiDAR obstacle zone callbacks ─────────────────────────────────

    def _on_lidar_front(self, msg: Range) -> None:
        self._lidar_front_range = float(msg.range) if not math.isinf(msg.range) else 99.0
        self._lidar_zones_last = time.monotonic()

    def _on_lidar_left(self, msg: Range) -> None:
        self._lidar_left_range = float(msg.range) if not math.isinf(msg.range) else 99.0

    def _on_lidar_right(self, msg: Range) -> None:
        self._lidar_right_range = float(msg.range) if not math.isinf(msg.range) else 99.0

    def _on_lidar_rear(self, msg: Range) -> None:
        self._lidar_rear_range = float(msg.range) if not math.isinf(msg.range) else 99.0

    def get_lidar_zones_dict(self) -> dict:
        """Build obstacle zone dict for /api/lidar_zones."""
        now = time.monotonic()
        age = None
        if self._lidar_zones_last > 0.0:
            age = round(max(0.0, now - self._lidar_zones_last), 3)
        return {
            'front': round(self._lidar_front_range, 3),
            'left': round(self._lidar_left_range, 3),
            'right': round(self._lidar_right_range, 3),
            'rear': round(self._lidar_rear_range, 3),
            'age_s': age,
        }

    # ── Rotate-to-angle controller ────────────────────────────────────

    def start_rotate(self, target_deg: float, speed: float = 0.5) -> None:
        target_deg = max(-180.0, min(180.0, float(target_deg)))
        speed = max(0.1, min(1.0, float(speed)))
        self._rotate_target_deg = target_deg
        self._rotate_speed = speed
        self._rotate_active = True
        self._rotate_in_tolerance_since = None
        self._rotate_integral = 0.0
        self._rotate_prev_error = None
        self._rotate_last_time = time.monotonic()
        self._rotate_last_output = 0.0
        self.get_logger().info(f"Rotate to {target_deg:.1f}° at speed {speed:.1f}")

    def stop_rotate(self) -> None:
        self._rotate_active = False
        self._rotate_target_deg = None
        self._rotate_in_tolerance_since = None
        self._rotate_integral = 0.0
        self._rotate_prev_error = None
        self._rotate_last_output = 0.0
        # Send a stop command
        msg = Twist()
        self._cmd_pub.publish(msg)

    def _rotate_tick(self) -> None:
        if not self._rotate_active or self._rotate_target_deg is None:
            return
        if not self._imu_calibrated:
            return

        target = self._rotate_target_deg
        current = self._imu_yaw_deg

        # Compute shortest angular error (±180)
        error = target - current
        while error > 180.0:
            error -= 360.0
        while error < -180.0:
            error += 360.0

        now = time.monotonic()
        dt = now - self._rotate_last_time if self._rotate_last_time > 0 else 0.05
        dt = min(dt, 0.1)  # safety cap
        self._rotate_last_time = now

        yaw_rate = self._imu_gyro[2]  # °/s from IMU

        # Check if within tolerance — also require low yaw rate to avoid
        # counting a fast pass-through as "settled".
        if abs(error) < self._rotate_tolerance and abs(yaw_rate) < 8.0:
            if self._rotate_in_tolerance_since is None:
                self._rotate_in_tolerance_since = now
            elif (now - self._rotate_in_tolerance_since) >= self._rotate_settle_sec:
                # Done!
                self.get_logger().info(
                    f"Rotation complete: target={target:.1f}°, current={current:.1f}°, error={error:.1f}°"
                )
                self._rotate_active = False
                self._rotate_integral = 0.0
                self._rotate_last_output = 0.0
                msg = Twist()
                self._cmd_pub.publish(msg)
                return
        else:
            self._rotate_in_tolerance_since = None

        # ── Normalised PID controller ─────────────────────────────
        # Both error and yaw rate are normalised to [−1, 1] so that
        # the P and D gains are directly comparable and the D term
        # cannot overwhelm P at approach speeds.
        ramp = max(self._rotate_ramp_deg, 1.0)
        max_rate_dps = self._rotate_speed * self._max_angular_rps * 57.2958
        if max_rate_dps < 1.0:
            max_rate_dps = 1.0

        error_norm = error / ramp            # [-1, 1] at ± ramp_deg
        rate_norm = yaw_rate / max_rate_dps  # [-1, 1] at max slew rate

        # P: proportional to normalised heading error
        p_term = self._rotate_kp * error_norm

        # I: only accumulate near the target to overcome friction
        #    during final settling — prevents integral windup during
        #    the high-speed slew phase.
        near_target = abs(error) < self._rotate_tolerance * 3.0  # within ~6°
        if near_target:
            self._rotate_integral += error_norm * dt
            self._rotate_integral = max(-self._rotate_integral_max,
                                         min(self._rotate_integral_max,
                                             self._rotate_integral))
        elif abs(error) > self._rotate_tolerance * 6.0:
            # Far from target: reset integral to prevent stale buildup
            self._rotate_integral = 0.0
        # Reduce integral on zero-crossing to cut overshoot
        if (self._rotate_prev_error is not None
                and error * self._rotate_prev_error < 0):
            self._rotate_integral *= 0.3
        self._rotate_prev_error = error
        i_term = self._rotate_ki * self._rotate_integral

        # D: gyro-rate braking, normalised so D=kd when rate=max_rate
        d_term = -self._rotate_kd * rate_norm

        angular_unit = p_term + i_term + d_term

        # Clamp to requested speed
        angular_unit = max(-self._rotate_speed,
                           min(self._rotate_speed, angular_unit))

        # Apply minimum output to overcome motor dead-band near target.
        # Only when actively correcting (not within tolerance settling).
        if abs(error) >= self._rotate_tolerance * 0.3:
            min_out = self._rotate_min_output
            if 0.0 < abs(angular_unit) < min_out:
                angular_unit = min_out if angular_unit > 0 else -min_out

        self._rotate_last_output = angular_unit

        msg = Twist()
        msg.angular.z = float(angular_unit) * float(self._max_angular_rps)
        self._cmd_pub.publish(msg)
        self._cmd_last_rx_monotonic = now
        self._cmd_last_sent_monotonic = now


def make_handler(
    *,
    frame_buffer: FrameBuffer,
    front_frame_buffer: FrameBuffer = None,
    depth_frame_buffer: FrameBuffer = None,
    fps_limit: float,
    depth_fps_limit: float,
    logger,
    status_provider=None,
    detections_provider=None,
    detections_json_provider=None,
    gimbal_setter=None,
    gimbal_center=None,
    cmd_vel_setter=None,
    cmd_vel_stopper=None,
    tracking_setter=None,
    tracking_config_setter=None,
    person_select_setter=None,
    follow_setter=None,
    follow_config_setter=None,
    snapshot_taker=None,
    snapshot_dir=None,
    lightbar_setter=None,
    imu_provider=None,
    imu_calibrate=None,
    autocal_starter=None,
    autocal_stopper=None,
    rotate_starter=None,
    rotate_stopper=None,
    audio_buffer=None,
    audio_enabler=None,
    audio_disabler=None,
    depth_enabler=None,
    depth_disabler=None,
    collision_failsafe_enabler=None,
    collision_failsafe_disabler=None,
    cliff_failsafe_enabler=None,
    cliff_failsafe_disabler=None,
    odom_provider=None,
    odom_set_origin=None,
    odom_start_recording=None,
    odom_stop_recording=None,
    odom_return_to_origin=None,
    odom_cancel_return=None,
    odom_navigate_to=None,
    odom_navigate_path=None,
    odom_cancel_navigate=None,
    scan_provider=None,
    map_provider=None,
    lidar_zones_provider=None,
    slam_reset=None,
    calibrate_front=None,
    calibrate_front_static=None,
    face_list=None,
    face_rename=None,
    face_delete=None,
    face_merge=None,
    face_clear=None,
    face_thumbnail=None,
):
    boundary = b"--frame"
    fps = max(float(fps_limit), 1.0)
    min_period = 1.0 / fps
    depth_fps = max(float(depth_fps_limit), 1.0)
    depth_min_period = 1.0 / depth_fps

    # ── Low-resolution JPEG resize helper (for Cardputer / embedded clients) ─
    _cv2 = None
    try:
        import cv2 as _cv2_mod  # type: ignore
        _cv2 = _cv2_mod
    except ImportError:
        if logger is not None:
            logger.warn("cv2 not available — /stream_*_lo.mjpg endpoints disabled")

    def _resize_jpeg(jpeg_bytes: bytes, target_w: int = 320, target_h: int = 240,
                     quality: int = 50) -> Optional[bytes]:
        """Decode JPEG, resize, re-encode at lower quality.  Returns None on error."""
        if _cv2 is None:
            return None
        try:
            arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
            img = _cv2.imdecode(arr, _cv2.IMREAD_COLOR)
            if img is None:
                return None
            resized = _cv2.resize(img, (target_w, target_h), interpolation=_cv2.INTER_AREA)
            ok, enc = _cv2.imencode('.jpg', resized, [_cv2.IMWRITE_JPEG_QUALITY, quality])
            if not ok:
                return None
            return bytes(enc)
        except Exception:
            return None

    # ── Shared low-res frame cache (avoids per-client decode+resize+encode) ──
    # Key: (source_id, w, h, q)  Value: (source_stamp, jpeg_bytes)
    _lo_cache: dict = {}
    _lo_cache_lock = threading.Lock()

    def _get_cached_lo(source_id: str, source_stamp: float,
                       jpeg_bytes: bytes, w: int, h: int, q: int) -> bytes:
        """Return cached low-res JPEG, resizing only on first access per source frame."""
        key = (source_id, w, h, q)
        with _lo_cache_lock:
            cached = _lo_cache.get(key)
            if cached is not None and cached[0] == source_stamp:
                return cached[1]
        # Resize outside the lock to avoid blocking other threads
        resized = _resize_jpeg(jpeg_bytes, w, h, q)
        if resized is None:
            resized = jpeg_bytes  # fallback: send original
        with _lo_cache_lock:
            _lo_cache[key] = (source_stamp, resized)
        return resized

    class Handler(BaseHTTPRequestHandler):
        # Use a large write buffer so each flush() sends one TCP segment
        # per frame instead of 3-4 small send() syscalls that thrash the GIL.
        # wbufsize = 65536  # DISABLED for debugging


        def setup(self):
            super().setup()
            # Disable Nagle algorithm — send frames immediately, don't
            # wait up to 40 ms trying to coalesce small writes.
            try:
                self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except Exception:
                pass

        def handle(self):
            # Suppress noisy BrokenPipeError tracebacks that the base class
            # prints to stderr when a stream client disconnects.
            try:
                super().handle()
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
                pass

        def finish(self):
            # Suppress errors during final wfile flush/close after stream disconnect.
            try:
                super().finish()
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
                pass

        def log_message(self, fmt, *args):
            # Quiet default logging; use ROS logger instead.
            if logger is not None:
                try:
                    logger.debug(fmt % args)
                except Exception:
                    pass

        def do_GET(self):
            parsed = urlparse(self.path)
            path = parsed.path

            if path in {"/", "/index.html"}:
                body = INDEX_HTML.encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-cache, must-revalidate")
                self.end_headers()
                self.wfile.write(body)
                try:
                    self.wfile.flush()
                except Exception:
                    pass
                return

            # ── Static file serving ──────────────────────────────────
            if path.startswith("/static/"):
                fname = path[len("/static/"):]
                if not fname or '/' in fname or '..' in fname or '\\' in fname:
                    self.send_response(HTTPStatus.BAD_REQUEST)
                    self.end_headers()
                    return
                fpath = os.path.join(_STATIC_DIR, fname)
                if not os.path.isfile(fpath):
                    self.send_response(HTTPStatus.NOT_FOUND)
                    self.end_headers()
                    return
                _ct_map = {'.css': 'text/css', '.js': 'application/javascript',
                           '.html': 'text/html'}
                ext = os.path.splitext(fname)[1].lower()
                ct = _ct_map.get(ext, 'application/octet-stream')
                with open(fpath, 'rb') as sf:
                    data = sf.read()

                # ETag from content hash — enables 304 Not Modified
                etag = '"' + hashlib.md5(data).hexdigest() + '"'
                if_none = self.headers.get('If-None-Match', '')
                if if_none == etag:
                    self.send_response(HTTPStatus.NOT_MODIFIED)
                    self.send_header("ETag", etag)
                    self.send_header("Cache-Control", "public, max-age=300, must-revalidate")
                    self.end_headers()
                    return

                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", ct + '; charset=utf-8')
                self.send_header("Content-Length", str(len(data)))
                self.send_header("ETag", etag)
                self.send_header("Cache-Control", "public, max-age=300, must-revalidate")
                self.end_headers()
                self.wfile.write(data)
                try:
                    self.wfile.flush()
                except Exception:
                    pass
                return

            # ── WebSocket endpoint ───────────────────────────────────
            if path == "/ws":
                upgrade = self.headers.get("Upgrade", "").lower()
                ws_key = self.headers.get("Sec-WebSocket-Key", "")
                if upgrade != "websocket" or not ws_key:
                    self.send_response(HTTPStatus.BAD_REQUEST)
                    self.end_headers()
                    return
                accept = _ws_accept_key(ws_key)
                self.protocol_version = "HTTP/1.1"
                self.send_response(101, "Switching Protocols")
                self.send_header("Upgrade", "websocket")
                self.send_header("Connection", "Upgrade")
                self.send_header("Sec-WebSocket-Accept", accept)
                self.end_headers()
                self.close_connection = True
                try:
                    self.wfile.flush()
                except Exception:
                    pass

                sock = self.request
                sock.setblocking(True)

                # Push intervals (seconds)
                FAST_IV = 0.333      # detections + imu
                SENSORS_IV = 0.5     # odom + lidar + lidar_zones
                STATUS_IV = 1.0      # status
                MAP_IV = 2.0         # map
                FACES_IV = 10.0      # face gallery

                next_due = [0.0, 0.0, 0.0, 0.0, 0.0]

                if logger:
                    logger.info("WebSocket client connected")

                try:
                    while True:
                        now = time.time()
                        payload = {}

                        if now >= next_due[0]:
                            next_due[0] = now + FAST_IV
                            # 1. Detections: Send as raw JSON string if possible
                            if callable(detections_json_provider):
                                try:
                                    # The provider returns a JSON string, e.g. '{"detections": [...], ...}'
                                    # We can send it directly.
                                    det_json = detections_json_provider()
                                    if det_json:
                                        _ws_send(sock, det_json.encode('utf-8'), 0x01)
                                except Exception:
                                    pass
                            
                            # 2. IMU: Send separately
                            if callable(imu_provider):
                                try:
                                    imu_data = imu_provider()
                                    if imu_data:
                                        _ws_send(sock, json.dumps({"imu": imu_data}).encode('utf-8'), 0x01)
                                except Exception:
                                    pass

                        if now >= next_due[1]:
                            next_due[1] = now + SENSORS_IV
                            sensors_payload = {}
                            if callable(odom_provider):
                                try:
                                    sensors_payload["odom"] = odom_provider()
                                except Exception:
                                    pass
                            if callable(scan_provider):
                                try:
                                    sensors_payload["lidar"] = scan_provider()
                                except Exception:
                                    pass
                            if callable(lidar_zones_provider):
                                try:
                                    sensors_payload["lidar_zones"] = lidar_zones_provider()
                                except Exception:
                                    pass
                            
                            if sensors_payload:
                                _ws_send(sock, json.dumps(sensors_payload).encode('utf-8'), 0x01)

                        if now >= next_due[2]:
                            next_due[2] = now + STATUS_IV
                            if callable(status_provider):
                                try:
                                    status_data = status_provider()
                                    _ws_send(sock, json.dumps({"status": status_data}).encode('utf-8'), 0x01)
                                except Exception:
                                    pass

                        if now >= next_due[3]:
                            next_due[3] = now + MAP_IV
                            if callable(map_provider):
                                try:
                                    map_data = map_provider()
                                    # Map data can be large! But it's base64 string inside the dict.
                                    # Still effective to send it alone.
                                    if map_data:
                                        _ws_send(sock, json.dumps({"map": map_data}).encode('utf-8'), 0x01)
                                except Exception:
                                    pass

                        if now >= next_due[4]:
                            next_due[4] = now + FACES_IV
                            if callable(face_list):
                                try:
                                    _ws_send(sock, json.dumps({"faces": face_list()}).encode('utf-8'), 0x01)
                                except Exception:
                                    pass

                        # Drain incoming frames (close / ping / pong)
                        sock.settimeout(0.05)
                        try:
                            result = _ws_recv(sock)
                            if result is None:
                                break
                            opcode, frame_data = result
                            if opcode == 0x08:  # close
                                try:
                                    _ws_send(sock, b"", 0x08)
                                except Exception:
                                    pass
                                break
                            elif opcode == 0x09:  # ping
                                _ws_send(sock, frame_data or b"", 0x0A)
                        except (socket.timeout, BlockingIOError):
                            pass
                        finally:
                            sock.settimeout(None)

                        sleep_until = min(next_due)
                        time.sleep(max(0.01, sleep_until - time.time()))

                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError) as exc:
                    if logger:
                        logger.warning(f"WebSocket network error: {exc}")
                except Exception as exc:
                    if logger:
                        import traceback
                        logger.error(f"WebSocket unexpected error: {exc}\n{traceback.format_exc()}")
                finally:
                    if logger:
                        logger.info("WebSocket client disconnected")
                return

            if path.startswith("/status"):
                payload = {}
                if callable(status_provider):
                    try:
                        payload = status_provider()
                    except Exception:
                        payload = {'error': 'status_provider_failed'}
                # Include map data in status response
                if callable(map_provider):
                    try:
                        payload['map'] = map_provider()
                    except Exception:
                        payload['map'] = {'available': False, 'error': 'map_provider_failed'}
                body = (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                try:
                    self.wfile.flush()
                except Exception:
                    pass
                return

            if path.startswith("/detections"):
                if callable(detections_json_provider):
                    try:
                        body = (detections_json_provider() + "\n").encode("utf-8")
                    except Exception:
                        body = b'{"detections": []}\n'
                else:
                    payload = {"detections": []}
                    if callable(detections_provider):
                        try:
                            payload = detections_provider()
                        except Exception:
                            payload = {'error': 'detections_provider_failed', 'detections': []}
                    body = (json.dumps(payload) + "\n").encode("utf-8")
                
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                try:
                    self.wfile.flush()
                except Exception:
                    pass
                return

            if path.startswith("/api/gimbal") and path.endswith("/set"):
                qs = parse_qs(parsed.query)
                try:
                    pan = float(qs.get('pan_deg', ['nan'])[0])
                    tilt = float(qs.get('tilt_deg', ['nan'])[0])
                except Exception:
                    pan = float('nan')
                    tilt = float('nan')
                if not callable(gimbal_setter):
                    self.send_response(HTTPStatus.NOT_IMPLEMENTED)
                    self.end_headers()
                    return
                if pan != pan or tilt != tilt:
                    self.send_response(HTTPStatus.BAD_REQUEST)
                    self.end_headers()
                    return
                try:
                    gimbal_setter(pan, tilt)
                except Exception:
                    self.send_response(HTTPStatus.INTERNAL_SERVER_ERROR)
                    self.end_headers()
                    return
                self.send_response(HTTPStatus.NO_CONTENT)
                self.end_headers()
                return

            if path == "/api/imu":
                payload = {}
                if callable(imu_provider):
                    try:
                        payload = imu_provider()
                    except Exception:
                        payload = {'error': 'imu_provider_failed'}
                body = (json.dumps(payload) + "\n").encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                self.wfile.write(body)
                try:
                    self.wfile.flush()
                except Exception:
                    pass
                return

            if path == "/api/odom":
                payload = {}
                if callable(odom_provider):
                    try:
                        payload = odom_provider()
                    except Exception:
                        payload = {'error': 'odom_provider_failed'}
                body = (json.dumps(payload) + "\n").encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                self.wfile.write(body)
                try:
                    self.wfile.flush()
                except Exception:
                    pass
                return

            if path == "/api/lidar":
                payload = {}
                if callable(scan_provider):
                    try:
                        payload = scan_provider()
                    except Exception:
                        payload = {'error': 'scan_provider_failed'}
                body = (json.dumps(payload) + "\n").encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                self.wfile.write(body)
                try:
                    self.wfile.flush()
                except Exception:
                    pass
                return

            if path == "/api/map":
                payload = {}
                if callable(map_provider):
                    try:
                        payload = map_provider()
                    except Exception:
                        payload = {'available': False, 'error': 'map_provider_failed'}
                body = (json.dumps(payload) + "\n").encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                self.wfile.write(body)
                try:
                    self.wfile.flush()
                except Exception:
                    pass
                return

            if path == "/api/lidar_zones":
                payload = {}
                if callable(lidar_zones_provider):
                    try:
                        payload = lidar_zones_provider()
                    except Exception:
                        payload = {'error': 'lidar_zones_provider_failed'}
                body = (json.dumps(payload) + "\n").encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                self.wfile.write(body)
                try:
                    self.wfile.flush()
                except Exception:
                    pass
                return

            # ── Combined endpoint: /api/fast (detections + imu) ──────
            # ── Combined endpoint: /api/fast (detections + imu) ──────
            if path == "/api/fast":
                # Manual string concatenation for speed
                det_part = '{"detections": []}'
                if callable(detections_json_provider):
                     det_part = detections_json_provider()
                elif callable(detections_provider):
                     try:
                         det_part = json.dumps(detections_provider())
                     except Exception:
                         pass
                
                imu_part = '"imu": {}'
                if callable(imu_provider):
                    try:
                        imu_part = '"imu": ' + json.dumps(imu_provider())
                    except Exception:
                        pass
                
                # Construct { "detections": ..., "imu": ... }
                # But det_part is {"detections": [...], ...} (entire object).
                # Wait, if det_part is the WHOLE json object `{"detections": ...}`, we can't just concat it easily into another dict/json?
                # The raw 'detections/json' topic returns the full object.
                # If we want to return `{"detections": ..., "imu": ...}`, we need to merge them.
                # Merging raw JSON strings is tricky without parsing.
                # structure: det_part = `{"detections": [...]}`
                # We want: `{"detections": [...], "imu": {...}}`
                # Heuristic: strip the trailing '}' from det_part, append `, "imu": ... }`.
                # This depends on det_part being well-formed and ending with '}'.
                det_str = det_part.strip()
                if det_str.endswith('}'):
                    combined = det_str[:-1] + ', ' + imu_part + '}'
                else:
                    # Fallback to just wrapping detections if malformed, or empty
                    combined = '{"detections": [], ' + imu_part + '}'

                body = (combined + "\n").encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                self.wfile.write(body)
                try:
                    self.wfile.flush()
                except Exception:
                    pass
                return

            # ── Combined endpoint: /api/sensors (odom + lidar + zones) 
            if path == "/api/sensors":
                payload = {}
                if callable(odom_provider):
                    try:
                        payload['odom'] = odom_provider()
                    except Exception:
                        payload['odom'] = {'error': 'failed'}
                if callable(scan_provider):
                    try:
                        payload['lidar'] = scan_provider()
                    except Exception:
                        payload['lidar'] = {'error': 'failed'}
                if callable(lidar_zones_provider):
                    try:
                        payload['lidar_zones'] = lidar_zones_provider()
                    except Exception:
                        payload['lidar_zones'] = {'error': 'failed'}
                body = (json.dumps(payload) + "\n").encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                self.wfile.write(body)
                try:
                    self.wfile.flush()
                except Exception:
                    pass
                return

            if path.startswith("/api/snapshots/"):
                fname = path[len("/api/snapshots/"):]
                # Sanitize: only allow simple filenames (no path traversal)
                if not fname or '/' in fname or '..' in fname or '\\' in fname:
                    self.send_response(HTTPStatus.BAD_REQUEST)
                    self.end_headers()
                    return
                sdir = os.path.expanduser(snapshot_dir) if snapshot_dir else os.path.expanduser("~/Pictures/raspbot")
                fpath = os.path.join(sdir, fname)
                if not os.path.isfile(fpath):
                    self.send_response(HTTPStatus.NOT_FOUND)
                    self.end_headers()
                    return
                try:
                    with open(fpath, "rb") as f:
                        data = f.read()
                    self.send_response(HTTPStatus.OK)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(data)))
                    self.send_header("Content-Disposition", f'inline; filename="{fname}"')
                    self.end_headers()
                    self.wfile.write(data)
                except Exception:
                    self.send_response(HTTPStatus.INTERNAL_SERVER_ERROR)
                    self.end_headers()
                return

            if path.startswith("/stream.mjpg"):
                self.send_response(HTTPStatus.OK)
                self.send_header("Age", "0")
                self.send_header("Cache-Control", "no-cache, private")
                self.send_header("Pragma", "no-cache")
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                self.wfile.flush()

                last_stamp = 0.0
                last_sent_time = 0.0
                try:
                    self.connection.settimeout(10.0)
                except Exception:
                    pass

                # Parse query parameters: w, h, q
                qs = parse_qs(parsed.query)
                do_resize = 'w' in qs or 'h' in qs or 'q' in qs
                if do_resize and _cv2 is not None:
                    lo_w = int(qs.get("w", ["640"])[0])
                    lo_h = int(qs.get("h", ["480"])[0])
                    lo_q = int(qs.get("q", ["30"])[0])
                    lo_w = max(80, min(lo_w, 1280))
                    lo_h = max(60, min(lo_h, 720))
                    lo_q = max(5, min(lo_q, 95))
                else:
                    do_resize = False

                try:
                    while True:
                        # Throttle server-side to avoid pegging CPU/network.
                        now = time.monotonic()
                        sleep_needed = (last_sent_time + min_period) - now
                        if sleep_needed > 0:
                            time.sleep(sleep_needed)

                        frame = frame_buffer.wait_for_newer(last_stamp, timeout=1.0)
                        if frame.jpeg is None:
                            # if logger: logger.debug("Stream wait_for_newer timeout/empty")
                            continue


                        last_stamp = frame.stamp_monotonic
                        last_sent_time = time.monotonic()

                        if do_resize:
                             jpeg = _get_cached_lo(
                                "gimbal", frame.stamp_monotonic,
                                frame.jpeg, lo_w, lo_h, lo_q)
                        else:
                             jpeg = frame.jpeg

                        # Single write per frame: boundary + headers + jpeg + CRLF
                        self.wfile.write(
                            boundary + b"\r\n"
                            b"Content-Type: image/jpeg\r\n"
                            b"Content-Length: " + str(len(jpeg)).encode("ascii") + b"\r\n\r\n"
                            + jpeg + b"\r\n"
                        )
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
                    return
                except Exception as e:
                    if logger is not None:
                        logger.warn(f"stream client error: {e!r}")
                    return
                finally:
                    if logger is not None:
                         logger.info("Stream client disconnected")


            if path.startswith("/stream_front.mjpg"):
                if front_frame_buffer is None:
                    self.send_response(HTTPStatus.SERVICE_UNAVAILABLE)
                    self.send_header("Content-Type", "text/plain; charset=utf-8")
                    self.end_headers()
                    self.wfile.write(b"Front camera not available\n")
                    return

                self.send_response(HTTPStatus.OK)
                self.send_header("Age", "0")
                self.send_header("Cache-Control", "no-cache, private")
                self.send_header("Pragma", "no-cache")
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                self.wfile.flush()

                last_stamp = 0.0
                last_sent_time = 0.0
                try:
                    self.connection.settimeout(10.0)
                except Exception:
                    pass

                try:
                    while True:
                        now = time.monotonic()
                        sleep_needed = (last_sent_time + min_period) - now
                        if sleep_needed > 0:
                            time.sleep(sleep_needed)

                        frame = front_frame_buffer.wait_for_newer(last_stamp, timeout=1.0)
                        if frame.jpeg is None:
                            continue

                        last_stamp = frame.stamp_monotonic
                        last_sent_time = time.monotonic()

                        jpeg = frame.jpeg
                        self.wfile.write(
                            boundary + b"\r\n"
                            b"Content-Type: image/jpeg\r\n"
                            b"Content-Length: " + str(len(jpeg)).encode("ascii") + b"\r\n\r\n"
                            + jpeg + b"\r\n"
                        )
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
                    return
                except Exception as e:
                    if logger is not None:
                        logger.warn(f"front stream client error: {e!r}")
                    return

            if path.startswith("/stream_depth.mjpg"):
                if depth_frame_buffer is None:
                    self.send_response(HTTPStatus.SERVICE_UNAVAILABLE)
                    self.send_header("Content-Type", "text/plain; charset=utf-8")
                    self.end_headers()
                    self.wfile.write(b"Depth stream not available\n")
                    return

                self.send_response(HTTPStatus.OK)
                self.send_header("Age", "0")
                self.send_header("Cache-Control", "no-cache, private")
                self.send_header("Pragma", "no-cache")
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                self.wfile.flush()

                last_stamp = 0.0
                last_sent_time = 0.0
                try:
                    self.connection.settimeout(10.0)
                except Exception:
                    pass

                try:
                    while True:
                        now = time.monotonic()
                        sleep_needed = (last_sent_time + depth_min_period) - now
                        if sleep_needed > 0:
                            time.sleep(sleep_needed)

                        frame = depth_frame_buffer.wait_for_newer(last_stamp, timeout=1.0)
                        if frame.jpeg is None:
                            continue

                        last_stamp = frame.stamp_monotonic
                        last_sent_time = time.monotonic()

                        jpeg = frame.jpeg
                        self.wfile.write(
                            boundary + b"\r\n"
                            b"Content-Type: image/jpeg\r\n"
                            b"Content-Length: " + str(len(jpeg)).encode("ascii") + b"\r\n\r\n"
                            + jpeg + b"\r\n"
                        )
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
                    return
                except Exception as e:
                    if logger is not None:
                        logger.warn(f"depth stream client error: {e!r}")
                    return

            # ── Low-resolution streams (320×240) for embedded clients ────
            # /stream_lo.mjpg   → gimbal camera, resized
            # /stream_front_lo.mjpg → front camera, resized
            if path.startswith("/stream_lo.mjpg") or path.startswith("/stream_front_lo.mjpg"):
                is_front = path.startswith("/stream_front_lo")
                src_buffer = front_frame_buffer if is_front else frame_buffer
                cam_label = "front" if is_front else "gimbal"

                if _cv2 is None:
                    self.send_response(HTTPStatus.SERVICE_UNAVAILABLE)
                    self.send_header("Content-Type", "text/plain; charset=utf-8")
                    self.end_headers()
                    self.wfile.write(b"Low-res stream requires OpenCV (cv2)\n")
                    return

                if src_buffer is None:
                    self.send_response(HTTPStatus.SERVICE_UNAVAILABLE)
                    self.send_header("Content-Type", "text/plain; charset=utf-8")
                    self.end_headers()
                    self.wfile.write(f"{cam_label} camera not available\n".encode())
                    return

                # Parse optional query parameters: w, h, q
                qs = parse_qs(parsed.query)
                lo_w = int(qs.get("w", ["320"])[0])
                lo_h = int(qs.get("h", ["240"])[0])
                lo_q = int(qs.get("q", ["50"])[0])
                lo_w = max(80, min(lo_w, 640))
                lo_h = max(60, min(lo_h, 480))
                lo_q = max(10, min(lo_q, 95))

                self.send_response(HTTPStatus.OK)
                self.send_header("Age", "0")
                self.send_header("Cache-Control", "no-cache, private")
                self.send_header("Pragma", "no-cache")
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                self.wfile.flush()

                last_stamp = 0.0
                last_sent_time = 0.0
                try:
                    self.connection.settimeout(10.0)
                except Exception:
                    pass

                try:
                    while True:
                        now = time.monotonic()
                        sleep_needed = (last_sent_time + min_period) - now
                        if sleep_needed > 0:
                            time.sleep(sleep_needed)

                        frame = src_buffer.wait_for_newer(last_stamp, timeout=1.0)
                        if frame.jpeg is None:
                            continue

                        last_stamp = frame.stamp_monotonic
                        last_sent_time = time.monotonic()

                        resized = _get_cached_lo(
                            cam_label, frame.stamp_monotonic,
                            frame.jpeg, lo_w, lo_h, lo_q)

                        self.wfile.write(
                            boundary + b"\r\n"
                            b"Content-Type: image/jpeg\r\n"
                            b"Content-Length: " + str(len(resized)).encode("ascii") + b"\r\n\r\n"
                            + resized + b"\r\n"
                        )
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
                    return
                except Exception as e:
                    if logger is not None:
                        logger.warn(f"{cam_label} lo-res stream error: {e!r}")
                    return

            if path.startswith("/stream_audio"):
                if audio_buffer is None:
                    self.send_response(HTTPStatus.SERVICE_UNAVAILABLE)
                    self.send_header("Content-Type", "text/plain; charset=utf-8")
                    self.end_headers()
                    self.wfile.write(b"Audio not available\n")
                    return

                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "application/octet-stream")
                self.send_header("Cache-Control", "no-cache, private")
                self.send_header("X-Audio-Sample-Rate", "8000")
                self.send_header("X-Audio-Format", "u8")
                self.send_header("X-Audio-Channels", "1")
                self.end_headers()
                try:
                    self.wfile.flush()
                except Exception:
                    pass

                last_seq = -1
                try:
                    self.connection.settimeout(3.0)
                except Exception:
                    pass
                try:
                    while True:
                        seq, data = audio_buffer.get_newer(last_seq, timeout=2.0)
                        if not data:
                            continue
                        last_seq = seq
                        self.wfile.write(data)
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, TimeoutError):
                    return
                except Exception as e:
                    if logger is not None:
                        logger.warn(f"audio stream client error: {e!r}")
                    return

            # ── Face DB API (GET) ────────────────────────────────
            if path == '/api/faces':
                faces = []
                if callable(face_list):
                    try:
                        faces = face_list()
                    except Exception:
                        pass
                body = json.dumps({"faces": faces}).encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if path == '/api/faces/thumbnail':
                qs = parse_qs(parsed.query)
                try:
                    fid = int(qs.get('id', ['0'])[0])
                except Exception:
                    fid = 0
                thumb = None
                if callable(face_thumbnail) and fid > 0:
                    try:
                        thumb = face_thumbnail(fid)
                    except Exception:
                        pass
                if thumb:
                    self.send_response(HTTPStatus.OK)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(thumb)))
                    self.send_header("Cache-Control", "max-age=60")
                    self.end_headers()
                    self.wfile.write(thumb)
                else:
                    self.send_response(HTTPStatus.NOT_FOUND)
                    self.end_headers()
                return

            self.send_response(HTTPStatus.NOT_FOUND)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.end_headers()
            self.wfile.write(b"Not found\n")

        def do_POST(self):
            parsed = urlparse(self.path)
            path = parsed.path

            if path == '/api/gimbal/center':
                if not callable(gimbal_center):
                    self.send_response(HTTPStatus.NOT_IMPLEMENTED)
                    self.end_headers()
                    return
                try:
                    gimbal_center()
                except Exception:
                    self.send_response(HTTPStatus.INTERNAL_SERVER_ERROR)
                    self.end_headers()
                    return
                self.send_response(HTTPStatus.NO_CONTENT)
                self.end_headers()
                return

            if path == '/api/gimbal':
                if not callable(gimbal_setter):
                    self.send_response(HTTPStatus.NOT_IMPLEMENTED)
                    self.end_headers()
                    return

                try:
                    length = int(self.headers.get('Content-Length', '0'))
                except Exception:
                    length = 0
                if length <= 0 or length > 8192:
                    self.send_response(HTTPStatus.BAD_REQUEST)
                    self.end_headers()
                    return

                raw = self.rfile.read(length)
                try:
                    payload = json.loads(raw.decode('utf-8'))
                    pan = float(payload.get('pan_deg'))
                    tilt = float(payload.get('tilt_deg'))
                except Exception:
                    self.send_response(HTTPStatus.BAD_REQUEST)
                    self.end_headers()
                    return

                if pan != pan or tilt != tilt:
                    self.send_response(HTTPStatus.BAD_REQUEST)
                    self.end_headers()
                    return

                try:
                    gimbal_setter(pan, tilt)
                except Exception:
                    self.send_response(HTTPStatus.INTERNAL_SERVER_ERROR)
                    self.end_headers()
                    return

                self.send_response(HTTPStatus.NO_CONTENT)
                self.end_headers()
                return

            if path == '/api/cmd_vel/stop':
                if not callable(cmd_vel_stopper):
                    self.send_response(HTTPStatus.NOT_IMPLEMENTED)
                    self.end_headers()
                    return
                try:
                    cmd_vel_stopper()
                except Exception:
                    self.send_response(HTTPStatus.INTERNAL_SERVER_ERROR)
                    self.end_headers()
                    return
                self.send_response(HTTPStatus.NO_CONTENT)
                self.end_headers()
                return

            if path == '/api/cmd_vel':
                if not callable(cmd_vel_setter):
                    self.send_response(HTTPStatus.NOT_IMPLEMENTED)
                    self.end_headers()
                    return

                try:
                    length = int(self.headers.get('Content-Length', '0'))
                except Exception:
                    length = 0
                if length <= 0 or length > 8192:
                    self.send_response(HTTPStatus.BAD_REQUEST)
                    self.end_headers()
                    return

                raw = self.rfile.read(length)
                try:
                    payload = json.loads(raw.decode('utf-8'))
                    lin = float(payload.get('linear_x'))
                    lat = float(payload.get('linear_y', 0.0))
                    ang = float(payload.get('angular_z'))
                except Exception:
                    self.send_response(HTTPStatus.BAD_REQUEST)
                    self.end_headers()
                    return

                if lin != lin or ang != ang or lat != lat:
                    self.send_response(HTTPStatus.BAD_REQUEST)
                    self.end_headers()
                    return

                try:
                    cmd_vel_setter(lin, ang, lat)
                except Exception:
                    self.send_response(HTTPStatus.INTERNAL_SERVER_ERROR)
                    self.end_headers()
                    return

                self.send_response(HTTPStatus.NO_CONTENT)
                self.end_headers()
                return

            if path == '/api/tracking':
                if not callable(tracking_setter):
                    self.send_response(HTTPStatus.NOT_IMPLEMENTED)
                    self.end_headers()
                    return

                try:
                    length = int(self.headers.get('Content-Length', '0'))
                except Exception:
                    length = 0
                if length <= 0 or length > 8192:
                    self.send_response(HTTPStatus.BAD_REQUEST)
                    self.end_headers()
                    return

                raw = self.rfile.read(length)
                try:
                    payload = json.loads(raw.decode('utf-8'))
                    enabled = bool(payload.get('enabled'))
                except Exception:
                    self.send_response(HTTPStatus.BAD_REQUEST)
                    self.end_headers()
                    return

                try:
                    tracking_setter(enabled)
                except Exception:
                    self.send_response(HTTPStatus.INTERNAL_SERVER_ERROR)
                    self.end_headers()
                    return

                self.send_response(HTTPStatus.NO_CONTENT)
                self.end_headers()
                return

            if path == '/api/tracking/select':
                if not callable(person_select_setter):
                    self.send_response(HTTPStatus.NOT_IMPLEMENTED)
                    self.end_headers()
                    return

                try:
                    length = int(self.headers.get('Content-Length', '0'))
                except Exception:
                    length = 0
                if length <= 0 or length > 8192:
                    self.send_response(HTTPStatus.BAD_REQUEST)
                    self.end_headers()
                    return

                raw = self.rfile.read(length)
                try:
                    payload = json.loads(raw.decode('utf-8'))
                    person_id = int(payload.get('person_id', -1))
                except Exception:
                    self.send_response(HTTPStatus.BAD_REQUEST)
                    self.end_headers()
                    return

                try:
                    person_select_setter(person_id)
                except Exception:
                    self.send_response(HTTPStatus.INTERNAL_SERVER_ERROR)
                    self.end_headers()
                    return

                self.send_response(HTTPStatus.NO_CONTENT)
                self.end_headers()
                return

            if path == '/api/follow':
                if not callable(follow_setter):
                    self.send_response(HTTPStatus.NOT_IMPLEMENTED)
                    self.end_headers()
                    return

                try:
                    length = int(self.headers.get('Content-Length', '0'))
                except Exception:
                    length = 0
                if length <= 0 or length > 8192:
                    self.send_response(HTTPStatus.BAD_REQUEST)
                    self.end_headers()
                    return

                raw = self.rfile.read(length)
                try:
                    payload = json.loads(raw.decode('utf-8'))
                    enabled = bool(payload.get('enabled'))
                except Exception:
                    self.send_response(HTTPStatus.BAD_REQUEST)
                    self.end_headers()
                    return

                try:
                    follow_setter(enabled)
                except Exception:
                    self.send_response(HTTPStatus.INTERNAL_SERVER_ERROR)
                    self.end_headers()
                    return

                self.send_response(HTTPStatus.NO_CONTENT)
                self.end_headers()
                return

            if path == '/api/follow/config':
                if not callable(follow_config_setter):
                    self.send_response(HTTPStatus.NOT_IMPLEMENTED)
                    self.end_headers()
                    return

                try:
                    length = int(self.headers.get('Content-Length', '0'))
                except Exception:
                    length = 0
                if length <= 0 or length > 8192:
                    self.send_response(HTTPStatus.BAD_REQUEST)
                    self.end_headers()
                    return

                raw = self.rfile.read(length)
                try:
                    payload = json.loads(raw.decode('utf-8'))
                    target_area = payload.get('target_bbox_area')
                    max_linear = payload.get('max_linear')
                    strafe_gain = payload.get('strafe_gain')
                    gyro_damping = payload.get('gyro_damping')
                    if target_area is not None:
                        target_area = float(target_area)
                    if max_linear is not None:
                        max_linear = float(max_linear)
                    if strafe_gain is not None:
                        strafe_gain = float(strafe_gain)
                    if gyro_damping is not None:
                        gyro_damping = float(gyro_damping)
                except Exception:
                    self.send_response(HTTPStatus.BAD_REQUEST)
                    self.end_headers()
                    return

                try:
                    follow_config_setter(target_bbox_area=target_area, max_linear=max_linear,
                                         strafe_gain=strafe_gain, gyro_damping=gyro_damping)
                except Exception:
                    self.send_response(HTTPStatus.INTERNAL_SERVER_ERROR)
                    self.end_headers()
                    return

                self.send_response(HTTPStatus.NO_CONTENT)
                self.end_headers()
                return

            if path in {'/api/tracking/config', '/api/tracking_config'}:
                if not callable(tracking_config_setter):
                    self.send_response(HTTPStatus.NOT_IMPLEMENTED)
                    self.end_headers()
                    return

                try:
                    length = int(self.headers.get('Content-Length', '0'))
                except Exception:
                    length = 0
                if length <= 0 or length > 8192:
                    self.send_response(HTTPStatus.BAD_REQUEST)
                    self.end_headers()
                    return

                raw = self.rfile.read(length)
                try:
                    payload = json.loads(raw.decode('utf-8'))

                    # Preferred UI format: invert toggles
                    if 'invert_pan' in payload or 'invert_tilt' in payload:
                        inv_pan = bool(payload.get('invert_pan', False))
                        inv_tilt = bool(payload.get('invert_tilt', False))
                        pan_sign = -1 if inv_pan else 1
                        tilt_sign = -1 if inv_tilt else 1
                    else:
                        pan_sign = payload.get('pan_sign', None)
                        tilt_sign = payload.get('tilt_sign', None)
                except Exception:
                    self.send_response(HTTPStatus.BAD_REQUEST)
                    self.end_headers()
                    return

                try:
                    tracking_config_setter(pan_sign, tilt_sign)
                except Exception:
                    self.send_response(HTTPStatus.INTERNAL_SERVER_ERROR)
                    self.end_headers()
                    return

                self.send_response(HTTPStatus.NO_CONTENT)
                self.end_headers()
                return

            if path == '/api/snapshot':
                if not callable(snapshot_taker):
                    self.send_response(HTTPStatus.NOT_IMPLEMENTED)
                    self.end_headers()
                    return
                try:
                    result = snapshot_taker()
                    body = json.dumps(result).encode('utf-8')
                    self.send_response(HTTPStatus.OK)
                    self.send_header('Content-Type', 'application/json; charset=utf-8')
                    self.send_header('Content-Length', str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                except RuntimeError as e:
                    body = json.dumps({'error': str(e)}).encode('utf-8')
                    self.send_response(HTTPStatus.SERVICE_UNAVAILABLE)
                    self.send_header('Content-Type', 'application/json; charset=utf-8')
                    self.send_header('Content-Length', str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                except Exception:
                    self.send_response(HTTPStatus.INTERNAL_SERVER_ERROR)
                    self.end_headers()
                return

            if path == '/api/lightbar':
                if not callable(lightbar_setter):
                    self.send_response(HTTPStatus.NOT_IMPLEMENTED)
                    self.end_headers()
                    return

                try:
                    length = int(self.headers.get('Content-Length', '0'))
                except Exception:
                    length = 0
                if length <= 0 or length > 8192:
                    self.send_response(HTTPStatus.BAD_REQUEST)
                    self.end_headers()
                    return

                raw = self.rfile.read(length)
                try:
                    payload = json.loads(raw.decode('utf-8'))
                except Exception:
                    self.send_response(HTTPStatus.BAD_REQUEST)
                    self.end_headers()
                    return

                try:
                    lightbar_setter(payload)
                except Exception:
                    self.send_response(HTTPStatus.INTERNAL_SERVER_ERROR)
                    self.end_headers()
                    return

                self.send_response(HTTPStatus.NO_CONTENT)
                self.end_headers()
                return

            if path == '/api/imu/rotate':
                if not callable(rotate_starter):
                    self.send_response(HTTPStatus.NOT_IMPLEMENTED)
                    self.end_headers()
                    return

                try:
                    length = int(self.headers.get('Content-Length', '0'))
                except Exception:
                    length = 0
                if length <= 0 or length > 8192:
                    self.send_response(HTTPStatus.BAD_REQUEST)
                    self.end_headers()
                    return

                raw = self.rfile.read(length)
                try:
                    payload = json.loads(raw.decode('utf-8'))
                    target_deg = float(payload.get('target_deg', 0))
                    speed = float(payload.get('speed', 0.5))
                except Exception:
                    self.send_response(HTTPStatus.BAD_REQUEST)
                    self.end_headers()
                    return

                try:
                    rotate_starter(target_deg, speed)
                except Exception:
                    self.send_response(HTTPStatus.INTERNAL_SERVER_ERROR)
                    self.end_headers()
                    return

                self.send_response(HTTPStatus.NO_CONTENT)
                self.end_headers()
                return

            if path == '/api/imu/rotate/stop':
                if not callable(rotate_stopper):
                    self.send_response(HTTPStatus.NOT_IMPLEMENTED)
                    self.end_headers()
                    return
                try:
                    rotate_stopper()
                except Exception:
                    self.send_response(HTTPStatus.INTERNAL_SERVER_ERROR)
                    self.end_headers()
                    return
                self.send_response(HTTPStatus.NO_CONTENT)
                self.end_headers()
                return

            if path == '/api/imu/calibrate':
                if callable(imu_calibrate):
                    try:
                        imu_calibrate()
                    except Exception:
                        pass
                self.send_response(HTTPStatus.NO_CONTENT)
                self.end_headers()
                return

            if path == '/api/bno/autocal/start':
                result = {'ok': False, 'error': 'not available'}
                if callable(autocal_starter):
                    try:
                        result = autocal_starter()
                    except Exception as e:
                        result = {'ok': False, 'error': str(e)}
                body = (json.dumps(result) + '\n').encode('utf-8')
                self.send_response(HTTPStatus.OK)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if path == '/api/bno/autocal/stop':
                if callable(autocal_stopper):
                    try:
                        autocal_stopper()
                    except Exception:
                        pass
                self.send_response(HTTPStatus.NO_CONTENT)
                self.end_headers()
                return

            if path == '/api/audio/enable':
                if callable(audio_enabler):
                    try:
                        audio_enabler()
                    except Exception:
                        pass
                self.send_response(HTTPStatus.NO_CONTENT)
                self.end_headers()
                return

            if path == '/api/audio/disable':
                if callable(audio_disabler):
                    try:
                        audio_disabler()
                    except Exception:
                        pass
                self.send_response(HTTPStatus.NO_CONTENT)
                self.end_headers()
                return

            if path == '/api/depth/enable':
                if callable(depth_enabler):
                    try:
                        depth_enabler()
                    except Exception:
                        pass
                self.send_response(HTTPStatus.NO_CONTENT)
                self.end_headers()
                return

            if path == '/api/depth/disable':
                if callable(depth_disabler):
                    try:
                        depth_disabler()
                    except Exception:
                        pass
                self.send_response(HTTPStatus.NO_CONTENT)
                self.end_headers()
                return

            if path == '/api/collision_failsafe/enable':
                if callable(collision_failsafe_enabler):
                    try:
                        collision_failsafe_enabler()
                    except Exception:
                        pass
                self.send_response(HTTPStatus.NO_CONTENT)
                self.end_headers()
                return

            if path == '/api/collision_failsafe/disable':
                if callable(collision_failsafe_disabler):
                    try:
                        collision_failsafe_disabler()
                    except Exception:
                        pass
                self.send_response(HTTPStatus.NO_CONTENT)
                self.end_headers()
                return

            if path == '/api/cliff_failsafe/enable':
                if callable(cliff_failsafe_enabler):
                    try:
                        cliff_failsafe_enabler()
                    except Exception:
                        pass
                self.send_response(HTTPStatus.NO_CONTENT)
                self.end_headers()
                return

            if path == '/api/cliff_failsafe/disable':
                if callable(cliff_failsafe_disabler):
                    try:
                        cliff_failsafe_disabler()
                    except Exception:
                        pass
                self.send_response(HTTPStatus.NO_CONTENT)
                self.end_headers()
                return

            # ── Odometry service endpoints ───────────────────────────
            odom_service_map = {
                '/api/odom/set_origin': odom_set_origin,
                '/api/odom/start_recording': odom_start_recording,
                '/api/odom/stop_recording': odom_stop_recording,
                '/api/odom/return_to_origin': odom_return_to_origin,
                '/api/odom/cancel_return': odom_cancel_return,
                '/api/odom/cancel_navigate': odom_cancel_navigate,
                '/api/slam/reset': slam_reset,
                '/api/calibrate_front': calibrate_front,
                '/api/calibrate_front_static': calibrate_front_static,
            }

            # Navigate multi-waypoint path (needs JSON body)
            if path == '/api/odom/navigate_path':
                if callable(odom_navigate_path):
                    try:
                        content_length = int(self.headers.get('Content-Length', 0))
                        body = self.rfile.read(content_length) if content_length else b'{}'
                        import json as _json
                        payload = _json.loads(body)
                        wps = payload.get('waypoints', [])
                        result = odom_navigate_path(wps)
                    except Exception as exc:
                        result = {'ok': False, 'message': str(exc)}
                else:
                    result = {'ok': False, 'message': 'navigate_path not available'}
                import json as _json
                payload_bytes = _json.dumps(result).encode()
                self.send_response(HTTPStatus.OK)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(payload_bytes)))
                self.end_headers()
                self.wfile.write(payload_bytes)
                return

            # Navigate-to-waypoint (needs JSON body)
            if path == '/api/odom/navigate_to':
                if callable(odom_navigate_to):
                    try:
                        content_length = int(self.headers.get('Content-Length', 0))
                        body = self.rfile.read(content_length) if content_length else b'{}'
                        import json as _json
                        payload = _json.loads(body)
                        result = odom_navigate_to(payload.get('x', 0.0), payload.get('y', 0.0))
                    except Exception as exc:
                        result = {'ok': False, 'message': str(exc)}
                else:
                    result = {'ok': False, 'message': 'navigate_to not available'}
                import json as _json
                payload_bytes = _json.dumps(result).encode()
                self.send_response(HTTPStatus.OK)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(payload_bytes)))
                self.end_headers()
                self.wfile.write(payload_bytes)
                return

            if path in odom_service_map:
                handler_fn = odom_service_map[path]
                if not callable(handler_fn):
                    self.send_response(HTTPStatus.NOT_IMPLEMENTED)
                    self.end_headers()
                    return
                try:
                    result = handler_fn()
                    body = (json.dumps(result) + '\n').encode('utf-8')
                    self.send_response(HTTPStatus.OK)
                    self.send_header('Content-Type', 'application/json; charset=utf-8')
                    self.send_header('Content-Length', str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                except Exception:
                    self.send_response(HTTPStatus.INTERNAL_SERVER_ERROR)
                    self.end_headers()
                return

            # ── Face DB API (POST) ───────────────────────────────
            if path == '/api/faces/rename':
                try:
                    length = int(self.headers.get('Content-Length', '0'))
                    raw = self.rfile.read(length) if length else b'{}'
                    payload = json.loads(raw)
                    fid = int(payload['face_id'])
                    name = str(payload['name'])
                except Exception:
                    self.send_response(HTTPStatus.BAD_REQUEST)
                    self.end_headers()
                    return
                ok = False
                if callable(face_rename):
                    try:
                        ok = face_rename(fid, name)
                    except Exception:
                        pass
                body = json.dumps({"ok": ok}).encode()
                self.send_response(HTTPStatus.OK)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if path == '/api/faces/delete':
                try:
                    length = int(self.headers.get('Content-Length', '0'))
                    raw = self.rfile.read(length) if length else b'{}'
                    payload = json.loads(raw)
                    fid = int(payload['face_id'])
                except Exception:
                    self.send_response(HTTPStatus.BAD_REQUEST)
                    self.end_headers()
                    return
                ok = False
                if callable(face_delete):
                    try:
                        ok = face_delete(fid)
                    except Exception:
                        pass
                body = json.dumps({"ok": ok}).encode()
                self.send_response(HTTPStatus.OK)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if path == '/api/faces/merge':
                try:
                    length = int(self.headers.get('Content-Length', '0'))
                    raw = self.rfile.read(length) if length else b'{}'
                    payload = json.loads(raw)
                    keep_id = int(payload['keep_id'])
                    merge_id = int(payload['merge_id'])
                except Exception:
                    self.send_response(HTTPStatus.BAD_REQUEST)
                    self.end_headers()
                    return
                ok = False
                if callable(face_merge):
                    try:
                        ok = face_merge(keep_id, merge_id)
                    except Exception:
                        pass
                body = json.dumps({"ok": ok}).encode()
                self.send_response(HTTPStatus.OK)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if path == '/api/faces/clear':
                n = 0
                if callable(face_clear):
                    try:
                        n = face_clear()
                    except Exception:
                        pass
                body = json.dumps({"ok": True, "deleted": n}).encode()
                self.send_response(HTTPStatus.OK)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            self.send_response(HTTPStatus.NOT_FOUND)
            self.send_header('Content-Type', 'text/plain; charset=utf-8')
            self.end_headers()
            self.wfile.write(b'Not found\n')

    return Handler


def main() -> None:
    rclpy.init()
    frame_buffer = FrameBuffer()
    front_frame_buffer = FrameBuffer()
    audio_buffer = AudioBuffer()
    depth_frame_buffer = FrameBuffer()
    node = WebVideoNode(frame_buffer, front_frame_buffer, audio_buffer, depth_frame_buffer)

    bind = str(node.get_parameter("bind").value)
    port = int(node.get_parameter("port").value)
    fps_limit = float(node.get_parameter("fps_limit").value)
    depth_fps_limit = float(node.get_parameter("depth_fps_limit").value)
    http_backlog = int(node.get_parameter("http_backlog").value)

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    handler_cls = make_handler(
        frame_buffer=frame_buffer,
        front_frame_buffer=front_frame_buffer,
        depth_frame_buffer=depth_frame_buffer,
        fps_limit=fps_limit,
        depth_fps_limit=depth_fps_limit,
        logger=node.get_logger(),
        status_provider=node.status_dict,
        detections_provider=node.get_latest_detections,
        detections_json_provider=node.get_latest_detections_json,
        gimbal_setter=node.set_gimbal,
        gimbal_center=node.center_gimbal,
        cmd_vel_setter=node.set_cmd_vel,
        cmd_vel_stopper=node.stop_cmd_vel,
        tracking_setter=node.set_tracking_enabled,
        tracking_config_setter=node.set_tracking_config,
        person_select_setter=node.set_person_select,
        follow_setter=node.set_follow_enabled,
        follow_config_setter=node.set_follow_config,
        snapshot_taker=node.take_snapshot,
        snapshot_dir=str(node.get_parameter("snapshot_dir").value),
        lightbar_setter=node.set_lightbar_command,
        imu_provider=node.get_imu_dict,
        imu_calibrate=node.imu_calibrate,
        autocal_starter=node.start_autocal,
        autocal_stopper=node.stop_autocal,
        rotate_starter=node.start_rotate,
        rotate_stopper=node.stop_rotate,
        audio_buffer=audio_buffer,
        audio_enabler=node.audio_enable,
        audio_disabler=node.audio_disable,
        depth_enabler=node.depth_enable,
        depth_disabler=node.depth_disable,
        collision_failsafe_enabler=node.collision_failsafe_enable,
        collision_failsafe_disabler=node.collision_failsafe_disable,
        cliff_failsafe_enabler=node.cliff_failsafe_enable,
        cliff_failsafe_disabler=node.cliff_failsafe_disable,
        odom_provider=node.get_odom_dict,
        odom_set_origin=node.odom_set_origin,
        odom_start_recording=node.odom_start_recording,
        odom_stop_recording=node.odom_stop_recording,
        odom_return_to_origin=node.odom_return_to_origin,
        odom_cancel_return=node.odom_cancel_return,
        odom_navigate_to=node.odom_navigate_to,
        odom_navigate_path=node.odom_navigate_path,
        odom_cancel_navigate=node.odom_cancel_navigate,
        scan_provider=node.get_scan_dict,
        map_provider=node.get_map_dict,
        lidar_zones_provider=node.get_lidar_zones_dict,
        slam_reset=node.slam_reset,
        calibrate_front=node.calibrate_front,
        calibrate_front_static=node.calibrate_front_static,
        face_list=node.face_list,
        face_rename=node.face_rename,
        face_delete=node.face_delete,
        face_merge=node.face_merge,
        face_clear=node.face_clear,
        face_thumbnail=node.face_thumbnail,
    )
    class RaspbotHTTPServer(ThreadingHTTPServer):
        daemon_threads = True

        def handle_error(self, request, client_address):
            # Suppress noisy traceback for expected stream disconnections.
            import sys
            exc = sys.exc_info()[1]
            if isinstance(exc, (BrokenPipeError, ConnectionResetError,
                                ConnectionAbortedError, TimeoutError)):
                return  # expected — client closed the stream
            super().handle_error(request, client_address)

    RaspbotHTTPServer.request_queue_size = max(16, min(256, int(http_backlog)))
    httpd = RaspbotHTTPServer((bind, port), handler_cls)

    node.get_logger().info(
        f"Web video server on http://{bind}:{port}/ (MJPEG: /stream.mjpg, front: /stream_front.mjpg, depth: /stream_depth.mjpg, status: /status)"
    )

    # Use serve_forever() in a daemon thread for efficient accept() polling,
    # then watch rclpy.ok() in the main thread.
    serve_thread = threading.Thread(target=httpd.serve_forever, kwargs={"poll_interval": 0.5}, daemon=True)
    serve_thread.start()

    try:
        while rclpy.ok():
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            httpd.shutdown()   # signals serve_forever() to exit
        except Exception:
            pass
        try:
            httpd.server_close()
        except Exception:
            pass
        try:
            executor.shutdown()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()
