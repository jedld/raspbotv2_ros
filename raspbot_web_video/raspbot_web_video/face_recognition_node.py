"""
ROS 2 node for real-time face recognition using SFace (ONNX) embeddings.

Subscribes to the camera stream and hailo detection JSON, crops detected
faces, computes 128-d embeddings, and matches / auto-enrolls them against
the persistent SQLite face database.  Publishes an enriched detection JSON
with ``face_id`` and ``face_name`` fields added to face detections.
"""

from __future__ import annotations

import json
import os
import threading
import time
from typing import Optional

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from raspbot_web_video.face_db import FaceDB

# ---------------------------------------------------------------------------
#  Constants
# ---------------------------------------------------------------------------
_DEFAULT_MODEL_PATH = os.path.expanduser(
    "~/.local/share/raspbot/models/face_recognition_sface_2021dec.onnx"
)
_SFACE_INPUT_SIZE = 112
_EMBEDDING_DIM = 128


class FaceRecognitionNode(Node):
    """Lightweight face-recognition bridge that enriches hailo detections."""

    def __init__(self):
        super().__init__("face_recognition")

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter("model_path", _DEFAULT_MODEL_PATH)
        self.declare_parameter("match_threshold", 0.363)
        self.declare_parameter("auto_enroll", True)
        self.declare_parameter("min_face_px", 30)      # min face bbox side in pixels
        self.declare_parameter("process_interval", 0.5) # seconds between recognition passes
        self.declare_parameter("max_embeddings_per_face", 20)
        self.declare_parameter("input_topic", "image_raw/compressed")
        self.declare_parameter("detections_topic", "detections/json")
        self.declare_parameter("output_topic", "face_recognition/detections")
        self.declare_parameter("min_score", 0.55)       # min detection confidence

        model_path = str(self.get_parameter("model_path").value)
        self._match_threshold = float(self.get_parameter("match_threshold").value)
        self._auto_enroll = bool(self.get_parameter("auto_enroll").value)
        self._min_face_px = int(self.get_parameter("min_face_px").value)
        self._process_interval = float(self.get_parameter("process_interval").value)
        self._max_emb_per_face = int(self.get_parameter("max_embeddings_per_face").value)
        self._min_score = float(self.get_parameter("min_score").value)

        input_topic = str(self.get_parameter("input_topic").value)
        det_topic = str(self.get_parameter("detections_topic").value)
        out_topic = str(self.get_parameter("output_topic").value)

        # ── ONNX Runtime session ────────────────────────────────────
        try:
            import onnxruntime as ort

            self._session = ort.InferenceSession(
                model_path, providers=["CPUExecutionProvider"]
            )
            self._input_name = self._session.get_inputs()[0].name
            self.get_logger().info(
                f"SFace model loaded: {model_path} ({_EMBEDDING_DIM}-d, "
                f"input {_SFACE_INPUT_SIZE}x{_SFACE_INPUT_SIZE})"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to load SFace model: {e}")
            self._session = None

        # ── Face database ───────────────────────────────────────────
        self._db = FaceDB()
        self.get_logger().info(
            f"Face DB: {self._db.db_path} ({self._db.num_faces} faces)"
        )

        # ── State ───────────────────────────────────────────────────
        self._lock = threading.Lock()
        self._latest_jpeg: Optional[bytes] = None   # raw JPEG bytes (not decoded)
        self._latest_frame_stamp: Optional[tuple] = None  # (sec, nanosec)
        self._last_process_time = 0.0
        self._auto_name_counter = self._db.num_faces  # for "Person N"

        # Track-id → face_id mapping cache (ephemeral, for display continuity)
        self._track_face_map: dict[int, int] = {}

        # ── ROS I/O ─────────────────────────────────────────────────
        self._img_sub = self.create_subscription(
            CompressedImage, input_topic, self._on_image, qos_profile_sensor_data
        )
        self._det_sub = self.create_subscription(
            String, det_topic, self._on_detections, 10
        )
        self._det_pub = self.create_publisher(String, out_topic, 10)

        self.get_logger().info(
            f"Face recognition active: {input_topic} + {det_topic} → {out_topic} "
            f"(threshold={self._match_threshold}, auto_enroll={self._auto_enroll}, "
            f"interval={self._process_interval}s)"
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _on_image(self, msg: CompressedImage) -> None:
        """Cache the latest camera frame as raw JPEG (decode only when needed)."""
        with self._lock:
            self._latest_jpeg = bytes(msg.data)
            self._latest_frame_stamp = (
                msg.header.stamp.sec,
                msg.header.stamp.nanosec,
            )

    def _on_detections(self, msg: String) -> None:
        """Receive hailo detections, enrich face entries, republish."""
        if not msg.data or self._session is None:
            # Pass through
            out = String()
            out.data = msg.data
            self._det_pub.publish(out)
            return

        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        detections = data.get("detections", [])
        img_w = data.get("image_width", 0)
        img_h = data.get("image_height", 0)
        det_stamp = data.get("stamp", {})

        # Grab frame snapshot
        with self._lock:
            jpeg_data = self._latest_jpeg
            frame_stamp = self._latest_frame_stamp

        now = time.monotonic()
        should_process = (now - self._last_process_time) >= self._process_interval

        if should_process and jpeg_data is not None:
            # Decode JPEG only now — avoids decoding 30fps when we process 2fps
            buf = np.frombuffer(jpeg_data, dtype=np.uint8)
            frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            if frame is not None:
                self._last_process_time = now
                self._process_faces(detections, frame, img_w, img_h)
            else:
                self._annotate_from_cache(detections)
        else:
            # Still annotate from track-id cache (stale but useful)
            self._annotate_from_cache(detections)

        data["detections"] = detections

        out = String()
        out.data = json.dumps(data, separators=(",", ":"))
        self._det_pub.publish(out)

    # ------------------------------------------------------------------
    # Core face processing
    # ------------------------------------------------------------------
    def _process_faces(
        self, detections: list, frame: np.ndarray, img_w: int, img_h: int
    ) -> None:
        h, w = frame.shape[:2]
        for det in detections:
            if det.get("class_id") != 1:
                continue  # not a face

            score = det.get("score", 0.0)
            if score < self._min_score:
                continue

            # Pixel bbox from detection JSON
            x = int(det.get("x", 0))
            y = int(det.get("y", 0))
            bw = int(det.get("w", 0))
            bh = int(det.get("h", 0))

            if bw < self._min_face_px or bh < self._min_face_px:
                continue

            # Clamp to frame
            x1 = max(0, x)
            y1 = max(0, y)
            x2 = min(w, x + bw)
            y2 = min(h, y + bh)
            if x2 <= x1 or y2 <= y1:
                continue

            crop = frame[y1:y2, x1:x2]
            embedding = self._compute_embedding(crop)
            if embedding is None:
                continue

            track_id = det.get("track_id", -1)

            # Match against DB
            result = self._db.match(embedding, threshold=self._match_threshold)
            if result is not None:
                face_id, face_name, sim = result
                det["face_id"] = face_id
                det["face_name"] = face_name
                det["face_sim"] = round(sim, 3)
                if track_id >= 0:
                    self._track_face_map[track_id] = face_id
                # Optionally add more embeddings to improve centroid
                self._maybe_add_embedding(face_id, embedding, crop)
            elif self._auto_enroll:
                # New unknown face – auto-enroll
                self._auto_name_counter += 1
                name = f"Person {self._auto_name_counter}"
                face_id = self._db.add_face(name)
                crop_jpeg = self._encode_crop(crop)
                self._db.add_embedding(face_id, embedding, crop_jpeg)
                det["face_id"] = face_id
                det["face_name"] = name
                det["face_sim"] = 1.0
                if track_id >= 0:
                    self._track_face_map[track_id] = face_id
                self.get_logger().info(
                    f"Auto-enrolled face_id={face_id} as '{name}'"
                )

    def _annotate_from_cache(self, detections: list) -> None:
        """Add face_id/face_name from the track-id cache (no new embedding)."""
        for det in detections:
            if det.get("class_id") != 1:
                continue
            tid = det.get("track_id", -1)
            if tid >= 0 and tid in self._track_face_map:
                fid = self._track_face_map[tid]
                face = self._db.get_face(fid)
                if face is not None:
                    det["face_id"] = fid
                    det["face_name"] = face["name"]

    # ------------------------------------------------------------------
    # Embedding computation
    # ------------------------------------------------------------------
    def _compute_embedding(self, crop_bgr: np.ndarray) -> Optional[np.ndarray]:
        """Resize crop to 112×112 BGR, normalise, run SFace, return L2-normed 128-d."""
        try:
            resized = cv2.resize(crop_bgr, (_SFACE_INPUT_SIZE, _SFACE_INPUT_SIZE))
            # BGR→RGB, HWC→CHW, float32, /255
            rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
            blob = rgb.astype(np.float32).transpose(2, 0, 1)[np.newaxis] / 255.0
            result = self._session.run(None, {self._input_name: blob})
            emb = result[0].flatten().astype(np.float32)
            norm = np.linalg.norm(emb)
            if norm > 0:
                emb /= norm
            return emb
        except Exception as e:
            self.get_logger().warn(f"Embedding failed: {e}")
            return None

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _maybe_add_embedding(
        self, face_id: int, embedding: np.ndarray, crop_bgr: np.ndarray
    ) -> None:
        """Add an embedding if we haven't reached the cap for this face."""
        info = self._db.get_face(face_id)
        if info is None:
            return
        if info["num_embeddings"] >= self._max_emb_per_face:
            return
        crop_jpeg = self._encode_crop(crop_bgr)
        self._db.add_embedding(face_id, embedding, crop_jpeg)

    @staticmethod
    def _encode_crop(crop_bgr: np.ndarray, max_side: int = 112) -> bytes:
        """Encode a face crop as a small JPEG thumbnail."""
        h, w = crop_bgr.shape[:2]
        if max(h, w) > max_side:
            scale = max_side / max(h, w)
            crop_bgr = cv2.resize(crop_bgr, (int(w * scale), int(h * scale)))
        _, buf = cv2.imencode(".jpg", crop_bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return bytes(buf)

    # Expose DB for API access
    @property
    def face_db(self) -> FaceDB:
        return self._db


# ======================================================================
# Entry point
# ======================================================================
def main(args=None):
    rclpy.init(args=args)
    node = FaceRecognitionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.face_db.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
