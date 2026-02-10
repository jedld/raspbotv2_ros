"""
Pi Camera Module 3 (IMX708) ROS 2 node.

Captures frames via libcamera and publishes CompressedImage (JPEG).

Supports multiple capture backends in priority order:
  1. libcamera Python bindings (best quality, direct ISP pipeline)
  2. OpenCV + GStreamer libcamerasrc pipeline (fallback)
  3. OpenCV + V4L2 device index (last resort)

Requires:
  - dtoverlay=imx708 in /boot/firmware/config.txt (or camera_auto_detect=1)
  - libcamera with Raspberry Pi IPA modules, OR GStreamer + gst-libcamera plugin
"""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class PiCameraNode(Node):
    def __init__(self):
        super().__init__('pi_camera')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('topic', 'front_camera/compressed')
        self.declare_parameter('frame_id', 'front_camera_link')
        # Capture backend: auto | libcamera | gstreamer | v4l2
        self.declare_parameter('backend', 'auto')
        # V4L2 device index (only used with v4l2 backend)
        self.declare_parameter('v4l2_device_index', -1)
        # Camera index for libcamera (0 = first CSI camera enumerated)
        self.declare_parameter('camera_index', 0)

        self._width = int(self.get_parameter('width').value)
        self._height = int(self.get_parameter('height').value)
        self._fps = float(self.get_parameter('fps').value)
        self._jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self._topic = str(self.get_parameter('topic').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._backend = str(self.get_parameter('backend').value).lower()
        self._v4l2_device_index = int(self.get_parameter('v4l2_device_index').value)
        self._camera_index = int(self.get_parameter('camera_index').value)

        try:
            import cv2  # type: ignore
        except ImportError as e:
            raise RuntimeError(
                'cv2 is required for pi_camera_node. Install python3-opencv.'
            ) from e
        self._cv2 = cv2

        # ── Open capture ──────────────────────────────────────────────
        self._cap = None
        self._picam2 = None  # only used for libcamera backend
        self._capture_method = None  # 'libcamera' | 'gstreamer' | 'v4l2'

        if self._backend == 'auto':
            if not self._try_gstreamer():
                if not self._try_v4l2():
                    raise RuntimeError(
                        'Pi Camera: all capture backends failed. '
                        'Ensure dtoverlay=imx708 is set and reboot.'
                    )
        elif self._backend == 'gstreamer':
            if not self._try_gstreamer():
                raise RuntimeError('Pi Camera: GStreamer backend failed.')
        elif self._backend == 'v4l2':
            if not self._try_v4l2():
                raise RuntimeError('Pi Camera: V4L2 backend failed.')
        else:
            raise RuntimeError(f'Pi Camera: unknown backend "{self._backend}"')

        # ── Publisher + timer ─────────────────────────────────────────
        self._pub = self.create_publisher(CompressedImage, self._topic, 10)
        self._timer = self.create_timer(1.0 / max(self._fps, 1e-3), self._tick)

        self.get_logger().info(
            f'Pi Camera ready (backend={self._capture_method}, '
            f'{self._width}x{self._height}@{self._fps}Hz) '
            f'publishing {self._topic}'
        )

    # ── Backend initialisation helpers ────────────────────────────────

    def _try_gstreamer(self) -> bool:
        """Try OpenCV capture via GStreamer libcamerasrc pipeline."""
        try:
            cv2 = self._cv2
            pipeline = (
                f'libcamerasrc camera-name="/base/axi/pcie@120000/rp1/i2c@80000/imx708@1a" ! '
                f'video/x-raw,width={self._width},height={self._height},'
                f'framerate={int(self._fps)}/1 ! '
                f'videoconvert ! appsink drop=true sync=false'
            )
            self.get_logger().info(f'Pi Camera: trying GStreamer pipeline: {pipeline}')
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if cap.isOpened():
                # Verify we can actually read a frame
                ok, _ = cap.read()
                if ok:
                    self._cap = cap
                    self._capture_method = 'gstreamer'
                    return True
                cap.release()
            # Try generic libcamerasrc (no camera-name filter)
            pipeline_generic = (
                f'libcamerasrc ! '
                f'video/x-raw,width={self._width},height={self._height},'
                f'framerate={int(self._fps)}/1 ! '
                f'videoconvert ! appsink drop=true sync=false'
            )
            self.get_logger().info(f'Pi Camera: trying generic GStreamer pipeline')
            cap = cv2.VideoCapture(pipeline_generic, cv2.CAP_GSTREAMER)
            if cap.isOpened():
                ok, _ = cap.read()
                if ok:
                    self._cap = cap
                    self._capture_method = 'gstreamer'
                    return True
                cap.release()
        except Exception as e:
            self.get_logger().warn(f'Pi Camera: GStreamer init failed: {e}')
        return False

    def _try_v4l2(self) -> bool:
        """Try OpenCV capture via V4L2 on a specific device index."""
        cv2 = self._cv2
        indices_to_try = []

        if self._v4l2_device_index >= 0:
            indices_to_try = [self._v4l2_device_index]
        else:
            # Auto-discover: scan /dev/video* for non-USB cameras
            import glob
            import subprocess
            for dev in sorted(glob.glob('/dev/video*')):
                try:
                    idx = int(dev.replace('/dev/video', ''))
                except ValueError:
                    continue
                # Skip low indices likely to be the USB webcam
                if idx < 10:
                    continue
                indices_to_try.append(idx)

        for idx in indices_to_try:
            try:
                self.get_logger().info(f'Pi Camera: trying V4L2 /dev/video{idx}')
                cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
                if cap.isOpened():
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
                    cap.set(cv2.CAP_PROP_FPS, self._fps)
                    ok, _ = cap.read()
                    if ok:
                        self._cap = cap
                        self._capture_method = f'v4l2:/dev/video{idx}'
                        return True
                    cap.release()
            except Exception:
                pass
        return False

    # ── Frame capture ─────────────────────────────────────────────────

    def _tick(self) -> None:
        frame = self._capture_frame()
        if frame is None:
            return

        cv2 = self._cv2
        ok, buf = cv2.imencode(
            '.jpg',
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), int(self._jpeg_quality)],
        )
        if not ok:
            self.get_logger().warn('Pi Camera: JPEG encode failed')
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.format = 'jpeg'
        msg.data = buf.tobytes()
        self._pub.publish(msg)

    def _capture_frame(self):
        """Read a frame from whichever backend is active. Returns numpy array or None."""
        if self._cap is not None:
            ok, frame = self._cap.read()
            if not ok:
                self.get_logger().warn('Pi Camera: frame grab failed', throttle_duration_sec=5.0)
                return None
            return frame
        return None

    # ── Cleanup ───────────────────────────────────────────────────────

    def destroy_node(self):
        try:
            if self._cap is not None:
                self._cap.release()
        except Exception:
            pass
        try:
            if self._picam2 is not None:
                self._picam2.stop()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = PiCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
