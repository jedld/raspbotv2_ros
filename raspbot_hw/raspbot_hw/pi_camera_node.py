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
        # Autofocus mode for IMX708 (Pi Camera Module 3).
        #   0 = manual (no AF), 1 = auto (trigger once), 2 = continuous
        # Continuous AF is recommended for a moving robot.
        self.declare_parameter('af_mode', 2)
        # Autofocus range: 0 = normal, 1 = macro, 2 = full
        self.declare_parameter('af_range', 2)
        # Autofocus speed: 0 = normal, 1 = fast
        self.declare_parameter('af_speed', 1)

        self._width = int(self.get_parameter('width').value)
        self._height = int(self.get_parameter('height').value)
        self._fps = float(self.get_parameter('fps').value)
        self._jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self._topic = str(self.get_parameter('topic').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._backend = str(self.get_parameter('backend').value).lower()
        self._v4l2_device_index = int(self.get_parameter('v4l2_device_index').value)
        self._camera_index = int(self.get_parameter('camera_index').value)
        self._af_mode = int(self.get_parameter('af_mode').value)
        self._af_range = int(self.get_parameter('af_range').value)
        self._af_speed = int(self.get_parameter('af_speed').value)

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
        """Try OpenCV capture via GStreamer libcamerasrc pipeline.

        On Pi 5 with PiSP, libcamerasrc auto-negotiates the sensor
        format internally.  We let it choose the best raw mode, then
        use videoconvert + videoscale to deliver the requested BGR
        resolution to the OpenCV appsink.
        """
        try:
            cv2 = self._cv2

            # Discover the IMX708 camera-name from /dev/media*
            camera_name = self._discover_libcamera_name()

            pipelines_to_try = []

            # Build autofocus properties for libcamerasrc.
            # Newer GStreamer libcamera plugins expose af-mode, af-range,
            # af-speed as direct element properties (not extra-controls).
            af_props = self._build_af_properties()

            # Pipeline 1: explicit camera-name with auto-negotiation
            if camera_name:
                pipelines_to_try.append(
                    f'libcamerasrc camera-name="{camera_name}"{af_props} ! '
                    f'videoconvert ! videoscale ! '
                    f'video/x-raw,format=BGR,width={self._width},'
                    f'height={self._height} ! '
                    f'appsink drop=true sync=false'
                )

            # Pipeline 2: generic libcamerasrc (auto camera)
            pipelines_to_try.append(
                f'libcamerasrc{af_props} ! '
                f'videoconvert ! videoscale ! '
                f'video/x-raw,format=BGR,width={self._width},'
                f'height={self._height} ! '
                f'appsink drop=true sync=false'
            )

            for pipeline in pipelines_to_try:
                self.get_logger().info(
                    f'Pi Camera: trying GStreamer pipeline: {pipeline}'
                )
                cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
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

    @staticmethod
    def _discover_libcamera_name() -> str:
        """Return the device-tree path of the first CSI camera, or ''."""
        import glob
        import os
        for media in sorted(glob.glob('/sys/bus/media/devices/media*')):
            model_path = os.path.join(media, 'model')
            try:
                model = open(model_path).read().strip()
            except OSError:
                continue
            if model == 'rp1-cfe':
                # Walk sub-entities looking for an imx708 sensor
                for entity in sorted(glob.glob(os.path.join(media, 'device', 'v4l-subdev*'))):
                    name_path = os.path.join(entity, 'name')
                    try:
                        name = open(name_path).read().strip()
                    except OSError:
                        continue
                    if 'imx708' in name.lower():
                        # Resolve the DT path from of_node
                        of_node = os.path.join(entity, 'of_node')
                        if os.path.islink(of_node):
                            dt = os.path.realpath(of_node)
                            # Strip the /proc/device-tree prefix
                            dt = dt.replace('/proc/device-tree', '')
                            return dt
        return ''

    # GStreamer enum names for libcamerasrc AF properties.
    _AF_MODE_NAMES = {0: 'manual', 1: 'auto', 2: 'continuous'}
    _AF_RANGE_NAMES = {0: 'normal', 1: 'macro', 2: 'full'}
    _AF_SPEED_NAMES = {0: 'normal', 1: 'fast'}

    def _build_af_properties(self) -> str:
        """Build GStreamer element properties for libcamerasrc autofocus.

        Modern GStreamer libcamera plugins (v0.5+) expose AF controls as
        direct element properties using string enum values:
            af-mode:  manual | auto | continuous
            af-range: normal | macro | full
            af-speed: normal | fast
        Negative param values disable that property (use camera default).
        """
        parts = []
        name = self._AF_MODE_NAMES.get(self._af_mode)
        if name:
            parts.append(f'af-mode={name}')
        name = self._AF_RANGE_NAMES.get(self._af_range)
        if name:
            parts.append(f'af-range={name}')
        name = self._AF_SPEED_NAMES.get(self._af_speed)
        if name:
            parts.append(f'af-speed={name}')
        if not parts:
            return ''
        return ' ' + ' '.join(parts)

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
