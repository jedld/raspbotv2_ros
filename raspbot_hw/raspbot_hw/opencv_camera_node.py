import glob
import os
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


def _find_usb_camera() -> int:
    """Auto-detect a USB webcam by scanning /sys/class/video4linux.

    Skips platform devices (CSI/PiSP/rpivid) and returns the lowest
    /dev/videoN index that is a USB Video Class capture device.
    Returns -1 if nothing found.
    """
    best = -1
    for entry in sorted(glob.glob('/sys/class/video4linux/video*')):
        idx_str = os.path.basename(entry).replace('video', '')
        try:
            idx = int(idx_str)
        except ValueError:
            continue

        # Read the device name
        name_path = os.path.join(entry, 'name')
        if not os.path.isfile(name_path):
            continue
        try:
            name = open(name_path).read().strip()
        except OSError:
            continue

        # Skip known platform / ISP / codec devices
        lower = name.lower()
        skip_keywords = ('pispbe', 'rp1-cfe', 'rpivid', 'bcm2835',
                         'unicam', 'isp', 'codec', 'scaler')
        if any(kw in lower for kw in skip_keywords):
            continue

        # Check that the device bus is USB (follow symlink into /sys/devices/...usb...)
        dev_link = os.path.realpath(os.path.join(entry, 'device'))
        if 'usb' not in dev_link and 'USB' not in dev_link:
            continue

        # Prefer the first (lowest-numbered) capture device
        # V4L2 USB cameras typically expose two nodes: video-capture
        # and metadata. The capture node lists 'Video Capture' in
        # /sys/.../video4linux/videoN/index == 0.
        index_path = os.path.join(entry, 'index')
        try:
            v4l_index = int(open(index_path).read().strip())
        except (OSError, ValueError):
            v4l_index = 0
        if v4l_index != 0:
            continue

        best = idx
        break

    return best


class OpenCVCameraNode(Node):
    def __init__(self):
        super().__init__('opencv_camera')

        self.declare_parameter('device_index', -1)
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('jpeg_quality', 70)
        self.declare_parameter('topic', 'image_raw/compressed')

        self._device_index = int(self.get_parameter('device_index').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._width = int(self.get_parameter('width').value)
        self._height = int(self.get_parameter('height').value)
        self._fps = float(self.get_parameter('fps').value)
        self._jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self._topic = str(self.get_parameter('topic').value)

        # Auto-detect USB webcam when device_index is -1
        if self._device_index < 0:
            detected = _find_usb_camera()
            if detected < 0:
                raise RuntimeError(
                    'Auto-detect: no USB webcam found. '
                    'Set device_index explicitly or check connections.'
                )
            self.get_logger().info(
                f'Auto-detected USB webcam at /dev/video{detected}'
            )
            self._device_index = detected

        try:
            import cv2  # type: ignore
        except Exception as e:
            raise RuntimeError(
                'cv2 is required for opencv_camera_node. Install python3-opencv.'
            ) from e

        self._cv2 = cv2
        # Use V4L2 backend explicitly to avoid GStreamer/libcamera conflicts
        self._cap = self._cv2.VideoCapture(self._device_index, self._cv2.CAP_V4L2)
        self._cap.set(self._cv2.CAP_PROP_FRAME_WIDTH, self._width)
        self._cap.set(self._cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        self._cap.set(self._cv2.CAP_PROP_FPS, self._fps)
        # Prefer MJPEG for lower CPU usage on USB cameras
        self._cap.set(self._cv2.CAP_PROP_FOURCC,
                      self._cv2.VideoWriter_fourcc(*'MJPG'))
        # Minimize V4L2 kernel buffer queue to reduce latency
        self._cap.set(self._cv2.CAP_PROP_BUFFERSIZE, 2)

        if not self._cap.isOpened():
            raise RuntimeError(f'Failed to open camera device index {self._device_index}')

        # Check if we can grab raw MJPEG to avoid decode→re-encode
        fourcc = int(self._cap.get(self._cv2.CAP_PROP_FOURCC))
        mjpg_fourcc = self._cv2.VideoWriter_fourcc(*'MJPG')
        self._raw_mjpeg = (fourcc == mjpg_fourcc)
        if self._raw_mjpeg:
            # Request raw buffer (no BGR conversion)
            self._cap.set(self._cv2.CAP_PROP_CONVERT_RGB, 0)

        self._pub = self.create_publisher(CompressedImage, self._topic, 10)
        self._stopped = False
        self._capture_thread = threading.Thread(
            target=self._capture_loop, daemon=True
        )
        self._capture_thread.start()

        self.get_logger().info(
            f'Camera ready (device={self._device_index}, {self._width}x{self._height}@{self._fps}Hz, '
            f'raw_mjpeg={self._raw_mjpeg}) publishing {self._topic}'
        )

    def _capture_loop(self) -> None:
        """Blocking capture loop in dedicated thread.

        When the USB camera negotiates MJPG FourCC we grab raw JPEG
        buffers directly and skip the decode→re-encode round-trip.
        This is the primary CPU saving for USB webcams.
        """
        cv2 = self._cv2
        min_period = 1.0 / max(self._fps, 1e-3)
        raw_mjpeg = self._raw_mjpeg

        # Pre-allocate imencode params (only used in non-MJPEG path)
        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), int(self._jpeg_quality)]

        while not self._stopped:
            # Skip work if nobody is subscribed
            if self._pub.get_subscription_count() == 0:
                self._cap.grab()
                time.sleep(min_period)
                continue

            t0 = time.monotonic()

            if raw_mjpeg:
                # Grab raw JPEG buffer — no BGR decode → no re-encode
                ok = self._cap.grab()
                if not ok:
                    time.sleep(0.05)
                    continue
                ok, raw = self._cap.retrieve(0, cv2.CAP_PROP_CONVERT_RGB)
                if not ok or raw is None:
                    time.sleep(0.05)
                    continue
                jpeg_bytes = bytes(raw.data)
            else:
                ok, frame = self._cap.read()
                if not ok:
                    self.get_logger().warn(
                        'Camera frame grab failed',
                        throttle_duration_sec=5.0,
                    )
                    time.sleep(0.05)
                    continue
                ok, buf = cv2.imencode('.jpg', frame, encode_params)
                if not ok:
                    continue
                jpeg_bytes = buf.tobytes()

            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._frame_id
            msg.format = 'jpeg'
            msg.data = jpeg_bytes
            self._pub.publish(msg)

            elapsed = time.monotonic() - t0
            remaining = min_period - elapsed
            if remaining > 0:
                time.sleep(remaining)

    def destroy_node(self):
        self._stopped = True
        try:
            if self._capture_thread.is_alive():
                self._capture_thread.join(timeout=2.0)
        except Exception:
            pass
        try:
            self._cap.release()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = OpenCVCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
