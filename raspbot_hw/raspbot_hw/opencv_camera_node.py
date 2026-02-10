import glob
import os
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
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('jpeg_quality', 80)
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

        if not self._cap.isOpened():
            raise RuntimeError(f'Failed to open camera device index {self._device_index}')

        self._pub = self.create_publisher(CompressedImage, self._topic, 10)
        self._timer = self.create_timer(1.0 / max(self._fps, 1e-3), self._tick)

        self.get_logger().info(
            f'Camera ready (device={self._device_index}, {self._width}x{self._height}@{self._fps}Hz) publishing {self._topic}'
        )

    def _tick(self) -> None:
        ok, frame = self._cap.read()
        if not ok:
            self.get_logger().warn('Camera frame grab failed')
            return

        ok, buf = self._cv2.imencode(
            '.jpg',
            frame,
            [int(self._cv2.IMWRITE_JPEG_QUALITY), int(self._jpeg_quality)],
        )
        if not ok:
            self.get_logger().warn('JPEG encode failed')
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.format = 'jpeg'
        msg.data = buf.tobytes()
        self._pub.publish(msg)

    def destroy_node(self):
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
