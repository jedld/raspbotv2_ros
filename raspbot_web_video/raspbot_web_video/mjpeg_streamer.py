#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rcl_interfaces.msg import ParameterDescriptor

import threading
import asyncio
import time
import io
import cv2
import numpy as np
from typing import Optional, List, Tuple
from fastapi import FastAPI, Response
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
import uvicorn
from contextlib import asynccontextmanager

# -----------------------------------------------------------------------------
# Frame Buffer Logic (Ported from web_video_server.py)
# -----------------------------------------------------------------------------
class LatestFrame:
    def __init__(self, jpeg: Optional[bytes], stamp: float):
        self.jpeg = jpeg
        self.stamp_monotonic = stamp

class AsyncFrameBuffer:
    def __init__(self):
        self._latest = LatestFrame(None, 0.0)
        self._lock = threading.Lock()
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._waiters: List[asyncio.Future] = []

    def set_loop(self, loop: asyncio.AbstractEventLoop):
        self._loop = loop

    def update(self, jpeg: bytes):
        now = time.monotonic()
        with self._lock:
            self._latest = LatestFrame(jpeg, now)
            # Notify waiters in the event loop safely
            if self._loop and not self._loop.is_closed() and self._waiters:
                self._loop.call_soon_threadsafe(self._notify_all, self._latest)

    def _notify_all(self, frame: LatestFrame):
        """Called within the event loop."""
        # Snapshot the list and clear it immediately
        to_notify = self._waiters[:]
        self._waiters.clear()
        for fut in to_notify:
            if not fut.done():
                fut.set_result(frame)

    async def wait_for_newer(self, last_stamp: float, timeout: float = 1.0) -> LatestFrame:
        """Called from FastAPI route (async)."""
        # 1. Check if we already have a newer frame
        with self._lock:
            latest = self._latest
            
        if last_stamp == 0.0 and latest.jpeg is not None:
             return latest

        if latest.stamp_monotonic > last_stamp:
            return latest

        # 2. Wait for a new frame
        if self._loop is None:
            await asyncio.sleep(0.01)
            return latest

        fut = self._loop.create_future()
        self._waiters.append(fut)
        
        try:
            return await asyncio.wait_for(fut, timeout=timeout)
        except asyncio.TimeoutError:
            return latest
        except Exception:
            return latest

# -----------------------------------------------------------------------------
# Caching / Resize Logic
# -----------------------------------------------------------------------------
_resize_cache = {}  # { (stamp, w, h, q): jpeg_bytes }
_resize_lock = threading.Lock()

def _get_resized_jpeg(jpeg_in: bytes, w: int, h: int, q: int) -> bytes:
    # 1. Decode
    np_arr = np.frombuffer(jpeg_in, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    if img is None:
        return jpeg_in
    
    # 2. Resize
    # Use INTER_NEAREST for speed if drastic downscale, else LINEAR
    resized = cv2.resize(img, (w, h), interpolation=cv2.INTER_LINEAR)
    
    # 3. Encode
    params = [int(cv2.IMWRITE_JPEG_QUALITY), q]
    success, encoded_img = cv2.imencode('.jpg', resized, params)
    if not success:
        return jpeg_in
    return encoded_img.tobytes()

async def async_get_cached_lo(loop, cache_key_prefix, stamp, jpeg_data, w, h, q):
    """
    Check cache. If miss, run resize in thread executor to avoid blocking event loop.
    """
    key = (cache_key_prefix, stamp, w, h, q)
    with _resize_lock:
        if key in _resize_cache:
            return _resize_cache[key]
    
    # Expire old cache
    with _resize_lock:
        if len(_resize_cache) > 20:
            _resize_cache.clear()

    # Run heavy lifting in thread
    out_jpeg = await loop.run_in_executor(None, _get_resized_jpeg, jpeg_data, w, h, q)
    
    with _resize_lock:
        _resize_cache[key] = out_jpeg
    return out_jpeg


# -----------------------------------------------------------------------------
# Node & App
# -----------------------------------------------------------------------------
node = None
buffer = AsyncFrameBuffer()

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    loop = asyncio.get_running_loop()
    buffer.set_loop(loop)
    yield
    # Shutdown

app = FastAPI(lifespan=lifespan)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["GET"],
    allow_headers=["*"],
)

@app.get("/stream.mjpg")
async def stream_mjpg(w: int = 0, h: int = 0, q: int = 0):
    # Retrieve framerate limit from node param
    fps_limit = 15.0
    if node:
        fps_limit = node.get_parameter("framerate").value

    return StreamingResponse(
        mjpeg_generator(buffer, w, h, q, fps_limit),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )

async def mjpeg_generator(buf: AsyncFrameBuffer, w: int, h: int, q: int, fps_limit: float):
    last_stamp = 0.0
    loop = asyncio.get_running_loop()
    
    while True:
        frame = await buf.wait_for_newer(last_stamp, timeout=1.0)
        
        if frame.jpeg is None:
            # Keep alive packet if no video?
            # Or just wait again
            if frame.stamp_monotonic <= last_stamp:
                 # Timeout occurred, yield nothing or dummy?
                 # MJPEG needs data.
                 # Just continue and wait again (wait_for_newer returns latest on timeout)
                 pass
        
        if frame.stamp_monotonic > last_stamp:
            last_stamp = frame.stamp_monotonic
            
            # Helper logic: defaults
            req_w = w if w > 0 else 640
            req_h = h if h > 0 else 480
            req_q = q if q > 0 else 80
            
            # Simple heuristic: if w/h/q are 0, use original (no resize)
            if w == 0 and h == 0 and q == 0:
                out_data = frame.jpeg
            else:
                 # If requested size > current or similar, maybe skip resize?
                 # Check cache
                 out_data = await async_get_cached_lo(loop, "stream", frame.stamp_monotonic, frame.jpeg, req_w, req_h, req_q)
            
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + out_data + b'\r\n')
        else:
            # Prevent hot loop if timeout returns immediate on closed loop etc
            await asyncio.sleep(0.01)

class MjpegStreamerNode(Node):
    def __init__(self):
        super().__init__('mjpeg_streamer')
        
        self.declare_parameter('topic', 'image_raw/compressed')
        self.declare_parameter('port', 8001)
        self.declare_parameter('framerate', 15.0)
        
        topic = self.get_parameter('topic').value
        port = self.get_parameter('port').value
        fps = self.get_parameter('framerate').value
        
        self.get_logger().info(f"Starting MJPEG Streamer on port {port} for topic {topic} @ {fps} FPS")
        
        self.create_subscription(CompressedImage, topic, self._on_frame, 10)
        
    def _on_frame(self, msg):
        # Update buffer
        # This runs in ROS thread (executor).
        # AsyncFrameBuffer relies on asyncio loop which runs in Uvicorn thread.
        # But `update` method calls `loop.call_soon_threadsafe`, so it is safe.
        buffer.update(msg.data)

def main(args=None):
    rclpy.init(args=args)
    global node
    node = MjpegStreamerNode()
    
    port = node.get_parameter('port').value
    
    # Run ROS in a separate thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    
    # Run Uvicorn
    uvicorn.run(app, host="0.0.0.0", port=port, log_level="error")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
