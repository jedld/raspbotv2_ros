import threading
import time
from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from urllib.parse import parse_qs, urlparse
from typing import Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float64, String, Int32MultiArray


INDEX_HTML = """<!doctype html>
<html lang=\"en\">
<head>
  <meta charset=\"utf-8\" />
  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\" />
  <title>Raspbot Camera</title>
  <style>
    body { font-family: system-ui, sans-serif; margin: 16px; }
    .wrap { max-width: 960px; margin: 0 auto; }
        .videoWrap { position: relative; width: 100%; }
        img { width: 100%; height: auto; background: #111; border-radius: 8px; display: block; }
        canvas.overlay { position: absolute; left: 0; top: 0; width: 100%; height: 100%; pointer-events: none; }
    code { background: #f2f2f2; padding: 2px 6px; border-radius: 4px; }
    .muted { color: #666; }
        .card { border: 1px solid #e6e6e6; border-radius: 10px; padding: 12px; margin-top: 12px; }
        .row { display: flex; gap: 12px; flex-wrap: wrap; align-items: center; }
        .row > * { flex: 1 1 220px; }
        input[type=range] { width: 100%; }
        button { padding: 8px 12px; border-radius: 8px; border: 1px solid #ccc; background: #fafafa; cursor: pointer; }
        button:hover { background: #f0f0f0; }
                button.primary { background: #111; color: #fff; border-color: #111; }
                button.primary:hover { background: #222; }
                button.danger { background: #b00020; color: #fff; border-color: #b00020; }
                button.danger:hover { background: #8f001a; }
        .kv { font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace; font-size: 12px; }
  </style>
</head>
<body>
  <div class=\"wrap\">
    <h1>Raspbot Camera</h1>
    <p class=\"muted\">Streaming MJPEG from ROS 2 <code>sensor_msgs/CompressedImage</code>.</p>
        <div class=\"videoWrap\" id=\"videoWrap\">
            <img id=\"video\" src=\"/stream.mjpg\" alt=\"camera stream\" />
            <canvas id=\"overlay\" class=\"overlay\"></canvas>
        </div>

        <div class=\"card\">
            <h2>Gimbal</h2>
            <p class=\"muted\">Pan/tilt control publishes to <code>camera_gimbal/command_deg</code> (x=pan, y=tilt).</p>

            <div class=\"row\">
                <div>
                    <label for=\"pan\">Pan: <span id=\"panVal\"></span>°</label>
                    <input id=\"pan\" type=\"range\" min=\"0\" max=\"180\" step=\"1\" />
                </div>
                <div>
                    <label for=\"tilt\">Tilt: <span id=\"tiltVal\"></span>°</label>
                    <input id=\"tilt\" type=\"range\" min=\"0\" max=\"110\" step=\"1\" />
                </div>
            </div>

            <div class=\"row\" style=\"margin-top: 8px;\">
                <button id=\"sendBtn\">Send</button>
                <button id=\"centerBtn\">Center</button>
                <div class=\"kv\">Status: <span id=\"status\">loading…</span></div>
            </div>
        </div>

        <div class=\"card\">
            <h2>Drive (WASD)</h2>
            <p class=\"muted\">Keyboard teleop publishes to <code>cmd_vel</code>. Click this page once so it has focus.</p>
            <div class=\"row\">
                <button class=\"danger\" id=\"stopBtn\" title=\"Immediately publish zero cmd_vel\">STOP</button>
                <div class=\"kv\">Keys: <code>W</code>=forward, <code>S</code>=reverse, <code>A</code>=turn left, <code>D</code>=turn right. Hold <code>Shift</code> for faster.</div>
            </div>
            <div class=\"row\" style=\"margin-top: 8px;\">
                <div class=\"kv\">cmd_vel: <span id=\"cmdVel\">0, 0</span></div>
                <div class=\"kv\">Safety: auto-stop on keyup / tab switch.</div>
            </div>
        </div>

        <div class=\"card\">
            <h2>Detection / Tracking</h2>
            <p class=\"muted\">Shows bounding boxes from <code>detections/json</code> and can enable person tracking via <code>tracking/enable</code>.</p>
            <div class=\"row\">
                <button class=\"primary\" id=\"trackBtn\">Enable person tracking</button>
                <label class=\"kv\"><input id=\"boxesChk\" type=\"checkbox\" checked /> Show boxes</label>
                <label class=\"kv\"><input id=\"invPanChk\" type=\"checkbox\" checked /> Invert pan</label>
                <label class=\"kv\"><input id=\"invTiltChk\" type=\"checkbox\" checked /> Invert tilt</label>
                <div class=\"kv\">Tracking: <span id=\"trackState\">off</span></div>
            </div>
            <div class=\"row\" style=\"margin-top: 8px;\">
                <div class=\"kv\">Detections: <span id=\"detState\">n/a</span></div>
            </div>
        </div>

        <div class=\"card\">
            <h2>Auto Follow</h2>
            <p class=\"muted\">Makes the robot base move to follow the detected subject at a set distance. Works alongside gimbal tracking.</p>
            <div class=\"row\">
                <button class=\"primary\" id=\"followBtn\">Enable auto follow</button>
                <div class=\"kv\">Follow: <span id=\"followState\">off</span></div>
            </div>
            <div class=\"row\" style=\"margin-top: 8px;\">
                <div>
                    <label for=\"followDist\">Target distance (bbox area): <span id=\"followDistVal\">0.04</span></label>
                    <input id=\"followDist\" type=\"range\" min=\"0.01\" max=\"0.30\" step=\"0.01\" value=\"0.04\" />
                </div>
                <div>
                    <label for=\"followSpeed\">Max speed: <span id=\"followSpeedVal\">0.30</span> m/s</label>
                    <input id=\"followSpeed\" type=\"range\" min=\"0.05\" max=\"0.50\" step=\"0.05\" value=\"0.30\" />
                </div>
            </div>
            <div class=\"row\" style=\"margin-top: 4px;\">
                <div class=\"kv\">Tip: stand at the desired distance and read the detection bbox area, then set the slider to match.</div>
            </div>
        </div>

        <div class=\"card\">
            <h2>&#128247; Snapshot</h2>
            <p class=\"muted\">Capture a full-resolution still from the current camera frame.</p>
            <div class=\"row\">
                <button class=\"primary\" id=\"snapBtn\">Take Photo</button>
                <div class=\"kv\" id=\"snapStatus\"></div>
            </div>
            <div id=\"snapHistory\" style=\"margin-top:8px; font-size:12px;\"></div>
        </div>
  </div>

    <script>
        const pan = document.getElementById('pan');
        const tilt = document.getElementById('tilt');
        const panVal = document.getElementById('panVal');
        const tiltVal = document.getElementById('tiltVal');
        const statusEl = document.getElementById('status');
        const sendBtn = document.getElementById('sendBtn');
        const centerBtn = document.getElementById('centerBtn');
        const stopBtn = document.getElementById('stopBtn');
        const cmdVelEl = document.getElementById('cmdVel');
        const video = document.getElementById('video');
        const overlay = document.getElementById('overlay');
        const trackBtn = document.getElementById('trackBtn');
        const trackStateEl = document.getElementById('trackState');
        const detStateEl = document.getElementById('detState');
        const boxesChk = document.getElementById('boxesChk');
        const invPanChk = document.getElementById('invPanChk');
        const invTiltChk = document.getElementById('invTiltChk');
        const followBtn = document.getElementById('followBtn');
        const followStateEl = document.getElementById('followState');
        const followDist = document.getElementById('followDist');
        const followDistVal = document.getElementById('followDistVal');
        const followSpeed = document.getElementById('followSpeed');
        const followSpeedVal = document.getElementById('followSpeedVal');

        let debounceTimer = null;

        /* Track when a user last touched each slider so the status poll
           does not overwrite the value while the user is still dragging. */
        const _lastUserInput = {};  // element-id → timestamp
        const USER_INPUT_GRACE_MS = 3000;
        function markUserInput(el) { _lastUserInput[el.id] = Date.now(); }
        function userRecentlyTouched(el) {
            const t = _lastUserInput[el.id];
            return t && (Date.now() - t) < USER_INPUT_GRACE_MS;
        }

        function updateLabels() {
            panVal.textContent = String(pan.value);
            tiltVal.textContent = String(tilt.value);
        }

        async function fetchStatus() {
            try {
                const r = await fetch('/status');
                const j = await r.json();
                if (j.gimbal) {
                    if (j.gimbal.limits) {
                        pan.min = j.gimbal.limits.pan_min_deg;
                        pan.max = j.gimbal.limits.pan_max_deg;
                        tilt.min = j.gimbal.limits.tilt_min_deg;
                        tilt.max = j.gimbal.limits.tilt_max_deg;
                    }
                    if (j.gimbal.state) {
                        pan.value = Math.round(j.gimbal.state.pan_deg);
                        tilt.value = Math.round(j.gimbal.state.tilt_deg);
                        updateLabels();
                    }
                }
                statusEl.textContent = `frames=${j.frame_count ?? '?'} last_frame_age_s=${j.last_frame_age_s ?? 'null'}`;
                if (j.tracking) {
                    const enabled = Boolean(j.tracking.enabled);
                    setTrackingUi(enabled);

                    // Optional runtime tuning
                    if (typeof j.tracking.pan_sign === 'number') {
                        invPanChk.checked = (Number(j.tracking.pan_sign) < 0);
                    }
                    if (typeof j.tracking.tilt_sign === 'number') {
                        invTiltChk.checked = (Number(j.tracking.tilt_sign) < 0);
                    }
                }
                if (j.follow) {
                    setFollowUi(Boolean(j.follow.enabled));
                    if (typeof j.follow.target_bbox_area === 'number' && !userRecentlyTouched(followDist)) {
                        followDist.value = j.follow.target_bbox_area.toFixed(2);
                        followDistVal.textContent = j.follow.target_bbox_area.toFixed(2);
                    }
                    if (typeof j.follow.max_linear === 'number' && !userRecentlyTouched(followSpeed)) {
                        followSpeed.value = j.follow.max_linear.toFixed(2);
                        followSpeedVal.textContent = j.follow.max_linear.toFixed(2);
                    }
                }
            } catch (e) {
                statusEl.textContent = 'status error';
            }
        }

        function setTrackingUi(enabled) {
            trackStateEl.textContent = enabled ? 'on' : 'off';
            trackBtn.textContent = enabled ? 'Disable person tracking' : 'Enable person tracking';
        }

        async function setTracking(enabled) {
            try {
                await fetch('/api/tracking', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({enabled: Boolean(enabled)})
                });
            } catch (e) {
                // ignore
            }
        }

        async function setTrackingConfig(invertPan, invertTilt) {
            try {
                await fetch('/api/tracking/config', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({invert_pan: Boolean(invertPan), invert_tilt: Boolean(invertTilt)})
                });
            } catch (e) {
                // ignore
            }
        }

        function setFollowUi(enabled) {
            followStateEl.textContent = enabled ? 'on' : 'off';
            followBtn.textContent = enabled ? 'Disable auto follow' : 'Enable auto follow';
        }

        async function setFollow(enabled) {
            try {
                await fetch('/api/follow', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({enabled: Boolean(enabled)})
                });
            } catch (e) {
                // ignore
            }
        }

        trackBtn.addEventListener('click', async () => {
            const enabled = (trackStateEl.textContent === 'on');
            await setTracking(!enabled);
            // status poll will reconcile actual state
        });

        followBtn.addEventListener('click', async () => {
            const enabled = (followStateEl.textContent === 'on');
            await setFollow(!enabled);
        });

        async function sendFollowConfig(targetArea, maxLinear) {
            try {
                await fetch('/api/follow/config', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({target_bbox_area: Number(targetArea), max_linear: Number(maxLinear)})
                });
            } catch (e) { /* ignore */ }
        }

        followDist.addEventListener('input', () => {
            markUserInput(followDist);
            followDistVal.textContent = Number(followDist.value).toFixed(2);
            sendFollowConfig(followDist.value, followSpeed.value);
        });
        followSpeed.addEventListener('input', () => {
            markUserInput(followSpeed);
            followSpeedVal.textContent = Number(followSpeed.value).toFixed(2);
            sendFollowConfig(followDist.value, followSpeed.value);
        });

        invPanChk.addEventListener('change', async () => {
            await setTrackingConfig(invPanChk.checked, invTiltChk.checked);
        });
        invTiltChk.addEventListener('change', async () => {
            await setTrackingConfig(invPanChk.checked, invTiltChk.checked);
        });

        // ---- Snapshot ----
        const snapBtn = document.getElementById('snapBtn');
        const snapStatus = document.getElementById('snapStatus');
        const snapHistory = document.getElementById('snapHistory');
        snapBtn.addEventListener('click', async () => {
            snapBtn.disabled = true;
            snapStatus.textContent = 'Capturing…';
            try {
                const r = await fetch('/api/snapshot', {method: 'POST'});
                if (!r.ok) { snapStatus.textContent = 'Error ' + r.status; return; }
                const j = await r.json();
                snapStatus.textContent = '✅ ' + j.filename;
                const link = document.createElement('a');
                link.href = '/api/snapshots/' + encodeURIComponent(j.filename);
                link.target = '_blank';
                link.textContent = j.filename;
                const div = document.createElement('div');
                div.appendChild(link);
                snapHistory.prepend(div);
            } catch (e) {
                snapStatus.textContent = 'Failed';
            } finally {
                snapBtn.disabled = false;
            }
        });

        function resizeOverlay() {
            const w = Math.max(1, Math.floor(video.clientWidth));
            const h = Math.max(1, Math.floor(video.clientHeight));
            if (overlay.width !== w) overlay.width = w;
            if (overlay.height !== h) overlay.height = h;
        }

        window.addEventListener('resize', resizeOverlay);
        video.addEventListener('load', resizeOverlay);

        let latestDet = null;
        let lastDetFetchMs = 0;

        async function fetchDetections() {
            try {
                const r = await fetch('/detections', {cache: 'no-store'});
                const j = await r.json();
                latestDet = j;
                lastDetFetchMs = Date.now();
                const n = Array.isArray(j.detections) ? j.detections.length : 0;
                detStateEl.textContent = `n=${n}`;
            } catch (e) {
                // Keep last detections; just show stale
                detStateEl.textContent = 'error';
            }
        }

        function drawOverlay() {
            resizeOverlay();
            const ctx = overlay.getContext('2d');
            if (!ctx) return;
            ctx.clearRect(0, 0, overlay.width, overlay.height);

            if (!boxesChk.checked) {
                requestAnimationFrame(drawOverlay);
                return;
            }

            const j = latestDet;
            if (!j || !Array.isArray(j.detections) || !j.image_width || !j.image_height) {
                requestAnimationFrame(drawOverlay);
                return;
            }

            const sx = overlay.width / Number(j.image_width);
            const sy = overlay.height / Number(j.image_height);

            ctx.lineWidth = 2;
            ctx.font = '12px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace';
            ctx.textBaseline = 'top';

            for (const d of j.detections) {
                const x = Number(d.x) * sx;
                const y = Number(d.y) * sy;
                const w = Number(d.w) * sx;
                const h = Number(d.h) * sy;
                if (!Number.isFinite(x+y+w+h)) continue;

                ctx.strokeStyle = '#00ff66';
                ctx.fillStyle = 'rgba(0, 0, 0, 0.45)';
                ctx.strokeRect(x, y, w, h);

                const label = String(d.label ?? d.class_id ?? '?');
                const score = Number(d.score);
                const text = Number.isFinite(score) ? `${label} ${(score*100).toFixed(0)}%` : label;
                const tw = ctx.measureText(text).width + 6;
                const th = 14;
                ctx.fillRect(x, Math.max(0, y - th), tw, th);
                ctx.fillStyle = '#ffffff';
                ctx.fillText(text, x + 3, Math.max(0, y - th) + 1);
            }

            // Stale indicator
            const ageMs = Date.now() - lastDetFetchMs;
            if (ageMs > 1000) {
                ctx.fillStyle = 'rgba(255, 165, 0, 0.75)';
                ctx.fillRect(0, 0, 140, 16);
                ctx.fillStyle = '#000';
                ctx.fillText(`detections stale`, 4, 2);
            }

            requestAnimationFrame(drawOverlay);
        }

        async function sendGimbal(p, t) {
            try {
                await fetch('/api/gimbal', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({pan_deg: Number(p), tilt_deg: Number(t)})
                });
            } catch (e) {
                // ignore
            }
        }

        async function sendCmdVel(lin, ang) {
            try {
                await fetch('/api/cmd_vel', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({linear_x: Number(lin), angular_z: Number(ang)})
                });
            } catch (e) {
                // ignore
            }
        }

        async function stopCmdVel() {
            try {
                await fetch('/api/cmd_vel/stop', {method:'POST'});
            } catch (e) {
                // ignore
            }
        }

        function clamp(v, lo, hi) {
            v = Number(v);
            if (Number.isNaN(v)) return lo;
            return Math.max(lo, Math.min(hi, v));
        }

        function nmin(el) { return Number(el.min); }
        function nmax(el) { return Number(el.max); }

        function scheduleSend() {
            if (debounceTimer) clearTimeout(debounceTimer);
            debounceTimer = setTimeout(() => {
                sendGimbal(pan.value, tilt.value);
            }, 120);
        }

        pan.addEventListener('input', () => { updateLabels(); scheduleSend(); });
        tilt.addEventListener('input', () => { updateLabels(); scheduleSend(); });
        sendBtn.addEventListener('click', () => sendGimbal(pan.value, tilt.value));
        centerBtn.addEventListener('click', () => fetch('/api/gimbal/center', {method:'POST'}));
        stopBtn.addEventListener('click', () => stopCmdVel());

        // Arrow keys: pan/tilt in small increments.
        // Left/Right: pan, Up/Down: tilt. Hold Shift for larger steps.
        document.addEventListener('keydown', (ev) => {
            const key = ev.key;
            if (!['ArrowLeft','ArrowRight','ArrowUp','ArrowDown'].includes(key)) return;
            ev.preventDefault();

            const step = ev.shiftKey ? 10 : 2;
            let p = Number(pan.value);
            let t = Number(tilt.value);

            if (key === 'ArrowLeft') p -= step;
            if (key === 'ArrowRight') p += step;
            if (key === 'ArrowUp') t += step;
            if (key === 'ArrowDown') t -= step;

            p = clamp(p, nmin(pan), nmax(pan));
            t = clamp(t, nmin(tilt), nmax(tilt));

            pan.value = String(Math.round(p));
            tilt.value = String(Math.round(t));
            updateLabels();
            scheduleSend();
        }, { passive: false });

        // WASD: drive. We continuously send at ~10Hz while keys are held.
        const pressed = new Set();
        let lastLin = 0;
        let lastAng = 0;
        let driveTimer = null;

        function computeDrive() {
            const fast = pressed.has('shift');
            // These are multipliers; actual limits are enforced server-side.
            const linMag = fast ? 1.0 : 0.5;
            const angMag = fast ? 1.0 : 0.6;

            let lin = 0;
            let ang = 0;
            if (pressed.has('w')) lin += linMag;
            if (pressed.has('s')) lin -= linMag;
            if (pressed.has('a')) ang += angMag;
            if (pressed.has('d')) ang -= angMag;
            return {lin, ang};
        }

        function startDriveLoop() {
            if (driveTimer) return;
            driveTimer = setInterval(async () => {
                const {lin, ang} = computeDrive();
                if (lin !== lastLin || ang !== lastAng) {
                    lastLin = lin;
                    lastAng = ang;
                }
                cmdVelEl.textContent = `${lin.toFixed(2)}, ${ang.toFixed(2)}`;
                await sendCmdVel(lin, ang);
            }, 100);
        }

        function stopDriveLoop() {
            if (driveTimer) {
                clearInterval(driveTimer);
                driveTimer = null;
            }
            pressed.clear();
            lastLin = 0;
            lastAng = 0;
            cmdVelEl.textContent = `0.00, 0.00`;
            stopCmdVel();
        }

        function normalizeKey(ev) {
            if (ev.key === 'Shift') return 'shift';
            return String(ev.key || '').toLowerCase();
        }

        document.addEventListener('keydown', (ev) => {
            const k = normalizeKey(ev);
            if (!['w','a','s','d','shift'].includes(k)) return;
            // prevent browser from scrolling on space etc; WASD doesn't scroll but keep consistent.
            ev.preventDefault();
            pressed.add(k);
            startDriveLoop();
        }, { passive: false });

        document.addEventListener('keyup', (ev) => {
            const k = normalizeKey(ev);
            if (!['w','a','s','d','shift'].includes(k)) return;
            ev.preventDefault();
            pressed.delete(k);
            if (pressed.size === 0) {
                stopDriveLoop();
            }
        }, { passive: false });

        window.addEventListener('blur', () => stopDriveLoop());
        document.addEventListener('visibilitychange', () => {
            if (document.hidden) stopDriveLoop();
        });

        updateLabels();
        fetchStatus();
        setInterval(fetchStatus, 1000);
        fetchDetections();
        setInterval(fetchDetections, 200);
        drawOverlay();
    </script>
</body>
</html>
"""


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
        end = time.monotonic() + max(timeout, 0.0)
        with self._cv:
            while self._latest.stamp_monotonic <= last_stamp:
                remaining = end - time.monotonic()
                if remaining <= 0:
                    break
                self._cv.wait(timeout=remaining)
            return self._latest


class WebVideoNode(Node):
    def __init__(self, frame_buffer: FrameBuffer) -> None:
        super().__init__("web_video")

        self.declare_parameter("topic", "image_raw/compressed")
        self.declare_parameter("bind", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter("fps_limit", 15.0)

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
        self.declare_parameter("follow_enable_topic", "follow/enable")
        self.declare_parameter("follow_target_area_topic", "follow/target_area")
        self.declare_parameter("follow_max_linear_topic", "follow/max_linear")

        self._frame_buffer = frame_buffer
        self._frame_count = 0
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
        self._cmd_angular_z = 0.0

        topic = str(self.get_parameter("topic").value)
        self.get_logger().info(f"Subscribing to {topic}")

        # Use sensor-data QoS for better interoperability with camera publishers.
        self._sub = self.create_subscription(CompressedImage, topic, self._on_img, qos_profile_sensor_data)

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

    def set_cmd_vel(self, linear_x_unit: float, angular_z_unit: float) -> None:
        # Inputs from UI are in approx [-1..1]. Clamp and scale.
        lin_u = self._clamp(float(linear_x_unit), -1.0, 1.0)
        ang_u = self._clamp(float(angular_z_unit), -1.0, 1.0)

        self._cmd_linear_x = float(lin_u) * float(self._max_linear_mps)
        self._cmd_angular_z = float(ang_u) * float(self._max_angular_rps)

        msg = Twist()
        msg.linear.x = float(self._cmd_linear_x)
        msg.angular.z = float(self._cmd_angular_z)
        self._cmd_pub.publish(msg)

        now = time.monotonic()
        self._cmd_last_rx_monotonic = now
        self._cmd_last_sent_monotonic = now

    def stop_cmd_vel(self) -> None:
        self._cmd_linear_x = 0.0
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
            'cmd_vel': {
                'max_linear_mps': float(self._max_linear_mps),
                'max_angular_rps': float(self._max_angular_rps),
                'timeout_sec': float(self._cmd_timeout_sec),
                'state': {
                    'linear_x': float(self._cmd_linear_x),
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
            },
            'follow': {
                'enabled': bool(self._follow_enabled),
                'target_bbox_area': float(self._follow_target_bbox_area),
                'max_linear': float(self._follow_max_linear),
            },
        }

    def _on_detections(self, msg: String) -> None:
        if not msg.data:
            return
        self._latest_detections_json = msg.data
        self._last_detections_monotonic = time.monotonic()

    def get_latest_detections(self) -> dict:
        # Always return valid JSON.
        if not self._latest_detections_json:
            return {"image_width": None, "image_height": None, "detections": []}
        try:
            return json.loads(self._latest_detections_json)
        except Exception:
            return {"error": "invalid_detections_json", "detections": []}

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

    def set_follow_config(self, target_bbox_area: float = None, max_linear: float = None) -> None:
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

    def take_snapshot(self) -> dict:
        """Grab the latest full-resolution JPEG and save to disk. Returns {filename, path}."""
        import os
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


def make_handler(
    *,
    frame_buffer: FrameBuffer,
    fps_limit: float,
    logger,
    status_provider=None,
    detections_provider=None,
    gimbal_setter=None,
    gimbal_center=None,
    cmd_vel_setter=None,
    cmd_vel_stopper=None,
    tracking_setter=None,
    tracking_config_setter=None,
    follow_setter=None,
    follow_config_setter=None,
    snapshot_taker=None,
    snapshot_dir=None,
):
    boundary = b"--frame"
    fps = max(float(fps_limit), 1.0)
    min_period = 1.0 / fps

    class Handler(BaseHTTPRequestHandler):
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
                self.end_headers()
                self.wfile.write(body)
                try:
                    self.wfile.flush()
                except Exception:
                    pass
                return

            if path.startswith("/status"):
                payload = {}
                if callable(status_provider):
                    try:
                        payload = status_provider()
                    except Exception:
                        payload = {'error': 'status_provider_failed'}
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

            if path.startswith("/api/snapshots/"):
                import os
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
                try:
                    self.wfile.flush()
                except Exception:
                    pass

                last_stamp = 0.0
                last_sent_time = 0.0

                try:
                    while True:
                        # Throttle server-side to avoid pegging CPU/network.
                        now = time.monotonic()
                        sleep_needed = (last_sent_time + min_period) - now
                        if sleep_needed > 0:
                            time.sleep(sleep_needed)

                        frame = frame_buffer.wait_for_newer(last_stamp, timeout=1.0)
                        if frame.jpeg is None:
                            continue

                        last_stamp = frame.stamp_monotonic
                        last_sent_time = time.monotonic()

                        headers = (
                            boundary + b"\r\n"
                            b"Content-Type: image/jpeg\r\n" +
                            f"Content-Length: {len(frame.jpeg)}\r\n\r\n".encode("ascii")
                        )
                        self.wfile.write(headers)
                        self.wfile.write(frame.jpeg)
                        self.wfile.write(b"\r\n")
                        try:
                            self.wfile.flush()
                        except Exception:
                            pass
                except (BrokenPipeError, ConnectionResetError):
                    return
                except Exception as e:
                    if logger is not None:
                        logger.warn(f"stream client error: {e!r}")
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
                    ang = float(payload.get('angular_z'))
                except Exception:
                    self.send_response(HTTPStatus.BAD_REQUEST)
                    self.end_headers()
                    return

                if lin != lin or ang != ang:
                    self.send_response(HTTPStatus.BAD_REQUEST)
                    self.end_headers()
                    return

                try:
                    cmd_vel_setter(lin, ang)
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
                    if target_area is not None:
                        target_area = float(target_area)
                    if max_linear is not None:
                        max_linear = float(max_linear)
                except Exception:
                    self.send_response(HTTPStatus.BAD_REQUEST)
                    self.end_headers()
                    return

                try:
                    follow_config_setter(target_area=target_area, max_linear=max_linear)
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

            self.send_response(HTTPStatus.NOT_FOUND)
            self.send_header('Content-Type', 'text/plain; charset=utf-8')
            self.end_headers()
            self.wfile.write(b'Not found\n')

    return Handler


def main() -> None:
    rclpy.init()
    frame_buffer = FrameBuffer()
    node = WebVideoNode(frame_buffer)

    bind = str(node.get_parameter("bind").value)
    port = int(node.get_parameter("port").value)
    fps_limit = float(node.get_parameter("fps_limit").value)

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    handler_cls = make_handler(
        frame_buffer=frame_buffer,
        fps_limit=fps_limit,
        logger=node.get_logger(),
        status_provider=node.status_dict,
        detections_provider=node.get_latest_detections,
        gimbal_setter=node.set_gimbal,
        gimbal_center=node.center_gimbal,
        cmd_vel_setter=node.set_cmd_vel,
        cmd_vel_stopper=node.stop_cmd_vel,
        tracking_setter=node.set_tracking_enabled,
        tracking_config_setter=node.set_tracking_config,
        follow_setter=node.set_follow_enabled,
        follow_config_setter=node.set_follow_config,
        snapshot_taker=node.take_snapshot,
        snapshot_dir=str(node.get_parameter("snapshot_dir").value),
    )
    httpd = ThreadingHTTPServer((bind, port), handler_cls)
    # Allow the serve loop to wake up periodically so Ctrl-C / rclpy shutdown is responsive.
    httpd.timeout = 0.5

    node.get_logger().info(
        f"Web video server on http://{bind}:{port}/ (MJPEG: /stream.mjpg, status: /status, gimbal: /api/gimbal, drive: /api/cmd_vel)"
    )

    try:
        # Don't rely on custom signal handlers here.
        # In ROS 2, Ctrl-C often triggers rclpy shutdown via its own handler.
        # This loop exits when either Ctrl-C raises KeyboardInterrupt or rclpy.ok() becomes false.
        while rclpy.ok():
            httpd.handle_request()
    except KeyboardInterrupt:
        pass
    finally:
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
