import math
import threading
import time
from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from urllib.parse import parse_qs, urlparse
from typing import Optional

import numpy as np

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import CompressedImage, Imu, Range
from std_msgs.msg import Bool, Empty, Float32, Float64, Int32, Int32MultiArray, String, UInt8MultiArray


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

        <div class=\"card\" id=\"collisionCard\">
            <h2>&#128721; Collision Failsafe</h2>
            <p class=\"muted\">Ultrasonic-based forward-motion blocker. Protects all cmd_vel sources (WASD, auto-follow, teleop).</p>
            <div class=\"row\">
                <button class=\"danger\" id=\"collisionToggleBtn\">Disable Failsafe</button>
                <div class=\"kv\">Status: <span id=\"collisionStatus\">unknown</span></div>
                <div class=\"kv\">Distance: <span id=\"collisionDist\">—</span></div>
            </div>
            <div class=\"row\" style=\"margin-top: 4px;\">
                <div class=\"kv\" style=\"font-size:12px;\">Stop &lt; 0.15 m &nbsp;|&nbsp; Slow 0.15–0.35 m &nbsp;|&nbsp; Clear &gt; 0.35 m</div>
            </div>
        </div>

        <div class=\"card\" id=\"cliffCard\">
            <h2>&#9888; Cliff / Edge Failsafe</h2>
            <p class=\"muted\">Line-tracker IR sensors detect missing floor and block forward motion. Backward/strafe/rotation always allowed.</p>
            <div class=\"row\">
                <button class=\"danger\" id=\"cliffToggleBtn\">Disable Failsafe</button>
                <div class=\"kv\">Status: <span id=\"cliffStatus\">unknown</span></div>
                <div class=\"kv\">Sensors: <span id=\"cliffSensors\">&mdash;</span></div>
            </div>
            <div class=\"row\" style=\"margin-top: 4px;\">
                <div class=\"kv\" style=\"font-size:12px;\">&#9632;=floor &nbsp; &#9633;=edge/no floor &nbsp;|&nbsp; Sensors: L1 L2 (left) R1 R2 (right)</div>
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
            <p class=\"muted\">Makes the robot base move to follow the detected subject at a set distance. Uses mecanum strafing and IMU feedback for smooth, direct tracking.</p>
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
            <div class=\"row\" style=\"margin-top: 8px;\">
                <div>
                    <label for=\"followStrafe\">Strafe gain: <span id=\"followStrafeVal\">0.50</span></label>
                    <input id=\"followStrafe\" type=\"range\" min=\"0.0\" max=\"1.0\" step=\"0.05\" value=\"0.50\" />
                </div>
                <div>
                    <label for=\"followGyroDamp\">Gyro damping: <span id=\"followGyroDampVal\">0.15</span></label>
                    <input id=\"followGyroDamp\" type=\"range\" min=\"0.0\" max=\"0.50\" step=\"0.01\" value=\"0.15\" />
                </div>
            </div>
            <div class=\"row\" style=\"margin-top: 4px;\">
                <div class=\"kv\">Strafe moves the robot sideways toward the person (mecanum). Gyro damping uses the IMU to reduce angular oscillation.</div>
            </div>
        </div>

        <div class=\"card\">
            <h2>&#127752; Light Bar</h2>
            <p class=\"muted\">14 addressable WS2812 RGB LEDs. Publishes to <code>lightbar/command</code>.</p>
            <div class=\"row\">
                <div>
                    <label for=\"lbColor\">Colour:</label>
                    <input id=\"lbColor\" type=\"color\" value=\"#0066ff\" />
                </div>
                <button class=\"primary\" id=\"lbSolid\">Solid</button>
                <button id=\"lbRainbow\">Rainbow</button>
                <button id=\"lbBreathe\">Breathe</button>
                <button id=\"lbChase\">Chase</button>
                <button class=\"danger\" id=\"lbOff\">Off</button>
            </div>
            <div class=\"row\" style=\"margin-top: 8px;\">
                <div class=\"kv\">Mode: <span id=\"lbState\">off</span></div>
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

        <div class=\"card\" id=\"frontCamCard\">
            <h2>&#128247; Front Camera</h2>
            <p class=\"muted\">Pi Camera Module 3 (fixed, front-facing) — for navigation &amp; depth. Publishes on <code>front_camera/compressed</code>.</p>
            <div style=\"position:relative;width:100%;max-width:640px;\">
                <img id=\"frontVideo\" src=\"/stream_front.mjpg\" alt=\"front camera stream\" style=\"width:100%;height:auto;background:#111;border-radius:8px;display:block;\" />
            </div>
            <div class=\"row\" style=\"margin-top:6px;\">
                <div class=\"kv\">Front cam: <span id=\"frontCamStatus\">loading…</span></div>
            </div>
        </div>

        <div class=\"card\" id=\"depthCard\">
            <h2>&#127912; Depth Map</h2>
            <p class=\"muted\">Monocular depth estimation via Hailo-8 (fast_depth). Publishes on <code>depth/colorized/compressed</code>.</p>
            <div class=\"row\">
                <button class=\"primary\" id=\"depthToggleBtn\">Disable Depth</button>
                <label class=\"kv\"><input id=\"depthShowChk\" type=\"checkbox\" checked /> Show depth stream</label>
                <div class=\"kv\">Depth: <span id=\"depthStatus\">loading…</span></div>
            </div>
            <div id=\"depthStreamWrap\" style=\"position:relative;width:100%;max-width:640px;margin-top:8px;\">
                <img id=\"depthVideo\" src=\"/stream_depth.mjpg\" alt=\"depth map stream\" style=\"width:100%;height:auto;background:#111;border-radius:8px;display:block;\" />
            </div>
        </div>

        <div class=\"card\" id=\"imuCard\">
            <h2>&#129517; IMU &amp; Orientation</h2>
            <p class=\"muted\">Live 6-axis IMU data from Arduino Nano RP2040 Connect. Publishes on <code>imu/data</code>.</p>
            <div id=\"imuCalBanner\" style=\"background:#fff3cd;border:1px solid #ffc107;border-radius:6px;padding:8px 12px;margin-bottom:8px;display:none;\">
                &#9888;&#65039; IMU not calibrated — data may be unreliable. Hold robot still…
            </div>
            <div style=\"display:flex;gap:16px;flex-wrap:wrap;align-items:flex-start;\">
                <!-- 3D Robot orientation view -->
                <div style=\"flex:0 0 auto;text-align:center;\">
                    <div id=\"imuScene\" style=\"width:200px;height:200px;perspective:600px;margin:0 auto;\">
                        <div id=\"imuCube\" style=\"width:140px;height:80px;position:relative;transform-style:preserve-3d;margin:60px auto 0;\">
                            <!-- Front -->
                            <div style=\"position:absolute;width:140px;height:80px;background:rgba(51,51,51,0.92);border:2px solid #555;display:flex;align-items:center;justify-content:center;color:#0f0;font-size:11px;font-weight:bold;transform:translateZ(40px);\">FRONT</div>
                            <!-- Back -->
                            <div style=\"position:absolute;width:140px;height:80px;background:rgba(68,68,68,0.88);border:2px solid #555;display:flex;align-items:center;justify-content:center;color:#999;font-size:11px;transform:rotateY(180deg) translateZ(40px);\">BACK</div>
                            <!-- Left -->
                            <div style=\"position:absolute;width:80px;height:80px;background:rgba(60,60,60,0.88);border:2px solid #555;display:flex;align-items:center;justify-content:center;color:#f80;font-size:10px;transform:rotateY(-90deg) translateZ(70px);left:30px;\">L</div>
                            <!-- Right -->
                            <div style=\"position:absolute;width:80px;height:80px;background:rgba(60,60,60,0.88);border:2px solid #555;display:flex;align-items:center;justify-content:center;color:#08f;font-size:10px;transform:rotateY(90deg) translateZ(70px);left:30px;\">R</div>
                            <!-- Top -->
                            <div style=\"position:absolute;width:140px;height:80px;background:rgba(80,200,80,0.25);border:2px solid #4a4;display:flex;align-items:center;justify-content:center;color:#4a4;font-size:10px;transform:rotateX(90deg) translateZ(40px);\">TOP</div>
                            <!-- Bottom -->
                            <div style=\"position:absolute;width:140px;height:80px;background:rgba(180,50,50,0.25);border:2px solid #a44;transform:rotateX(-90deg) translateZ(40px);\"></div>
                            <!-- Wheels (decorative) -->
                            <div style=\"position:absolute;width:16px;height:16px;border-radius:50%;background:#222;border:2px solid #666;top:80px;left:6px;transform:translateZ(44px);\"></div>
                            <div style=\"position:absolute;width:16px;height:16px;border-radius:50%;background:#222;border:2px solid #666;top:80px;right:6px;transform:translateZ(44px);\"></div>
                            <div style=\"position:absolute;width:16px;height:16px;border-radius:50%;background:#222;border:2px solid #666;top:80px;left:6px;transform:translateZ(-44px);\"></div>
                            <div style=\"position:absolute;width:16px;height:16px;border-radius:50%;background:#222;border:2px solid #666;top:80px;right:6px;transform:translateZ(-44px);\"></div>
                        </div>
                    </div>
                    <div class=\"kv\" style=\"margin-top:4px;\">Yaw: <span id=\"imuYawLabel\">0.0</span>°</div>
                </div>

                <!-- Data readouts -->
                <div style=\"flex:1 1 260px;\">
                    <table style=\"width:100%;border-collapse:collapse;font-size:12px;font-family:ui-monospace,monospace;\">
                        <thead><tr style=\"border-bottom:1px solid #ddd;\"><th style=\"text-align:left;padding:2px 6px;\">Axis</th><th>Accel (m/s²)</th><th>Gyro (°/s)</th></tr></thead>
                        <tbody>
                            <tr><td style=\"padding:2px 6px;font-weight:bold;\">X</td><td id=\"imuAx\" style=\"text-align:center;\">—</td><td id=\"imuGx\" style=\"text-align:center;\">—</td></tr>
                            <tr><td style=\"padding:2px 6px;font-weight:bold;\">Y</td><td id=\"imuAy\" style=\"text-align:center;\">—</td><td id=\"imuGy\" style=\"text-align:center;\">—</td></tr>
                            <tr><td style=\"padding:2px 6px;font-weight:bold;\">Z</td><td id=\"imuAz\" style=\"text-align:center;\">—</td><td id=\"imuGz\" style=\"text-align:center;\">—</td></tr>
                        </tbody>
                    </table>
                    <div class=\"kv\" style=\"margin-top:6px;\">
                        Temp: <span id=\"imuTemp\">—</span> °C &nbsp;|&nbsp;
                        Mic: <span id=\"imuMic\">—</span> &nbsp;|&nbsp;
                        Cal: <span id=\"imuCal\">—</span>
                    </div>
                    <!-- Yaw top-down compass -->
                    <div style=\"margin-top:10px;\">
                        <canvas id=\"imuCompass\" width=\"180\" height=\"180\" style=\"display:block;margin:0 auto;\"></canvas>
                    </div>
                </div>
            </div>

            <!-- Rotate to angle control -->
            <div style=\"margin-top:12px;border-top:1px solid #eee;padding-top:10px;\">
                <h3 style=\"margin:0 0 6px 0;font-size:14px;\">Rotate to Angle</h3>
                <div class=\"row\">
                    <div>
                        <label for=\"imuTargetAngle\">Target: <span id=\"imuTargetVal\">0</span>°</label>
                        <input id=\"imuTargetAngle\" type=\"range\" min=\"-180\" max=\"180\" step=\"1\" value=\"0\" />
                    </div>
                    <div style=\"display:flex;gap:6px;align-items:center;\">
                        <input id=\"imuAngleInput\" type=\"number\" min=\"-180\" max=\"180\" step=\"1\" value=\"0\" style=\"width:70px;padding:6px;border:1px solid #ccc;border-radius:6px;\" />
                        <span>°</span>
                    </div>
                    <button class=\"primary\" id=\"imuRotateBtn\">Rotate</button>
                    <button class=\"danger\" id=\"imuRotateStop\">Stop</button>
                    <button id=\"imuResetYaw\">Reset Yaw</button>
                </div>
                <div class=\"row\" style=\"margin-top:6px;\">
                    <div>
                        <label for=\"imuRotateSpeed\">Speed: <span id=\"imuRotateSpeedVal\">0.5</span></label>
                        <input id=\"imuRotateSpeed\" type=\"range\" min=\"0.1\" max=\"1.0\" step=\"0.1\" value=\"0.5\" />
                    </div>
                    <div class=\"kv\">Rotation: <span id=\"imuRotateStatus\">idle</span></div>
                </div>
            </div>
        </div>

        <div class=\"card\" id=\"bnoCalCard\" style=\"display:none;\">
            <h2>&#129517; BNO055 Calibration</h2>
            <p class=\"muted\">Auto-calibrate the 9-DOF BNO055 sensor. The robot will spin slowly, tilt gently, then hold still. Place on a flat surface before starting.</p>
            <div style=\"display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin-bottom:10px;\">
                <button class=\"primary\" id=\"bnoCalStart\">&#9654; Auto-Calibrate</button>
                <button class=\"danger\" id=\"bnoCalStop\">&#9724; Cancel</button>
                <span id=\"bnoCalStateLabel\" style=\"font-size:13px;color:#666;\">idle</span>
            </div>
            <div id=\"bnoCalStatus\" style=\"background:#f0f4ff;border:1px solid #c0d0f0;border-radius:6px;padding:10px 12px;margin-bottom:10px;font-size:13px;display:none;\"></div>
            <div style=\"display:grid;grid-template-columns:80px 1fr 30px;gap:4px 8px;align-items:center;font-size:13px;font-family:ui-monospace,monospace;\">
                <span>System</span>
                <div style=\"background:#eee;border-radius:4px;height:18px;overflow:hidden;position:relative;\">
                    <div id=\"bnoBarSys\" style=\"background:#e74c3c;height:100%;width:0%;transition:width 0.3s;border-radius:4px;\"></div>
                </div>
                <span id=\"bnoValSys\">0/3</span>
                <span>Gyro</span>
                <div style=\"background:#eee;border-radius:4px;height:18px;overflow:hidden;position:relative;\">
                    <div id=\"bnoBarGyro\" style=\"background:#3498db;height:100%;width:0%;transition:width 0.3s;border-radius:4px;\"></div>
                </div>
                <span id=\"bnoValGyro\">0/3</span>
                <span>Accel</span>
                <div style=\"background:#eee;border-radius:4px;height:18px;overflow:hidden;position:relative;\">
                    <div id=\"bnoBarAccel\" style=\"background:#f39c12;height:100%;width:0%;transition:width 0.3s;border-radius:4px;\"></div>
                </div>
                <span id=\"bnoValAccel\">0/3</span>
                <span>Mag</span>
                <div style=\"background:#eee;border-radius:4px;height:18px;overflow:hidden;position:relative;\">
                    <div id=\"bnoBarMag\" style=\"background:#2ecc71;height:100%;width:0%;transition:width 0.3s;border-radius:4px;\"></div>
                </div>
                <span id=\"bnoValMag\">0/3</span>
            </div>
        </div>

        <div class=\"card\" id=\"micCard\">
            <h2>&#127908; Microphone</h2>
            <p class=\"muted\">Stream audio from Arduino Nano PDM mic (8 kHz mono). Uses Web Audio API for browser playback.</p>
            <div class=\"row\">
                <button class=\"primary\" id=\"micBtn\">Enable Mic</button>
                <div style=\"flex:2 1 300px;\">
                    <div style=\"background:#eee;border-radius:4px;height:20px;overflow:hidden;position:relative;\">
                        <div id=\"micVu\" style=\"background:linear-gradient(90deg,#4caf50,#ff9800,#f44336);height:100%;width:0%;transition:width 60ms;border-radius:4px;\"></div>
                    </div>
                </div>
                <div>
                    <label for=\"micVol\">Vol: <span id=\"micVolVal\">80</span>%</label>
                    <input id=\"micVol\" type=\"range\" min=\"0\" max=\"100\" step=\"5\" value=\"80\" style=\"width:100px;\" />
                </div>
            </div>
            <div class=\"row\" style=\"margin-top:6px;\">
                <div class=\"kv\">Status: <span id=\"micStatus\">off</span></div>
                <div class=\"kv\">Buffer: <span id=\"micBufInfo\">-</span></div>
            </div>
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
        const followStrafe = document.getElementById('followStrafe');
        const followStrafeVal = document.getElementById('followStrafeVal');
        const followGyroDamp = document.getElementById('followGyroDamp');
        const followGyroDampVal = document.getElementById('followGyroDampVal');

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
                    if (typeof j.follow.strafe_gain === 'number' && !userRecentlyTouched(followStrafe)) {
                        followStrafe.value = j.follow.strafe_gain.toFixed(2);
                        followStrafeVal.textContent = j.follow.strafe_gain.toFixed(2);
                    }
                    if (typeof j.follow.gyro_damping === 'number' && !userRecentlyTouched(followGyroDamp)) {
                        followGyroDamp.value = j.follow.gyro_damping.toFixed(2);
                        followGyroDampVal.textContent = j.follow.gyro_damping.toFixed(2);
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

        async function sendFollowConfig(targetArea, maxLinear, strafeGain, gyroDamping) {
            const body = {target_bbox_area: Number(targetArea), max_linear: Number(maxLinear)};
            if (strafeGain !== undefined) body.strafe_gain = Number(strafeGain);
            if (gyroDamping !== undefined) body.gyro_damping = Number(gyroDamping);
            try {
                await fetch('/api/follow/config', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify(body)
                });
            } catch (e) { /* ignore */ }
        }

        followDist.addEventListener('input', () => {
            markUserInput(followDist);
            followDistVal.textContent = Number(followDist.value).toFixed(2);
            sendFollowConfig(followDist.value, followSpeed.value, followStrafe.value, followGyroDamp.value);
        });
        followSpeed.addEventListener('input', () => {
            markUserInput(followSpeed);
            followSpeedVal.textContent = Number(followSpeed.value).toFixed(2);
            sendFollowConfig(followDist.value, followSpeed.value, followStrafe.value, followGyroDamp.value);
        });
        followStrafe.addEventListener('input', () => {
            markUserInput(followStrafe);
            followStrafeVal.textContent = Number(followStrafe.value).toFixed(2);
            sendFollowConfig(followDist.value, followSpeed.value, followStrafe.value, followGyroDamp.value);
        });
        followGyroDamp.addEventListener('input', () => {
            markUserInput(followGyroDamp);
            followGyroDampVal.textContent = Number(followGyroDamp.value).toFixed(2);
            sendFollowConfig(followDist.value, followSpeed.value, followStrafe.value, followGyroDamp.value);
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

        // ---- Lightbar ----
        const lbColor = document.getElementById('lbColor');
        const lbStateEl = document.getElementById('lbState');
        async function sendLightbar(cmd) {
            try {
                await fetch('/api/lightbar', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify(cmd)
                });
                lbStateEl.textContent = cmd.mode || 'off';
            } catch (e) { /* ignore */ }
        }
        function hexToRgb(hex) {
            const n = parseInt(hex.replace('#',''), 16);
            return {r: (n>>16)&255, g: (n>>8)&255, b: n&255};
        }
        document.getElementById('lbSolid').addEventListener('click', () => {
            const c = hexToRgb(lbColor.value);
            sendLightbar({mode:'solid', r:c.r, g:c.g, b:c.b});
        });
        document.getElementById('lbRainbow').addEventListener('click', () => sendLightbar({mode:'rainbow', speed:1.0}));
        document.getElementById('lbBreathe').addEventListener('click', () => {
            const c = hexToRgb(lbColor.value);
            sendLightbar({mode:'breathing', r:c.r, g:c.g, b:c.b, period:2.0});
        });
        document.getElementById('lbChase').addEventListener('click', () => {
            const c = hexToRgb(lbColor.value);
            sendLightbar({mode:'chase', r:c.r, g:c.g, b:c.b, speed:1.0});
        });
        document.getElementById('lbOff').addEventListener('click', () => sendLightbar({mode:'off'}));

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

        async function sendCmdVel(lin, lat, ang) {
            try {
                await fetch('/api/cmd_vel', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({linear_x: Number(lin), linear_y: Number(lat || 0), angular_z: Number(ang)})
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
            const latMag = fast ? 1.0 : 0.5;
            const angMag = fast ? 1.0 : 0.6;

            let lin = 0;
            let lat = 0;
            let ang = 0;
            if (pressed.has('w')) lin += linMag;
            if (pressed.has('s')) lin -= linMag;
            if (pressed.has('a')) lat += latMag;  // strafe left (ROS +Y = left)
            if (pressed.has('d')) lat -= latMag;  // strafe right
            if (pressed.has('q')) ang += angMag;  // rotate left
            if (pressed.has('e')) ang -= angMag;  // rotate right
            return {lin, lat, ang};
        }

        function startDriveLoop() {
            if (driveTimer) return;
            driveTimer = setInterval(async () => {
                const {lin, lat, ang} = computeDrive();
                lastLin = lin;
                lastAng = ang;
                cmdVelEl.textContent = `${lin.toFixed(2)}, ${lat.toFixed(2)}, ${ang.toFixed(2)}`;
                await sendCmdVel(lin, lat, ang);
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
            cmdVelEl.textContent = `0.00, 0.00, 0.00`;
            stopCmdVel();
        }

        function normalizeKey(ev) {
            if (ev.key === 'Shift') return 'shift';
            return String(ev.key || '').toLowerCase();
        }

        document.addEventListener('keydown', (ev) => {
            const k = normalizeKey(ev);
            if (!['w','a','s','d','q','e','shift'].includes(k)) return;
            // prevent browser from scrolling on space etc; WASD doesn't scroll but keep consistent.
            ev.preventDefault();
            pressed.add(k);
            startDriveLoop();
        }, { passive: false });

        document.addEventListener('keyup', (ev) => {
            const k = normalizeKey(ev);
            if (!['w','a','s','d','q','e','shift'].includes(k)) return;
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

        // ──────────────────────────────────────────────────────────────
        // IMU Dashboard
        // ──────────────────────────────────────────────────────────────
        const imuCube = document.getElementById('imuCube');
        const imuYawLabel = document.getElementById('imuYawLabel');
        const imuCalBanner = document.getElementById('imuCalBanner');
        const imuAx = document.getElementById('imuAx');
        const imuAy = document.getElementById('imuAy');
        const imuAz = document.getElementById('imuAz');
        const imuGx = document.getElementById('imuGx');
        const imuGy = document.getElementById('imuGy');
        const imuGz = document.getElementById('imuGz');
        const imuTemp = document.getElementById('imuTemp');
        const imuMic = document.getElementById('imuMic');
        const imuCal = document.getElementById('imuCal');
        const imuCompass = document.getElementById('imuCompass');
        const imuTargetAngle = document.getElementById('imuTargetAngle');
        const imuAngleInput = document.getElementById('imuAngleInput');
        const imuTargetVal = document.getElementById('imuTargetVal');
        const imuRotateBtn = document.getElementById('imuRotateBtn');
        const imuRotateStop = document.getElementById('imuRotateStop');
        const imuResetYaw = document.getElementById('imuResetYaw');
        const imuRotateSpeed = document.getElementById('imuRotateSpeed');
        const imuRotateSpeedVal = document.getElementById('imuRotateSpeedVal');
        const imuRotateStatus = document.getElementById('imuRotateStatus');

        let imuData = null;
        let imuRotating = false;

        imuTargetAngle.addEventListener('input', () => {
            imuTargetVal.textContent = imuTargetAngle.value;
            imuAngleInput.value = imuTargetAngle.value;
        });
        imuAngleInput.addEventListener('input', () => {
            let v = clamp(Number(imuAngleInput.value), -180, 180);
            imuTargetAngle.value = String(Math.round(v));
            imuTargetVal.textContent = String(Math.round(v));
        });
        imuRotateSpeed.addEventListener('input', () => {
            imuRotateSpeedVal.textContent = Number(imuRotateSpeed.value).toFixed(1);
        });

        async function fetchImu() {
            try {
                const r = await fetch('/api/imu', {cache: 'no-store'});
                if (!r.ok) return;
                imuData = await r.json();

                const d = imuData;
                const f2 = (v) => v != null ? Number(v).toFixed(2) : '—';
                const f1 = (v) => v != null ? Number(v).toFixed(1) : '—';

                imuAx.textContent = f2(d.accel_x);
                imuAy.textContent = f2(d.accel_y);
                imuAz.textContent = f2(d.accel_z);
                imuGx.textContent = f2(d.gyro_x);
                imuGy.textContent = f2(d.gyro_y);
                imuGz.textContent = f2(d.gyro_z);
                imuTemp.textContent = f1(d.temperature);
                imuMic.textContent = d.mic_level != null ? String(d.mic_level) : '—';

                const cal = Boolean(d.calibrated);
                imuCal.textContent = cal ? '✅ OK' : '⏳ pending';
                imuCal.style.color = cal ? '#080' : '#b00';
                imuCalBanner.style.display = cal ? 'none' : 'block';

                // Update yaw label
                const yaw = d.yaw_deg != null ? Number(d.yaw_deg) : 0;
                imuYawLabel.textContent = yaw.toFixed(1);

                // Update 3D cube orientation
                updateCube(d);
                // Update compass
                drawCompass(yaw, d.rotate_target_deg);
                // Update BNO055 calibration card
                updateBnoCal(d);
                // Update rotate status
                if (d.rotate_active) {
                    imuRotating = true;
                    imuRotateStatus.textContent = 'rotating to ' + (d.rotate_target_deg != null ? d.rotate_target_deg.toFixed(0) + '°' : '?');
                    imuRotateStatus.style.color = '#05a';
                } else {
                    if (imuRotating) {
                        imuRotateStatus.textContent = 'done';
                        imuRotateStatus.style.color = '#080';
                        imuRotating = false;
                    }
                }
            } catch (e) {
                // ignore
            }
        }

        function updateCube(d) {
            if (!d) return;
            // Server computes pitch/roll from BNO055 quaternion when
            // available, falling back to accel-trig for LSM6DSOX.
            const pitch = d.pitch_deg || 0;
            const roll  = d.roll_deg  || 0;
            const yaw   = d.yaw_deg   || 0;

            // CSS transform: rotations applied in order
            imuCube.style.transform = 'rotateX(' + (-pitch).toFixed(1) + 'deg) rotateZ(' + (roll).toFixed(1) + 'deg) rotateY(' + (-yaw).toFixed(1) + 'deg)';
        }

        function drawCompass(yaw, target) {
            const ctx = imuCompass.getContext('2d');
            if (!ctx) return;
            const W = imuCompass.width;
            const H = imuCompass.height;
            const cx = W / 2;
            const cy = H / 2;
            const R = Math.min(cx, cy) - 10;

            ctx.clearRect(0, 0, W, H);

            // Outer circle
            ctx.beginPath();
            ctx.arc(cx, cy, R, 0, 2 * Math.PI);
            ctx.strokeStyle = '#ccc';
            ctx.lineWidth = 2;
            ctx.stroke();

            // Cardinal ticks & labels
            ctx.font = '11px ui-monospace, monospace';
            ctx.textAlign = 'center';
            ctx.textBaseline = 'middle';
            ctx.fillStyle = '#666';
            const cardinals = [{a:0, l:'0°'}, {a:90, l:'90°'}, {a:180, l:'±180°'}, {a:-90, l:'-90°'}];
            for (const c of cardinals) {
                const rad = (c.a - 90) * Math.PI / 180;
                const tx = cx + Math.cos(rad) * (R + 0);
                const ty = cy + Math.sin(rad) * (R + 0);
                // Tick
                ctx.beginPath();
                ctx.moveTo(cx + Math.cos(rad) * (R - 6), cy + Math.sin(rad) * (R - 6));
                ctx.lineTo(cx + Math.cos(rad) * R, cy + Math.sin(rad) * R);
                ctx.strokeStyle = '#999';
                ctx.lineWidth = 2;
                ctx.stroke();
            }
            // Labels outside
            ctx.fillStyle = '#666';
            ctx.fillText('0°', cx, cy - R - 0 + 14);
            ctx.fillText('±180°', cx, cy + R + 0 - 12);
            ctx.fillText('90°', cx + R - 16, cy);
            ctx.fillText('-90°', cx - R + 18, cy);

            // Minor ticks every 30°
            for (let a = 0; a < 360; a += 30) {
                const rad = (a - 90) * Math.PI / 180;
                ctx.beginPath();
                ctx.moveTo(cx + Math.cos(rad) * (R - 3), cy + Math.sin(rad) * (R - 3));
                ctx.lineTo(cx + Math.cos(rad) * R, cy + Math.sin(rad) * R);
                ctx.strokeStyle = '#ddd';
                ctx.lineWidth = 1;
                ctx.stroke();
            }

            // Target angle indicator (if set and rotating)
            if (target != null) {
                const tRad = (target - 90) * Math.PI / 180;
                ctx.beginPath();
                ctx.moveTo(cx, cy);
                ctx.lineTo(cx + Math.cos(tRad) * (R - 8), cy + Math.sin(tRad) * (R - 8));
                ctx.strokeStyle = 'rgba(0, 100, 255, 0.4)';
                ctx.lineWidth = 6;
                ctx.lineCap = 'round';
                ctx.stroke();
                ctx.lineCap = 'butt';
                // Target dot
                ctx.beginPath();
                ctx.arc(cx + Math.cos(tRad) * (R - 8), cy + Math.sin(tRad) * (R - 8), 5, 0, 2 * Math.PI);
                ctx.fillStyle = 'rgba(0, 100, 255, 0.6)';
                ctx.fill();
            }

            // Current yaw arrow
            const yRad = (yaw - 90) * Math.PI / 180;
            ctx.beginPath();
            ctx.moveTo(cx, cy);
            ctx.lineTo(cx + Math.cos(yRad) * (R - 12), cy + Math.sin(yRad) * (R - 12));
            ctx.strokeStyle = '#e00';
            ctx.lineWidth = 3;
            ctx.lineCap = 'round';
            ctx.stroke();
            ctx.lineCap = 'butt';

            // Arrow head
            const headLen = 10;
            const headAngle = 0.4;
            const tipX = cx + Math.cos(yRad) * (R - 12);
            const tipY = cy + Math.sin(yRad) * (R - 12);
            ctx.beginPath();
            ctx.moveTo(tipX, tipY);
            ctx.lineTo(tipX - headLen * Math.cos(yRad - headAngle), tipY - headLen * Math.sin(yRad - headAngle));
            ctx.moveTo(tipX, tipY);
            ctx.lineTo(tipX - headLen * Math.cos(yRad + headAngle), tipY - headLen * Math.sin(yRad + headAngle));
            ctx.strokeStyle = '#e00';
            ctx.lineWidth = 2.5;
            ctx.stroke();

            // Center dot
            ctx.beginPath();
            ctx.arc(cx, cy, 4, 0, 2 * Math.PI);
            ctx.fillStyle = '#333';
            ctx.fill();

            // Yaw text
            ctx.font = 'bold 13px ui-monospace, monospace';
            ctx.textAlign = 'center';
            ctx.fillStyle = '#333';
            ctx.fillText(yaw.toFixed(1) + '°', cx, cy + R + 0 - 25);
        }

        // Rotate to angle
        imuRotateBtn.addEventListener('click', async () => {
            const angle = Number(imuAngleInput.value);
            const speed = Number(imuRotateSpeed.value);
            if (Number.isNaN(angle) || Number.isNaN(speed)) return;
            imuRotateStatus.textContent = 'starting…';
            imuRotateStatus.style.color = '#05a';
            try {
                await fetch('/api/imu/rotate', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({target_deg: angle, speed: speed})
                });
            } catch (e) {
                imuRotateStatus.textContent = 'error';
                imuRotateStatus.style.color = '#b00';
            }
        });

        imuRotateStop.addEventListener('click', async () => {
            try {
                await fetch('/api/imu/rotate/stop', {method: 'POST'});
                imuRotateStatus.textContent = 'stopped';
                imuRotateStatus.style.color = '#b00';
            } catch (e) { /* ignore */ }
        });

        imuResetYaw.addEventListener('click', async () => {
            try {
                await fetch('/api/imu/calibrate', {method: 'POST'});
                imuRotateStatus.textContent = 'recalibrating…';
                imuRotateStatus.style.color = '#a80';
            } catch (e) { /* ignore */ }
        });

        // ── BNO055 Calibration Card ────────────────────────────────
        const bnoCalCard = document.getElementById('bnoCalCard');
        const bnoCalStart = document.getElementById('bnoCalStart');
        const bnoCalStop = document.getElementById('bnoCalStop');
        const bnoCalStateLabel = document.getElementById('bnoCalStateLabel');
        const bnoCalStatus = document.getElementById('bnoCalStatus');
        const bnoBarSys = document.getElementById('bnoBarSys');
        const bnoBarGyro = document.getElementById('bnoBarGyro');
        const bnoBarAccel = document.getElementById('bnoBarAccel');
        const bnoBarMag = document.getElementById('bnoBarMag');
        const bnoValSys = document.getElementById('bnoValSys');
        const bnoValGyro = document.getElementById('bnoValGyro');
        const bnoValAccel = document.getElementById('bnoValAccel');
        const bnoValMag = document.getElementById('bnoValMag');

        function updateBnoCal(d) {
            if (!d) return;
            // Show/hide card based on BNO055 availability
            if (d.bno_available) {
                bnoCalCard.style.display = '';
            } else {
                bnoCalCard.style.display = 'none';
                return;
            }
            const cal = d.bno_cal || {};
            const ac = d.autocal || {};

            // Update bars
            const colors3 = ['#e74c3c', '#f39c12', '#2ecc71'];
            function setBar(bar, valEl, v, name) {
                const pct = Math.min(100, (v / 3) * 100);
                bar.style.width = pct + '%';
                bar.style.background = v >= 3 ? '#2ecc71' : v >= 2 ? '#f39c12' : v >= 1 ? '#3498db' : '#e74c3c';
                valEl.textContent = v + '/3';
            }
            setBar(bnoBarSys, bnoValSys, cal.sys || 0, 'sys');
            setBar(bnoBarGyro, bnoValGyro, cal.gyro || 0, 'gyro');
            setBar(bnoBarAccel, bnoValAccel, cal.accel || 0, 'accel');
            setBar(bnoBarMag, bnoValMag, cal.mag || 0, 'mag');

            // State label
            const stateMap = {
                'idle': '⚪ Idle',
                'spin_mag': '🔄 Spinning (magnetometer)',
                'tilt_accel': '↕️ Tilting (accelerometer)',
                'still_gyro': '⏸️ Holding still (gyroscope)',
                'wait_sys': '⏳ Waiting for fusion',
                'saving': '💾 Saving…',
                'done': '✅ Done',
                'failed': '❌ Failed',
            };
            bnoCalStateLabel.textContent = stateMap[ac.state] || ac.state || 'idle';
            const stateColor = {
                'idle': '#666', 'spin_mag': '#05a', 'tilt_accel': '#05a',
                'still_gyro': '#05a', 'wait_sys': '#a80', 'saving': '#a80',
                'done': '#080', 'failed': '#b00',
            };
            bnoCalStateLabel.style.color = stateColor[ac.state] || '#666';

            // Status message
            if (ac.message) {
                bnoCalStatus.style.display = 'block';
                bnoCalStatus.textContent = ac.message;
            } else {
                bnoCalStatus.style.display = 'none';
            }

            // Button states
            const running = ac.state && ac.state !== 'idle' && ac.state !== 'done' && ac.state !== 'failed';
            bnoCalStart.disabled = running;
            bnoCalStop.disabled = !running;
        }

        bnoCalStart.addEventListener('click', async () => {
            try {
                const r = await fetch('/api/bno/autocal/start', {method: 'POST'});
                const data = await r.json();
                if (!data.ok) {
                    bnoCalStatus.style.display = 'block';
                    bnoCalStatus.textContent = '⚠️ ' + (data.error || 'Failed to start');
                }
            } catch (e) {
                bnoCalStatus.style.display = 'block';
                bnoCalStatus.textContent = '⚠️ Request failed';
            }
        });

        bnoCalStop.addEventListener('click', async () => {
            try {
                await fetch('/api/bno/autocal/stop', {method: 'POST'});
            } catch (e) { /* ignore */ }
        });

        // Poll IMU at ~10 Hz
        fetchImu();
        setInterval(fetchImu, 100);

        // Front camera status
        const frontCamStatus = document.getElementById('frontCamStatus');
        const frontVideo = document.getElementById('frontVideo');
        const ultrasonicDistText = document.getElementById('ultrasonicDistText');
        const ultrasonicOverlay = document.getElementById('ultrasonicOverlay');
        const ultrasonicWarnIcon = document.getElementById('ultrasonicWarnIcon');
        let frontCamOk = false;
        frontVideo.addEventListener('load', () => { frontCamOk = true; frontCamStatus.textContent = 'streaming'; });
        frontVideo.addEventListener('error', () => { frontCamOk = false; frontCamStatus.textContent = 'no stream'; });
        setInterval(() => {
            if (frontCamOk) {
                frontCamStatus.textContent = 'streaming';
                frontCamStatus.style.color = '#080';
            } else {
                frontCamStatus.textContent = 'no stream';
                frontCamStatus.style.color = '#b00';
            }
        }, 2000);

        // Update ultrasonic overlay from collision failsafe status
        function updateUltrasonicOverlay(cf) {
            if (!cf || !ultrasonicOverlay) return;
            const dist = cf.distance_m;
            const active = cf.active;
            const enabled = cf.enabled;
            const stopD = 0.15, slowD = 0.35;

            if (dist !== null && dist !== undefined && isFinite(dist)) {
                ultrasonicDistText.textContent = dist.toFixed(2) + ' m';
                if (active || dist <= stopD) {
                    // Red — hard stop zone
                    ultrasonicOverlay.style.background = 'rgba(180,0,0,0.85)';
                    ultrasonicDistText.style.color = '#fff';
                    ultrasonicWarnIcon.style.display = '';
                } else if (dist < slowD) {
                    // Orange — slow-down zone
                    ultrasonicOverlay.style.background = 'rgba(200,120,0,0.85)';
                    ultrasonicDistText.style.color = '#fff';
                    ultrasonicWarnIcon.style.display = '';
                } else {
                    // Normal — clear
                    ultrasonicOverlay.style.background = 'rgba(0,0,0,0.7)';
                    ultrasonicDistText.style.color = '#fff';
                    ultrasonicWarnIcon.style.display = 'none';
                }
            } else {
                ultrasonicDistText.textContent = '— m';
                ultrasonicOverlay.style.background = 'rgba(0,0,0,0.7)';
                ultrasonicDistText.style.color = '#888';
                ultrasonicWarnIcon.style.display = 'none';
            }
            if (!enabled) {
                ultrasonicOverlay.style.opacity = '0.5';
            } else {
                ultrasonicOverlay.style.opacity = '1';
            }
        }

        // ──────────────────────────────────────────────────────────────
        // Microphone Streaming (Arduino PDM → 8 kHz unsigned 8-bit PCM)
        // ──────────────────────────────────────────────────────────────
        const micBtn = document.getElementById('micBtn');
        const micVu = document.getElementById('micVu');
        const micVol = document.getElementById('micVol');
        const micVolVal = document.getElementById('micVolVal');
        const micStatusEl = document.getElementById('micStatus');
        const micBufInfo = document.getElementById('micBufInfo');

        let micEnabled = false;
        let micAudioCtx = null;
        let micGainNode = null;
        let micProcessor = null;
        let micReader = null;
        const MIC_RING_SIZE = 32768;
        let micRingBuf = new Float32Array(MIC_RING_SIZE);
        let micRingWr = 0;
        let micRingRd = 0;

        micVol.addEventListener('input', () => {
            micVolVal.textContent = micVol.value;
            if (micGainNode) micGainNode.gain.value = Number(micVol.value) / 100;
        });

        micBtn.addEventListener('click', () => {
            if (micEnabled) { disableMic(); } else { enableMic(); }
        });

        async function enableMic() {
            try {
                await fetch('/api/audio/enable', { method: 'POST' });

                micAudioCtx = new (window.AudioContext || window.webkitAudioContext)({ sampleRate: 8000 });
                micGainNode = micAudioCtx.createGain();
                micGainNode.gain.value = Number(micVol.value) / 100;
                micGainNode.connect(micAudioCtx.destination);

                micProcessor = micAudioCtx.createScriptProcessor(2048, 0, 1);
                micProcessor.onaudioprocess = function(e) {
                    const out = e.outputBuffer.getChannelData(0);
                    let peak = 0;
                    for (let i = 0; i < out.length; i++) {
                        if (micRingRd !== micRingWr) {
                            out[i] = micRingBuf[micRingRd];
                            const a = Math.abs(out[i]);
                            if (a > peak) peak = a;
                            micRingRd = (micRingRd + 1) & (MIC_RING_SIZE - 1);
                        } else {
                            out[i] = 0;
                        }
                    }
                    micVu.style.width = Math.min(100, peak * 250).toFixed(0) + '%';
                    const avail = (micRingWr >= micRingRd)
                                  ? (micRingWr - micRingRd)
                                  : (MIC_RING_SIZE - micRingRd + micRingWr);
                    micBufInfo.textContent = avail + ' samples';
                };
                micProcessor.connect(micGainNode);

                micEnabled = true;
                micBtn.textContent = 'Disable Mic';
                micBtn.classList.remove('primary');
                micBtn.classList.add('danger');
                micStatusEl.textContent = 'connecting...';
                micStatusEl.style.color = '#a80';

                const resp = await fetch('/stream_audio');
                micReader = resp.body.getReader();
                micStatusEl.textContent = 'streaming';
                micStatusEl.style.color = '#080';

                while (micEnabled) {
                    const { done, value } = await micReader.read();
                    if (done) break;
                    for (let i = 0; i < value.length; i++) {
                        micRingBuf[micRingWr] = (value[i] - 128) / 128.0;
                        micRingWr = (micRingWr + 1) & (MIC_RING_SIZE - 1);
                    }
                }
            } catch (e) {
                micStatusEl.textContent = 'error';
                micStatusEl.style.color = '#b00';
                console.error('Mic error:', e);
            }
        }

        async function disableMic() {
            micEnabled = false;
            try { await fetch('/api/audio/disable', { method: 'POST' }); } catch(e) {}
            if (micReader) { try { micReader.cancel(); } catch(e) {} micReader = null; }
            if (micProcessor) { micProcessor.disconnect(); micProcessor = null; }
            if (micGainNode) { micGainNode.disconnect(); micGainNode = null; }
            if (micAudioCtx) { micAudioCtx.close(); micAudioCtx = null; }
            micRingWr = 0;
            micRingRd = 0;
            micVu.style.width = '0%';
            micBtn.textContent = 'Enable Mic';
            micBtn.classList.remove('danger');
            micBtn.classList.add('primary');
            micStatusEl.textContent = 'off';
            micStatusEl.style.color = '';
            micBufInfo.textContent = '-';
        }

        // ──────────────────────────────────────────────────────────────
        // Collision Failsafe
        // ──────────────────────────────────────────────────────────────
        const collisionToggleBtn = document.getElementById('collisionToggleBtn');
        const collisionStatus = document.getElementById('collisionStatus');
        const collisionDist = document.getElementById('collisionDist');
        let collisionEnabled = true;

        collisionToggleBtn.addEventListener('click', async () => {
            if (collisionEnabled) {
                try { await fetch('/api/collision_failsafe/disable', { method: 'POST' }); } catch(e) {}
                collisionEnabled = false;
                collisionToggleBtn.textContent = 'Enable Failsafe';
                collisionToggleBtn.classList.remove('danger');
                collisionToggleBtn.classList.add('primary');
                collisionStatus.textContent = 'disabled';
                collisionStatus.style.color = '#666';
            } else {
                try { await fetch('/api/collision_failsafe/enable', { method: 'POST' }); } catch(e) {}
                collisionEnabled = true;
                collisionToggleBtn.textContent = 'Disable Failsafe';
                collisionToggleBtn.classList.remove('primary');
                collisionToggleBtn.classList.add('danger');
                collisionStatus.textContent = 'enabled';
                collisionStatus.style.color = '#080';
            }
        });

        // ──────────────────────────────────────────────────────────────
        // Cliff Failsafe
        // ──────────────────────────────────────────────────────────────
        const cliffToggleBtn = document.getElementById('cliffToggleBtn');
        const cliffStatus = document.getElementById('cliffStatus');
        const cliffSensors = document.getElementById('cliffSensors');
        let cliffEnabled = true;

        cliffToggleBtn.addEventListener('click', async () => {
            if (cliffEnabled) {
                try { await fetch('/api/cliff_failsafe/disable', { method: 'POST' }); } catch(e) {}
                cliffEnabled = false;
                cliffToggleBtn.textContent = 'Enable Failsafe';
                cliffToggleBtn.classList.remove('danger');
                cliffToggleBtn.classList.add('primary');
                cliffStatus.textContent = 'disabled';
                cliffStatus.style.color = '#666';
            } else {
                try { await fetch('/api/cliff_failsafe/enable', { method: 'POST' }); } catch(e) {}
                cliffEnabled = true;
                cliffToggleBtn.textContent = 'Disable Failsafe';
                cliffToggleBtn.classList.remove('primary');
                cliffToggleBtn.classList.add('danger');
                cliffStatus.textContent = 'enabled';
                cliffStatus.style.color = '#080';
            }
        });

        // Poll collision status from /status endpoint
        setInterval(async () => {
            try {
                const r = await fetch('/status');
                const d = await r.json();
                if (d.collision_failsafe) {
                    const cf = d.collision_failsafe;
                    const dist_m = cf.distance_m;
                    if (dist_m !== null && dist_m !== undefined && isFinite(dist_m)) {
                        collisionDist.textContent = dist_m.toFixed(2) + ' m';
                    } else {
                        collisionDist.textContent = '—';
                    }
                    if (cf.active) {
                        collisionStatus.textContent = 'BLOCKING';
                        collisionStatus.style.color = '#b00';
                        collisionDist.style.color = '#b00';
                    } else if (cf.enabled) {
                        collisionStatus.textContent = 'clear';
                        collisionStatus.style.color = '#080';
                        collisionDist.style.color = '';
                    } else {
                        collisionStatus.textContent = 'disabled';
                        collisionStatus.style.color = '#666';
                        collisionDist.style.color = '#666';
                    }
                    // Sync toggle button state
                    if (cf.enabled !== collisionEnabled) {
                        collisionEnabled = cf.enabled;
                        if (collisionEnabled) {
                            collisionToggleBtn.textContent = 'Disable Failsafe';
                            collisionToggleBtn.classList.remove('primary');
                            collisionToggleBtn.classList.add('danger');
                        } else {
                            collisionToggleBtn.textContent = 'Enable Failsafe';
                            collisionToggleBtn.classList.remove('danger');
                            collisionToggleBtn.classList.add('primary');
                        }
                    }
                    // Update front-camera ultrasonic overlay
                    updateUltrasonicOverlay(cf);
                }

                // ── Cliff failsafe status ──────────────────────────
                if (d.cliff_failsafe) {
                    const cl = d.cliff_failsafe;
                    const state = cl.sensor_state || 0;
                    // Render sensor boxes: filled = floor present, empty = no floor/edge
                    const labels = ['L1','L2','R1','R2'];
                    let sensorStr = '';
                    for (let i = 0; i < 4; i++) {
                        const hasFloor = (state >> i) & 1;
                        sensorStr += (hasFloor ? '\u25A0' : '\u25A1') + labels[i] + ' ';
                    }
                    cliffSensors.textContent = sensorStr.trim();

                    if (cl.active) {
                        cliffStatus.textContent = 'BLOCKING';
                        cliffStatus.style.color = '#b00';
                        cliffSensors.style.color = '#b00';
                    } else if (cl.enabled) {
                        cliffStatus.textContent = 'clear';
                        cliffStatus.style.color = '#080';
                        cliffSensors.style.color = '';
                    } else {
                        cliffStatus.textContent = 'disabled';
                        cliffStatus.style.color = '#666';
                        cliffSensors.style.color = '#666';
                    }
                    // Sync toggle button
                    if (cl.enabled !== cliffEnabled) {
                        cliffEnabled = cl.enabled;
                        if (cliffEnabled) {
                            cliffToggleBtn.textContent = 'Disable Failsafe';
                            cliffToggleBtn.classList.remove('primary');
                            cliffToggleBtn.classList.add('danger');
                        } else {
                            cliffToggleBtn.textContent = 'Enable Failsafe';
                            cliffToggleBtn.classList.remove('danger');
                            cliffToggleBtn.classList.add('primary');
                        }
                    }
                }
            } catch(e) {}
        }, 500);

        // ──────────────────────────────────────────────────────────────
        // Depth Map (Hailo-8 fast_depth)
        // ──────────────────────────────────────────────────────────────
        const depthToggleBtn = document.getElementById('depthToggleBtn');
        const depthShowChk = document.getElementById('depthShowChk');
        const depthStatus = document.getElementById('depthStatus');
        const depthStreamWrap = document.getElementById('depthStreamWrap');
        const depthVideo = document.getElementById('depthVideo');
        let depthEnabled = true;
        let depthStreamOk = false;

        depthVideo.addEventListener('load', () => { depthStreamOk = true; });
        depthVideo.addEventListener('error', () => { depthStreamOk = false; });
        setInterval(() => {
            if (depthStreamOk && depthEnabled) {
                depthStatus.textContent = 'streaming';
                depthStatus.style.color = '#080';
            } else if (!depthEnabled) {
                depthStatus.textContent = 'disabled';
                depthStatus.style.color = '#666';
            } else {
                depthStatus.textContent = 'no stream';
                depthStatus.style.color = '#b00';
            }
        }, 2000);

        depthToggleBtn.addEventListener('click', async () => {
            if (depthEnabled) {
                try { await fetch('/api/depth/disable', { method: 'POST' }); } catch(e) {}
                depthEnabled = false;
                depthVideo.src = '';
                depthToggleBtn.textContent = 'Enable Depth';
                depthToggleBtn.classList.remove('danger');
                depthToggleBtn.classList.add('primary');
            } else {
                try { await fetch('/api/depth/enable', { method: 'POST' }); } catch(e) {}
                depthEnabled = true;
                depthVideo.src = '/stream_depth.mjpg';
                depthToggleBtn.textContent = 'Disable Depth';
                depthToggleBtn.classList.remove('primary');
                depthToggleBtn.classList.add('danger');
            }
        });
        // Start with button showing 'Disable' since depth starts enabled
        depthToggleBtn.classList.remove('primary');
        depthToggleBtn.classList.add('danger');

        depthShowChk.addEventListener('change', () => {
            depthStreamWrap.style.display = depthShowChk.checked ? '' : 'none';
        });
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
        self.declare_parameter("rotate_kp", 1.8)       # proportional gain
        self.declare_parameter("rotate_kd", 0.3)       # derivative gain (damping via gyro yaw rate)
        self.declare_parameter("rotate_tolerance_deg", 2.0)
        self.declare_parameter("rotate_settle_sec", 0.5)

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
        self._rotate_kd = float(self.get_parameter("rotate_kd").value)
        self._rotate_tolerance = float(self.get_parameter("rotate_tolerance_deg").value)
        self._rotate_settle_sec = float(self.get_parameter("rotate_settle_sec").value)
        self._rotate_in_tolerance_since = None

        self._imu_sub = self.create_subscription(Imu, imu_data_topic, self._on_imu_data, 10)
        self._imu_yaw_sub = self.create_subscription(Float64, imu_yaw_topic, self._on_imu_yaw, 10)
        self._imu_cal_sub = self.create_subscription(Bool, imu_cal_topic, self._on_imu_cal, 10)
        self._imu_temp_sub = self.create_subscription(Float32, imu_temp_topic, self._on_imu_temp, 10)
        self._imu_mic_sub = self.create_subscription(Int32, imu_mic_topic, self._on_imu_mic, 10)
        self._imu_calibrate_pub = self.create_publisher(Empty, imu_calibrate_topic, 10)

        # Rotate controller timer (20 Hz)
        self._rotate_timer = self.create_timer(0.05, self._rotate_tick)

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
            self._imu_pitch_deg = math.degrees(
                math.atan2(ax_eq, math.sqrt(denom_p))
            ) if denom_p > 1e-6 else 0.0
            self._imu_roll_deg = math.degrees(
                math.atan2(-ay_eq, az_eq)
            ) if (abs(az_eq) > 1e-6 or abs(ay_eq) > 1e-6) else 0.0
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

    # ── Rotate-to-angle controller ────────────────────────────────────

    def start_rotate(self, target_deg: float, speed: float = 0.5) -> None:
        target_deg = max(-180.0, min(180.0, float(target_deg)))
        speed = max(0.1, min(1.0, float(speed)))
        self._rotate_target_deg = target_deg
        self._rotate_speed = speed
        self._rotate_active = True
        self._rotate_in_tolerance_since = None
        self.get_logger().info(f"Rotate to {target_deg:.1f}° at speed {speed:.1f}")

    def stop_rotate(self) -> None:
        self._rotate_active = False
        self._rotate_target_deg = None
        self._rotate_in_tolerance_since = None
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

        # Check if within tolerance
        if abs(error) < self._rotate_tolerance:
            if self._rotate_in_tolerance_since is None:
                self._rotate_in_tolerance_since = now
            elif (now - self._rotate_in_tolerance_since) >= self._rotate_settle_sec:
                # Done!
                self.get_logger().info(
                    f"Rotation complete: target={target:.1f}°, current={current:.1f}°, error={error:.1f}°"
                )
                self._rotate_active = False
                msg = Twist()
                self._cmd_pub.publish(msg)
                return
        else:
            self._rotate_in_tolerance_since = None

        # PD controller: use gyro yaw-rate (°/s) as derivative for damping
        yaw_rate = self._imu_gyro[2]  # °/s from IMU (already converted)

        p_term = self._rotate_kp * error           # proportional
        d_term = -self._rotate_kd * yaw_rate        # derivative (damping)
        raw_cmd = p_term + d_term

        # Normalize to ±1 range and clamp to requested speed
        angular_unit = raw_cmd / 180.0
        angular_unit = max(-self._rotate_speed, min(self._rotate_speed, angular_unit))

        # Gentle minimum nudge only when far from target and nearly stalled
        min_unit = 0.08
        if abs(error) > self._rotate_tolerance * 2.0 and abs(angular_unit) < min_unit:
            angular_unit = min_unit if error > 0 else -min_unit

        # Dead-zone: if error is small and output is tiny, just stop
        if abs(error) < self._rotate_tolerance * 1.5 and abs(angular_unit) < min_unit:
            angular_unit = 0.0

        msg = Twist()
        msg.angular.z = float(angular_unit) * float(self._max_angular_rps)
        self._cmd_pub.publish(msg)
        self._cmd_last_rx_monotonic = time.monotonic()
        self._cmd_last_sent_monotonic = self._cmd_last_rx_monotonic


def make_handler(
    *,
    frame_buffer: FrameBuffer,
    front_frame_buffer: FrameBuffer = None,
    depth_frame_buffer: FrameBuffer = None,
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
):
    boundary = b"--frame"
    fps = max(float(fps_limit), 1.0)
    min_period = 1.0 / fps

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
                try:
                    self.wfile.flush()
                except Exception:
                    pass

                last_stamp = 0.0
                last_sent_time = 0.0

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
                try:
                    self.wfile.flush()
                except Exception:
                    pass

                last_stamp = 0.0
                last_sent_time = 0.0

                try:
                    while True:
                        now = time.monotonic()
                        sleep_needed = (last_sent_time + min_period) - now
                        if sleep_needed > 0:
                            time.sleep(sleep_needed)

                        frame = depth_frame_buffer.wait_for_newer(last_stamp, timeout=1.0)
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
                try:
                    self.wfile.flush()
                except Exception:
                    pass

                last_stamp = 0.0
                last_sent_time = 0.0

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

                        resized = _resize_jpeg(frame.jpeg, lo_w, lo_h, lo_q)
                        if resized is None:
                            # Fallback: send original frame
                            resized = frame.jpeg

                        headers = (
                            boundary + b"\r\n"
                            b"Content-Type: image/jpeg\r\n" +
                            f"Content-Length: {len(resized)}\r\n\r\n".encode("ascii")
                        )
                        self.wfile.write(headers)
                        self.wfile.write(resized)
                        self.wfile.write(b"\r\n")
                        try:
                            self.wfile.flush()
                        except Exception:
                            pass
                except (BrokenPipeError, ConnectionResetError):
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
                    while True:
                        seq, data = audio_buffer.get_newer(last_seq, timeout=2.0)
                        if not data:
                            continue
                        last_seq = seq
                        self.wfile.write(data)
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    return
                except Exception as e:
                    if logger is not None:
                        logger.warn(f"audio stream client error: {e!r}")
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

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    handler_cls = make_handler(
        frame_buffer=frame_buffer,
        front_frame_buffer=front_frame_buffer,
        depth_frame_buffer=depth_frame_buffer,
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
    )
    httpd = ThreadingHTTPServer((bind, port), handler_cls)
    # Allow the serve loop to wake up periodically so Ctrl-C / rclpy shutdown is responsive.
    httpd.timeout = 0.5

    node.get_logger().info(
        f"Web video server on http://{bind}:{port}/ (MJPEG: /stream.mjpg, front: /stream_front.mjpg, depth: /stream_depth.mjpg, status: /status)"
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
