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
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import CompressedImage, Imu, LaserScan, Range
from std_msgs.msg import Bool, Empty, Float32, Float64, Int32, Int32MultiArray, String, UInt8MultiArray
from std_srvs.srv import Trigger


INDEX_HTML = """<!doctype html>
<html lang=\"en\">
<head>
  <meta charset=\"utf-8\" />
  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\" />
  <title>Raspbot Camera</title>
  <style>
    *, *::before, *::after { box-sizing: border-box; }
    body { font-family: system-ui, sans-serif; margin: 0; padding: 12px; background: #f0f2f5; color: #1a1a1a; }
    .wrap { max-width: 1800px; margin: 0 auto; }
    .header { margin-bottom: 10px; display: flex; align-items: baseline; gap: 12px; flex-wrap: wrap; }
    .header h1 { margin: 0; font-size: 20px; white-space: nowrap; }
    .header .muted { margin: 0; }
    .dashboard { display: grid; grid-template-columns: 1fr; gap: 10px; align-items: start; }
    .card-stream { grid-column: 1 / -1; }
    .videoWrap { position: relative; width: 100%; }
    img { width: 100%; height: auto; background: #111; border-radius: 8px; display: block; }
    canvas.overlay { position: absolute; left: 0; top: 0; width: 100%; height: 100%; pointer-events: none; }
    canvas.overlay.selectable { pointer-events: auto; cursor: crosshair; }
    code { background: #e8e8e8; padding: 1px 5px; border-radius: 4px; font-size: 12px; }
    .muted { color: #888; font-size: 12px; margin: 2px 0 6px; }
    .card { border: 1px solid #dde0e4; border-radius: 10px; padding: 12px; background: #fff; box-shadow: 0 1px 3px rgba(0,0,0,0.05); }
    .card h2 { margin: 0 0 2px; font-size: 15px; line-height: 1.3; }
    .card > .muted { margin-bottom: 8px; }
    .row { display: flex; gap: 10px; flex-wrap: wrap; align-items: center; }
    .row > * { flex: 1 1 180px; }
    input[type=range] { width: 100%; }
    button { padding: 6px 12px; border-radius: 8px; border: 1px solid #ccc; background: #fafafa; cursor: pointer; font-size: 13px; }
    button:hover { background: #eee; }
    button.primary { background: #111; color: #fff; border-color: #111; }
    button.primary:hover { background: #333; }
    button.danger { background: #b00020; color: #fff; border-color: #b00020; }
    button.danger:hover { background: #8f001a; }
    .kv { font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace; font-size: 12px; }
    @media (min-width: 960px) {
      .dashboard { grid-template-columns: repeat(2, 1fr); }
    }
    @media (min-width: 1400px) {
      .dashboard { grid-template-columns: repeat(3, 1fr); }
      .card-stream { grid-column: span 2; }
    }
  </style>
</head>
<body>
  <div class=\"wrap\">
    <div class=\"header\">
      <h1>&#129302; Raspbot Dashboard</h1>
      <p class=\"muted\">Real-time robot control &amp; sensor monitoring</p>
    </div>
    <div class=\"dashboard\">
        <div class=\"card card-stream\">
            <div class=\"videoWrap\" id=\"videoWrap\">
                <img id=\"video\" src=\"/stream.mjpg\" alt=\"camera stream\" />
                <canvas id=\"overlay\" class=\"overlay\"></canvas>
            </div>
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
                <div class=\"kv\">Selected: <span id=\"selectedState\">none</span></div>
                <button id=\"clearSelBtn\" style=\"flex: 0 0 auto;\">Clear selection</button>
            </div>
            <p class=\"muted\" style=\"margin-top: 4px;\">Click on a detected person in the video to select them for exclusive tracking/following.</p>
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

        <div class=\"card\" id=\"odomCard\">
            <h2>&#128506; Odometry &amp; Navigation</h2>
            <p class=\"muted\">Sensor-fused position (cmd_vel + IMU + ZUPT). Click to place numbered waypoints, then navigate the planned path. Scroll to zoom, drag to pan, right-click waypoint to remove.</p>

            <div style=\"display:flex;gap:16px;flex-wrap:wrap;align-items:flex-start;\">
                <!-- Path visualisation canvas -->
                <div style=\"flex:0 0 auto;text-align:center;\">
                    <canvas id=\"odomCanvas\" width=\"500\" height=\"500\" style=\"display:block;border:1px solid #ddd;border-radius:8px;background:#fafafa;cursor:crosshair;\"></canvas>
                    <div class=\"kv\" style=\"margin-top:4px;font-size:11px;\">
                        <span style=\"color:#e00;\">&#9632;</span> robot &nbsp;
                        <span style=\"color:#08f;\">&#9632;</span> trail &nbsp;
                        <span style=\"color:#0a0;\">&#9632;</span> recorded &nbsp;
                        <span style=\"color:#fa0;\">&#11044;</span> origin &nbsp;
                        <span style=\"color:#f0f;\">&#9670;</span> waypoint &nbsp;
                        <span style=\"color:#f80;\">━</span> planned
                    </div>
                    <div class=\"kv\" style=\"margin-top:2px;font-size:11px;color:#888;\">Click=add WP, R-click WP=remove, Drag=pan, Scroll=zoom, R-click=reset view</div>
                </div>

                <!-- Position readout + controls -->
                <div style=\"flex:1 1 280px;\">
                    <table style=\"width:100%;border-collapse:collapse;font-size:13px;font-family:ui-monospace,monospace;\">
                        <tr><td style=\"padding:3px 6px;color:#666;\">X (m)</td><td id=\"odomX\" style=\"padding:3px 6px;font-weight:bold;\">0.000</td></tr>
                        <tr><td style=\"padding:3px 6px;color:#666;\">Y (m)</td><td id=\"odomY\" style=\"padding:3px 6px;font-weight:bold;\">0.000</td></tr>
                        <tr><td style=\"padding:3px 6px;color:#666;\">Yaw (°)</td><td id=\"odomYaw\" style=\"padding:3px 6px;font-weight:bold;\">0.0</td></tr>
                        <tr><td style=\"padding:3px 6px;color:#666;\">Speed</td><td id=\"odomSpeed\" style=\"padding:3px 6px;\">0.000 m/s</td></tr>
                        <tr><td style=\"padding:3px 6px;color:#666;\">Distance</td><td id=\"odomDist\" style=\"padding:3px 6px;\">0.000 m</td></tr>
                    </table>

                    <div style=\"margin-top:10px;display:flex;flex-direction:column;gap:6px;\">
                        <div style=\"display:flex;gap:6px;flex-wrap:wrap;\">
                            <button id=\"odomSetOrigin\" title=\"Reset position to (0,0,0)\">&#127937; Set Origin</button>
                            <button class=\"primary\" id=\"odomStartRec\" title=\"Start recording path waypoints\">&#9899; Record</button>
                            <button id=\"odomStopRec\" title=\"Stop recording\" style=\"display:none;\">&#9724; Stop Rec</button>
                        </div>
                        <div style=\"display:flex;gap:6px;flex-wrap:wrap;\">
                            <button class=\"primary\" id=\"odomReturnBtn\" title=\"Navigate back to origin along recorded path\">&#8617; Return to Origin</button>
                            <button class=\"danger\" id=\"odomCancelBtn\" title=\"Cancel return-to-origin\" style=\"display:none;\">&#10006; Cancel Return</button>
                        </div>
                        <div style=\"display:flex;gap:6px;flex-wrap:wrap;\">
                            <button class=\"primary\" id=\"odomNavBtn\" title=\"Navigate the planned multi-waypoint path\" style=\"display:none;\">&#128205; Navigate Path</button>
                            <button class=\"danger\" id=\"odomNavCancelBtn\" title=\"Cancel navigation\" style=\"display:none;\">&#10006; Cancel Nav</button>
                            <button id=\"odomUndoWpBtn\" title=\"Remove last waypoint\" style=\"display:none;\">&#8630; Undo WP</button>
                            <button id=\"odomClearWpBtn\" title=\"Clear all waypoints\" style=\"display:none;\">&#128465; Clear All</button>
                        </div>
                    </div>

                    <div class=\"kv\" style=\"margin-top:8px;\">
                        Status: <span id=\"odomStatus\">idle</span>
                    </div>
                    <div id=\"odomStuckBanner\" style=\"background:#fdd;border:1px solid #e00;border-radius:6px;padding:6px 10px;margin-top:6px;display:none;font-size:12px;color:#b00;font-weight:bold;\">&#9888; STUCK DETECTED — auto-navigation cancelled</div>
                    <div class=\"kv\" style=\"margin-top:3px;font-size:11px;color:#888;\">
                        Waypoints: <span id=\"odomWpCount\">0</span> &nbsp;|&nbsp; Trail: <span id=\"odomTrailCount\">0</span>
                        &nbsp;|&nbsp; Nav: <span id=\"odomWpTarget\">idle</span>
                    </div>
                    <div style=\"margin-top:8px;padding-top:6px;border-top:1px solid #eee;\">
                        <div class=\"kv\" style=\"font-size:12px;\">&#128506; SLAM Map: <span id=\"slamStatus\" style=\"color:#888;\">waiting…</span></div>
                        <div style=\"display:flex;gap:6px;margin-top:4px;flex-wrap:wrap;\">
                            <button id=\"slamResetBtn\" title=\"Clear occupancy grid and reset SLAM\">&#128465; Reset Map</button>
                            <label style=\"display:flex;align-items:center;gap:4px;font-size:12px;\"><input type=\"checkbox\" id=\"slamOverlay\" checked /> Show map</label>
                        </div>
                        <div class=\"kv\" style=\"margin-top:4px;font-size:11px;\">
                            <span style=\"background:#ddd;display:inline-block;width:10px;height:10px;border:1px solid #aaa;\"></span> free &nbsp;
                            <span style=\"background:#333;display:inline-block;width:10px;height:10px;border:1px solid #aaa;\"></span> wall &nbsp;
                            Cells: <span id=\"slamCells\">0</span>
                        </div>
                    </div>
                    <div style=\"margin-top:8px;padding-top:6px;border-top:1px solid #eee;\">
                        <div class=\"kv\" style=\"font-size:12px;\">&#127993; Front Calibration: <span id=\"calStatus\" style=\"color:#888;\">idle</span></div>
                        <div style=\"font-size:11px;color:#888;margin:3px 0;\">Place robot facing a wall, then calibrate.</div>
                        <div style=\"display:flex;gap:6px;margin-top:4px;flex-wrap:wrap;\">
                            <button id=\"calStaticBtn\" title=\"Static calibration using ultrasonic + LiDAR (no motion)\">&#128270; Static</button>
                            <button id=\"calFullBtn\" title=\"Full calibration with brief forward drive + IMU check\">&#128296; Full</button>
                        </div>
                        <div id=\"calResult\" style=\"margin-top:4px;font-size:11px;color:#555;display:none;background:#f8f8f0;padding:4px 6px;border-radius:3px;border:1px solid #ddd;\"></div>
                    </div>
                </div>
            </div>
        </div>

        <div class=\"card\" id=\"lidarCard\">
            <h2>&#128752; LiDAR Scan</h2>
            <p class=\"muted\">Live 360° point cloud from YDLidar T-mini Plus on <code>/scan</code>. Top-down view: forward = up.</p>
            <div style=\"display:flex;gap:16px;flex-wrap:wrap;align-items:flex-start;\">
                <div style=\"flex:0 0 auto;text-align:center;\">
                    <canvas id=\"lidarCanvas\" width=\"400\" height=\"400\" style=\"display:block;border:1px solid #ddd;border-radius:8px;background:#111;\"></canvas>
                    <div class=\"kv\" style=\"margin-top:4px;font-size:11px;\"><span style=\"color:#0f0;\">&#9632;</span> points &nbsp; <span style=\"color:#e00;\">&#9632;</span> robot &nbsp; rings = range</div>
                </div>
                <div style=\"flex:1 1 200px;\">
                    <table style=\"width:100%;border-collapse:collapse;font-size:13px;font-family:ui-monospace,monospace;\">
                        <tr><td style=\"padding:3px 6px;color:#666;\">Points</td><td id=\"lidarPoints\" style=\"padding:3px 6px;font-weight:bold;\">0</td></tr>
                        <tr><td style=\"padding:3px 6px;color:#666;\">Range</td><td id=\"lidarRange\" style=\"padding:3px 6px;\">— m</td></tr>
                        <tr><td style=\"padding:3px 6px;color:#666;\">FOV</td><td id=\"lidarFov\" style=\"padding:3px 6px;\">—°</td></tr>
                        <tr><td style=\"padding:3px 6px;color:#666;\">Status</td><td id=\"lidarStatus\" style=\"padding:3px 6px;color:#888;\">waiting…</td></tr>
                    </table>
                    <div style=\"margin-top:8px;padding-top:6px;border-top:1px solid #eee;\">
                        <div class=\"kv\" style=\"font-size:12px;\">&#128737; Obstacle Zones</div>
                        <table style=\"width:100%;border-collapse:collapse;font-size:12px;font-family:ui-monospace,monospace;margin-top:2px;\">
                            <tr><td style=\"padding:2px 6px;color:#666;\">Front</td><td id=\"ozFront\" style=\"padding:2px 6px;\">—</td></tr>
                            <tr><td style=\"padding:2px 6px;color:#666;\">Left</td><td id=\"ozLeft\" style=\"padding:2px 6px;\">—</td></tr>
                            <tr><td style=\"padding:2px 6px;color:#666;\">Right</td><td id=\"ozRight\" style=\"padding:2px 6px;\">—</td></tr>
                            <tr><td style=\"padding:2px 6px;color:#666;\">Rear</td><td id=\"ozRear\" style=\"padding:2px 6px;\">—</td></tr>
                        </table>
                    </div>
                    <div style=\"margin-top:10px;\">
                        <label for=\"lidarZoom\">Zoom: <span id=\"lidarZoomVal\">auto</span></label>
                        <input id=\"lidarZoom\" type=\"range\" min=\"0\" max=\"12\" step=\"0.5\" value=\"0\" style=\"width:100%;\" />
                    </div>
                </div>
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
                    <canvas id=\"imuRobotCanvas\" width=\"200\" height=\"200\" style=\"display:block;margin:0 auto;border:1px solid #ddd;border-radius:8px;background:#fafafa;\"></canvas>
                    <div class=\"kv\" style=\"margin-top:4px;\">Yaw: <span id=\"imuYawLabel\">0.0</span>° &nbsp; P: <span id=\"imuPitchLabel\">0.0</span>° &nbsp; R: <span id=\"imuRollLabel\">0.0</span>°</div>
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

        <div class=\"card\" id=\"faceCard\">
            <h2>&#128100; Face Recognition</h2>
            <p class=\"muted\">Manage enrolled faces. Faces are auto-detected and stored with 128-d SFace embeddings in a local SQLite DB.</p>
            <div class=\"row\" style=\"margin-bottom:8px;\">
                <button class=\"primary\" id=\"faceRefreshBtn\">Refresh</button>
                <button class=\"danger\" id=\"faceClearBtn\">Clear All</button>
                <div class=\"kv\">Enrolled: <span id=\"faceCount\">0</span></div>
            </div>
            <div id=\"faceList\" style=\"display:grid;grid-template-columns:repeat(auto-fill,minmax(180px,1fr));gap:8px;\"></div>
            <div id=\"faceMergePanel\" style=\"margin-top:10px;border-top:1px solid #eee;padding-top:10px;display:none;\">
                <h3 style=\"margin:0 0 6px;font-size:14px;\">Merge Faces</h3>
                <div class=\"row\">
                    <div>
                        <label>Keep ID: <input id=\"faceKeepId\" type=\"number\" min=\"1\" style=\"width:60px;padding:4px;border:1px solid #ccc;border-radius:4px;\" /></label>
                    </div>
                    <div>
                        <label>Merge ID: <input id=\"faceMergeId\" type=\"number\" min=\"1\" style=\"width:60px;padding:4px;border:1px solid #ccc;border-radius:4px;\" /></label>
                    </div>
                    <button class=\"primary\" id=\"faceMergeBtn\">Merge</button>
                </div>
            </div>
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
        const selectedStateEl = document.getElementById('selectedState');
        const clearSelBtn = document.getElementById('clearSelBtn');
        const followDist = document.getElementById('followDist');
        const followDistVal = document.getElementById('followDistVal');
        const followSpeed = document.getElementById('followSpeed');
        const followSpeedVal = document.getElementById('followSpeedVal');
        const followStrafe = document.getElementById('followStrafe');
        const followStrafeVal = document.getElementById('followStrafeVal');
        const followGyroDamp = document.getElementById('followGyroDamp');
        const followGyroDampVal = document.getElementById('followGyroDampVal');

        let debounceTimer = null;
        let selectedPersonId = -1;

        /* Track when a user last touched each slider so the status poll
           does not overwrite the value while the user is still dragging. */
        const _lastUserInput = {};  // element-id → timestamp
        const USER_INPUT_GRACE_MS = 3000;
        function markUserInput(el) { _lastUserInput[el.id] = Date.now(); }
        function userRecentlyTouched(el) {
            const t = _lastUserInput[el.id];
            return t && (Date.now() - t) < USER_INPUT_GRACE_MS;
        }

        // ── Fetch utilities: AbortController timeout + in-flight guard ─
        function guardedFetch(url, opts = {}, timeoutMs = 3000) {
            const ac = new AbortController();
            const tid = setTimeout(() => ac.abort(), timeoutMs);
            return fetch(url, {...opts, signal: ac.signal})
                .finally(() => clearTimeout(tid));
        }
        function polled(fn, ms) {
            let busy = false;
            const wrapped = async () => {
                if (busy) return;
                busy = true;
                try { await fn(); } finally { busy = false; }
            };
            wrapped();
            setInterval(wrapped, ms);
        }
        let _onStatusCollisionCliff = null;

        function updateLabels() {
            panVal.textContent = String(pan.value);
            tiltVal.textContent = String(tilt.value);
        }

        async function fetchStatus() {
            try {
                const r = await guardedFetch('/status');
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
                    // Sync person selection state from server
                    if (typeof j.tracking.selected_person_id === 'number') {
                        selectedPersonId = j.tracking.selected_person_id;
                        updateSelectionUi();
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
                if (_onStatusCollisionCliff) _onStatusCollisionCliff(j);
            } catch (e) {
                if (e.name !== 'AbortError') statusEl.textContent = 'status error';
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

        // ---- Person selection (click-to-track) ----

        async function selectPerson(personId) {
            selectedPersonId = personId;
            updateSelectionUi();
            try {
                await fetch('/api/tracking/select', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({person_id: personId})
                });
            } catch (e) { /* ignore */ }
        }

        async function clearSelection() {
            await selectPerson(-1);
        }

        function updateSelectionUi() {
            if (selectedPersonId >= 0) {
                selectedStateEl.textContent = `Person #${selectedPersonId}`;
                selectedStateEl.style.color = '#00aaff';
                selectedStateEl.style.fontWeight = 'bold';
                overlay.classList.add('selectable');
            } else {
                selectedStateEl.textContent = 'none';
                selectedStateEl.style.color = '';
                selectedStateEl.style.fontWeight = '';
            }
        }

        // Make overlay clickable when boxes are visible.
        // Toggle selectability based on checkbox (always selectable when boxes on).
        function updateOverlayClickable() {
            if (boxesChk.checked) {
                overlay.classList.add('selectable');
            } else {
                overlay.classList.remove('selectable');
            }
        }
        boxesChk.addEventListener('change', updateOverlayClickable);
        updateOverlayClickable();

        overlay.addEventListener('click', (e) => {
            const j = latestDet;
            if (!j || !Array.isArray(j.detections) || !j.image_width || !j.image_height) return;

            const rect = overlay.getBoundingClientRect();
            const clickX = (e.clientX - rect.left) * (overlay.width / rect.width);
            const clickY = (e.clientY - rect.top) * (overlay.height / rect.height);

            const sx = overlay.width / Number(j.image_width);
            const sy = overlay.height / Number(j.image_height);

            // Prefer person-class (class_id=0) detections for selection
            // because person bboxes have the most stable track IDs.
            // Fall back to any detection class if no person box matches.
            let bestDet = null;
            let bestArea = Infinity;
            let fallbackDet = null;
            let fallbackArea = Infinity;
            for (const d of j.detections) {
                const x = Number(d.x) * sx;
                const y = Number(d.y) * sy;
                const w = Number(d.w) * sx;
                const h = Number(d.h) * sy;
                if (!Number.isFinite(x + y + w + h)) continue;
                if (clickX >= x && clickX <= x + w && clickY >= y && clickY <= y + h) {
                    const area = w * h;
                    if (typeof d.track_id === 'number' && d.track_id >= 0) {
                        if (d.class_id === 0 && area < bestArea) {
                            bestArea = area;
                            bestDet = d;
                        } else if (area < fallbackArea) {
                            fallbackArea = area;
                            fallbackDet = d;
                        }
                    }
                }
            }
            if (!bestDet) bestDet = fallbackDet;

            if (bestDet) {
                selectPerson(bestDet.track_id);
            }
        });

        clearSelBtn.addEventListener('click', () => clearSelection());
        updateSelectionUi();

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
                const r = await guardedFetch('/detections', {cache: 'no-store'});
                const j = await r.json();
                latestDet = j;
                lastDetFetchMs = Date.now();
                const n = Array.isArray(j.detections) ? j.detections.length : 0;
                detStateEl.textContent = `n=${n}`;
            } catch (e) {
                // Keep last detections; just show stale
                if (e.name !== 'AbortError') detStateEl.textContent = 'error';
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

                const tid = (typeof d.track_id === 'number') ? d.track_id : -1;
                const isSelected = (selectedPersonId >= 0 && tid === selectedPersonId);

                ctx.strokeStyle = isSelected ? '#00aaff' : '#00ff66';
                ctx.lineWidth = isSelected ? 3 : 2;
                ctx.fillStyle = isSelected ? 'rgba(0, 100, 255, 0.55)' : 'rgba(0, 0, 0, 0.45)';
                ctx.strokeRect(x, y, w, h);

                const label = String(d.label ?? d.class_id ?? '?');
                const score = Number(d.score);
                const idTag = tid >= 0 ? `#${tid} ` : '';
                const faceName = d.face_name ? `[${d.face_name}] ` : '';
                const text = Number.isFinite(score) ? `${faceName}${idTag}${label} ${(score*100).toFixed(0)}%` : `${faceName}${idTag}${label}`;
                const tw = ctx.measureText(text).width + 6;
                const th = 14;
                ctx.fillRect(x, Math.max(0, y - th), tw, th);
                ctx.fillStyle = '#ffffff';
                ctx.fillText(text, x + 3, Math.max(0, y - th) + 1);
                ctx.lineWidth = 2;  // reset
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

        // WASD: drive. Fire-and-forget sends to avoid request pile-up.
        const pressed = new Set();
        let lastLin = 0;
        let lastAng = 0;
        let driveTimer = null;
        let driveSending = false;  // guard against overlapping requests

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

        function fireDriveSend() {
            if (driveSending) return;  // skip if previous request still in-flight
            const {lin, lat, ang} = computeDrive();
            lastLin = lin;
            lastAng = ang;
            cmdVelEl.textContent = `${lin.toFixed(2)}, ${lat.toFixed(2)}, ${ang.toFixed(2)}`;
            driveSending = true;
            fetch('/api/cmd_vel', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({linear_x: lin, linear_y: lat, angular_z: ang})
            }).catch(() => {}).finally(() => { driveSending = false; });
        }

        function startDriveLoop() {
            fireDriveSend();  // send first command immediately
            if (driveTimer) return;
            driveTimer = setInterval(fireDriveSend, 60);  // 16 Hz sustain
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
            // Fire-and-forget stop
            fetch('/api/cmd_vel/stop', {method:'POST'}).catch(() => {});
            driveSending = false;
        }

        function normalizeKey(ev) {
            if (ev.key === 'Shift') return 'shift';
            return String(ev.key || '').toLowerCase();
        }

        document.addEventListener('keydown', (ev) => {
            const k = normalizeKey(ev);
            if (!['w','a','s','d','q','e','shift'].includes(k)) return;
            ev.preventDefault();
            const wasEmpty = pressed.size === 0;
            pressed.add(k);
            if (wasEmpty) {
                startDriveLoop();
            } else {
                fireDriveSend();  // key combo changed — send updated velocity immediately
            }
        }, { passive: false });

        document.addEventListener('keyup', (ev) => {
            const k = normalizeKey(ev);
            if (!['w','a','s','d','q','e','shift'].includes(k)) return;
            ev.preventDefault();
            pressed.delete(k);
            if (pressed.size === 0) {
                stopDriveLoop();
            } else {
                fireDriveSend();  // key combo changed — send updated velocity immediately
            }
        }, { passive: false });

        window.addEventListener('blur', () => stopDriveLoop());
        document.addEventListener('visibilitychange', () => {
            if (document.hidden) stopDriveLoop();
        });

        updateLabels();
        polled(fetchStatus, 1000);
        polled(fetchDetections, 333);
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
                const r = await guardedFetch('/api/imu', {cache: 'no-store'});
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
                drawRobotTopDown(d);
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

        const imuRobotCanvas = document.getElementById('imuRobotCanvas');
        const imuPitchLabel = document.getElementById('imuPitchLabel');
        const imuRollLabel = document.getElementById('imuRollLabel');

        function drawRobotTopDown(d) {
            if (!d) return;
            const ctx = imuRobotCanvas.getContext('2d');
            const W = imuRobotCanvas.width, H = imuRobotCanvas.height;
            const cx = W / 2, cy = H / 2;
            ctx.clearRect(0, 0, W, H);

            const yaw = (d.yaw_deg || 0) * Math.PI / 180;
            const pitch = d.pitch_deg || 0;
            const roll = d.roll_deg || 0;

            // Update pitch/roll labels
            imuPitchLabel.textContent = pitch.toFixed(1);
            imuRollLabel.textContent = roll.toFixed(1);

            ctx.save();
            ctx.translate(cx, cy);
            ctx.rotate(-yaw);  // top-down: +yaw = CCW on canvas = CW in real world (robot-right)

            // Robot body (rectangle: wider than tall = mecanum platform)
            const bw = 60, bh = 80; // width x length (forward is up = -Y)
            ctx.fillStyle = '#e8e8e8';
            ctx.strokeStyle = '#888';
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.roundRect(-bw/2, -bh/2, bw, bh, 6);
            ctx.fill();
            ctx.stroke();

            // 4 mecanum wheels
            const wheelW = 12, wheelH = 22;
            ctx.fillStyle = '#555';
            ctx.strokeStyle = '#333';
            ctx.lineWidth = 1.5;
            const wx = bw/2 + 2, wy = bh/2 - 16;
            [[-wx, -wy], [wx - wheelW, -wy], [-wx, wy - wheelH + 4], [wx - wheelW, wy - wheelH + 4]].forEach(([x, y]) => {
                ctx.beginPath();
                ctx.roundRect(x, y, wheelW, wheelH, 3);
                ctx.fill();
                ctx.stroke();
            });

            // Forward indicator (arrow at front)
            ctx.beginPath();
            ctx.moveTo(0, -bh/2 - 4);
            ctx.lineTo(-10, -bh/2 - 16);
            ctx.lineTo(10, -bh/2 - 16);
            ctx.closePath();
            ctx.fillStyle = '#0c0';
            ctx.fill();
            ctx.font = 'bold 9px ui-monospace, monospace';
            ctx.textAlign = 'center';
            ctx.fillStyle = '#0a0';
            ctx.fillText('FWD', 0, -bh/2 - 18);

            // Left/Right labels
            ctx.fillStyle = '#f80';
            ctx.font = '9px ui-monospace, monospace';
            ctx.textAlign = 'center';
            ctx.fillText('L', -bw/2 - 10, 3);
            ctx.fillStyle = '#08f';
            ctx.fillText('R', bw/2 + 10, 3);

            // Tilt indicator (cross-hair shifted by pitch/roll)
            // Pitch shifts dot forward/back, roll shifts left/right
            const tiltScale = 0.6;  // pixels per degree
            const tiltX = roll * tiltScale;   // roll-right → dot moves right
            const tiltY = -pitch * tiltScale; // pitch-forward → dot moves up (forward)
            const tiltMag = Math.sqrt(pitch*pitch + roll*roll);
            // Tilt cross
            ctx.strokeStyle = '#ccc';
            ctx.lineWidth = 0.5;
            ctx.beginPath(); ctx.moveTo(-15, 0); ctx.lineTo(15, 0); ctx.stroke();
            ctx.beginPath(); ctx.moveTo(0, -15); ctx.lineTo(0, 15); ctx.stroke();
            // Tilt dot
            ctx.beginPath();
            ctx.arc(tiltX, tiltY, 4, 0, 2 * Math.PI);
            ctx.fillStyle = tiltMag > 10 ? '#e00' : tiltMag > 3 ? '#fa0' : '#0a0';
            ctx.fill();

            ctx.restore();

            // Cardinal labels (fixed, don't rotate)
            ctx.font = '10px ui-monospace, monospace';
            ctx.textAlign = 'center';
            ctx.fillStyle = '#999';
            ctx.fillText('0°', cx, 12);
            ctx.fillText('180°', cx, H - 4);
            ctx.fillText('90°', W - 8, cy + 4);
            ctx.fillText('-90°', 14, cy + 4);
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

        polled(fetchImu, 333);

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

        // Collision/cliff status updates are piggybacked on fetchStatus()
        _onStatusCollisionCliff = function(d) {
            if (d.collision_failsafe) {
                const cf = d.collision_failsafe;
                const dist_m = cf.distance_m;
                if (dist_m !== null && dist_m !== undefined && isFinite(dist_m)) {
                    collisionDist.textContent = dist_m.toFixed(2) + ' m';
                } else {
                    collisionDist.textContent = '\u2014';
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
                updateUltrasonicOverlay(cf);
            }
            if (d.cliff_failsafe) {
                const cl = d.cliff_failsafe;
                const state = cl.sensor_state || 0;
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
        };

        // ──────────────────────────────────────────────────────────────
        // Odometry & Navigation
        // ──────────────────────────────────────────────────────────────
        const odomCanvas = document.getElementById('odomCanvas');
        const odomCtx = odomCanvas.getContext('2d');
        const odomX = document.getElementById('odomX');
        const odomY = document.getElementById('odomY');
        const odomYaw = document.getElementById('odomYaw');
        const odomSpeed = document.getElementById('odomSpeed');
        const odomDist = document.getElementById('odomDist');
        const odomStatus = document.getElementById('odomStatus');
        const odomWpCount = document.getElementById('odomWpCount');
        const odomTrailCount = document.getElementById('odomTrailCount');
        const odomWpTarget = document.getElementById('odomWpTarget');
        const odomSetOriginBtn = document.getElementById('odomSetOrigin');
        const odomStartRecBtn = document.getElementById('odomStartRec');
        const odomStopRecBtn = document.getElementById('odomStopRec');
        const odomReturnBtn = document.getElementById('odomReturnBtn');
        const odomCancelBtn = document.getElementById('odomCancelBtn');
        const odomNavBtn = document.getElementById('odomNavBtn');
        const odomNavCancelBtn = document.getElementById('odomNavCancelBtn');
        const odomClearWpBtn = document.getElementById('odomClearWpBtn');
        const odomUndoWpBtn = document.getElementById('odomUndoWpBtn');
        const odomStuckBanner = document.getElementById('odomStuckBanner');

        // SLAM map elements
        const slamStatusEl = document.getElementById('slamStatus');
        const slamCellsEl = document.getElementById('slamCells');
        const slamResetBtn = document.getElementById('slamResetBtn');
        const slamOverlayChk = document.getElementById('slamOverlay');

        // Front calibration elements
        const calStatusEl = document.getElementById('calStatus');
        const calStaticBtn = document.getElementById('calStaticBtn');
        const calFullBtn = document.getElementById('calFullBtn');
        const calResultEl = document.getElementById('calResult');

        let odomRecording = false;
        let odomReturning = false;
        let odomNavigating = false;
        let odomTotalDist = 0.0;
        let odomPrevX = null, odomPrevY = null;
        let odomBusy = false;

        // Pan & Zoom state
        let odomPanX = 0, odomPanY = 0; // pan offset in pixels
        let odomZoom = 1.0; // zoom multiplier (1.0 = auto-fit)
        let odomUserZoom = false; // true when user has manually zoomed
        let odomDragging = false;
        let odomDragStartX = 0, odomDragStartY = 0;
        let odomPanStartX = 0, odomPanStartY = 0;

        // Waypoint placement — multi-waypoint list
        let odomWaypoints = []; // [{x, y}, ...] in map coords
        let odomPlannedPath = []; // [{x, y}, ...] A*-planned path through all waypoints
        let odomDragWpIdx = -1; // index of waypoint being dragged, or -1
        const WAYPOINT_HIT_RADIUS = 12; // px radius for click/right-click hit testing

        // SLAM map state
        let lastMapData = null;
        let lastMapBytes = null;
        let slamOverlayOn = true;
        slamOverlayChk.addEventListener('change', function() { slamOverlayOn = this.checked; });

        // Last known auto-computed scale/maxR for canvas↔map conversion
        let lastOdomScale = 1.0;
        let lastOdomMaxR = 0.5;
        let lastOdomData = null;

        // ── Pan & Zoom handlers ──────────────────────────────────────
        odomCanvas.addEventListener('wheel', function(e) {
            e.preventDefault();
            const zoomFactor = e.deltaY < 0 ? 1.15 : 0.87;
            odomZoom = Math.max(0.1, Math.min(50, odomZoom * zoomFactor));
            odomUserZoom = true;
            if (lastOdomData) drawOdomMap(lastOdomData);
        }, {passive: false});

        // Helper: convert canvas pixel to map coords
        function canvasToMap(canvasX, canvasY) {
            const W = odomCanvas.width, H = odomCanvas.height;
            const cx2 = W / 2 + odomPanX;
            const cy2 = H / 2 + odomPanY;
            const scale = lastOdomScale * odomZoom;
            if (scale < 0.001) return null;
            const my = (cx2 - canvasX) / scale;
            const mx = (cy2 - canvasY) / scale;
            return {x: mx, y: my};
        }

        // Helper: map coords to canvas pixel
        function mapToCanvas(mx, my) {
            const W = odomCanvas.width, H = odomCanvas.height;
            const cx2 = W / 2 + odomPanX;
            const cy2 = H / 2 + odomPanY;
            const scale = lastOdomScale * odomZoom;
            return [cx2 - my * scale, cy2 - mx * scale];
        }

        // Helper: find waypoint index near canvas pixel, or -1
        function hitTestWaypoint(canvasX, canvasY) {
            for (let i = 0; i < odomWaypoints.length; i++) {
                const [wpx, wpy] = mapToCanvas(odomWaypoints[i].x, odomWaypoints[i].y);
                const dx = canvasX - wpx, dy = canvasY - wpy;
                if (Math.sqrt(dx*dx + dy*dy) <= WAYPOINT_HIT_RADIUS) return i;
            }
            return -1;
        }

        // Update UI buttons & label after waypoint list changes
        function syncWaypointUI() {
            const n = odomWaypoints.length;
            odomWpTarget.textContent = n > 0 ? (n + ' waypoint' + (n > 1 ? 's' : '')) : 'idle';
            odomNavBtn.style.display = n > 0 ? '' : 'none';
            odomClearWpBtn.style.display = n > 0 ? '' : 'none';
            odomUndoWpBtn.style.display = n > 0 ? '' : 'none';
            replanPath();
            if (lastOdomData) drawOdomMap(lastOdomData);
        }

        odomCanvas.addEventListener('mousedown', function(e) {
            if (e.button === 0) { // left button
                // Check if clicking on existing waypoint to drag it
                const hitIdx = hitTestWaypoint(e.offsetX, e.offsetY);
                if (hitIdx >= 0) {
                    odomDragWpIdx = hitIdx;
                    odomCanvas.style.cursor = 'move';
                    return;
                }
                odomDragging = true;
                odomDragStartX = e.offsetX;
                odomDragStartY = e.offsetY;
                odomPanStartX = odomPanX;
                odomPanStartY = odomPanY;
                odomCanvas.style.cursor = 'grabbing';
            }
        });
        odomCanvas.addEventListener('mousemove', function(e) {
            if (odomDragWpIdx >= 0) {
                // Dragging a waypoint
                const pt = canvasToMap(e.offsetX, e.offsetY);
                if (pt) {
                    odomWaypoints[odomDragWpIdx] = pt;
                    replanPath();
                    if (lastOdomData) drawOdomMap(lastOdomData);
                }
                return;
            }
            if (odomDragging) {
                odomPanX = odomPanStartX + (e.offsetX - odomDragStartX);
                odomPanY = odomPanStartY + (e.offsetY - odomDragStartY);
                if (lastOdomData) drawOdomMap(lastOdomData);
            }
        });
        odomCanvas.addEventListener('mouseup', function(e) {
            if (odomDragWpIdx >= 0) {
                // Finish waypoint drag
                const pt = canvasToMap(e.offsetX, e.offsetY);
                if (pt) odomWaypoints[odomDragWpIdx] = pt;
                odomDragWpIdx = -1;
                odomCanvas.style.cursor = 'crosshair';
                syncWaypointUI();
                return;
            }
            if (odomDragging) {
                const dx = Math.abs(e.offsetX - odomDragStartX);
                const dy = Math.abs(e.offsetY - odomDragStartY);
                odomDragging = false;
                odomCanvas.style.cursor = 'crosshair';
                // If it was a real drag (>3px), don't place waypoint
                if (dx > 3 || dy > 3) return;
                // Click — add waypoint
                const pt = canvasToMap(e.offsetX, e.offsetY);
                if (pt) {
                    odomWaypoints.push(pt);
                    syncWaypointUI();
                }
            }
        });
        odomCanvas.addEventListener('mouseleave', function() {
            odomDragging = false;
            odomDragWpIdx = -1;
            odomCanvas.style.cursor = 'crosshair';
        });

        // Double-click = add waypoint and start navigation immediately
        odomCanvas.addEventListener('dblclick', function(e) {
            e.preventDefault();
            const pt = canvasToMap(e.offsetX, e.offsetY);
            if (pt) {
                odomWaypoints.push(pt);
                syncWaypointUI();
                navigatePath();
            }
        });

        // Right-click: if near a waypoint remove it, else reset pan/zoom
        odomCanvas.addEventListener('contextmenu', function(e) {
            e.preventDefault();
            const hitIdx = hitTestWaypoint(e.offsetX, e.offsetY);
            if (hitIdx >= 0) {
                odomWaypoints.splice(hitIdx, 1);
                syncWaypointUI();
            } else {
                odomPanX = 0; odomPanY = 0;
                odomZoom = 1.0; odomUserZoom = false;
                if (lastOdomData) drawOdomMap(lastOdomData);
            }
        });

        // Undo last waypoint
        odomUndoWpBtn.addEventListener('click', function() {
            if (odomWaypoints.length > 0) {
                odomWaypoints.pop();
                syncWaypointUI();
            }
        });

        // Clear all waypoints
        odomClearWpBtn.addEventListener('click', function() {
            odomWaypoints = [];
            odomPlannedPath = [];
            syncWaypointUI();
        });

        // ── A* Path Planner ──────────────────────────────────────────
        // Uses SLAM occupancy grid to plan obstacle-free path
        function replanPath() {
            odomPlannedPath = [];
            if (odomWaypoints.length === 0) return;
            // Build start→wp1→wp2→...→wpN chain
            const robotPt = lastOdomData ? {x: lastOdomData.x, y: lastOdomData.y} : {x: 0, y: 0};
            const chain = [robotPt].concat(odomWaypoints);
            const fullPath = [];
            for (let i = 0; i < chain.length - 1; i++) {
                const seg = astarSegment(chain[i], chain[i+1]);
                // Append segment (skip first point of subsequent segments to avoid dups)
                if (seg && seg.length > 0) {
                    const start = (i === 0) ? 0 : 1;
                    for (let j = start; j < seg.length; j++) fullPath.push(seg[j]);
                } else {
                    // No SLAM data or direct path — just straight line
                    if (i > 0 || fullPath.length === 0) fullPath.push(chain[i]);
                    fullPath.push(chain[i+1]);
                }
            }
            odomPlannedPath = fullPath;
        }

        function astarSegment(startPt, endPt) {
            // If no SLAM data, return null (straight line fallback)
            if (!lastMapData || !lastMapData.available || !lastMapBytes) return null;
            const mw = lastMapData.width, mh = lastMapData.height;
            const mres = lastMapData.resolution;
            const ox = lastMapData.origin_x, oy = lastMapData.origin_y;
            if (mw === 0 || mh === 0 || mres <= 0) return null;

            // Convert map coords to grid cell
            function toCell(pt) {
                const col = Math.round((pt.y - oy) / mres);
                const row = Math.round((pt.x - ox) / mres);
                return [row, col];
            }
            function toWorld(row, col) {
                return {x: ox + row * mres, y: oy + col * mres};
            }

            const [sr, sc] = toCell(startPt);
            const [er, ec] = toCell(endPt);

            // Inflate obstacles — build blocked set (inflate by robot radius ~0.15m)
            const inflateR = Math.max(1, Math.ceil(0.15 / mres));
            // For performance, build a typed array for inflated grid
            const blocked = new Uint8Array(mw * mh);
            for (let r = 0; r < mh; r++) {
                for (let c = 0; c < mw; c++) {
                    const v = lastMapBytes[r * mw + c];
                    if (v !== 255 && v > 50) { // occupied
                        // Inflate
                        for (let dr = -inflateR; dr <= inflateR; dr++) {
                            for (let dc = -inflateR; dc <= inflateR; dc++) {
                                const nr = r + dr, nc = c + dc;
                                if (nr >= 0 && nr < mh && nc >= 0 && nc < mw) {
                                    blocked[nr * mw + nc] = 1;
                                }
                            }
                        }
                    }
                }
            }

            // Check start/end are in bounds
            function inBounds(r, c) { return r >= 0 && r < mh && c >= 0 && c < mw; }
            // If start or end is out of bounds or blocked, fallback to straight line
            if (!inBounds(sr, sc) || !inBounds(er, ec)) return null;
            // If start or end is in blocked area, try to find nearest free cell
            function findFreeNear(r, c) {
                if (inBounds(r, c) && !blocked[r * mw + c]) return [r, c];
                for (let d = 1; d <= inflateR + 2; d++) {
                    for (let dr = -d; dr <= d; dr++) {
                        for (let dc = -d; dc <= d; dc++) {
                            const nr = r + dr, nc = c + dc;
                            if (inBounds(nr, nc) && !blocked[nr * mw + nc]) return [nr, nc];
                        }
                    }
                }
                return null;
            }
            const freeStart = findFreeNear(sr, sc);
            const freeEnd = findFreeNear(er, ec);
            if (!freeStart || !freeEnd) return null;
            const [asr, asc] = freeStart;
            const [aer, aec] = freeEnd;

            // A* search (8-connected)
            const SQRT2 = 1.414;
            const dirs = [[-1,0,1],[1,0,1],[0,-1,1],[0,1,1],[-1,-1,SQRT2],[-1,1,SQRT2],[1,-1,SQRT2],[1,1,SQRT2]];
            const key = (r, c) => r * mw + c;
            const startKey = key(asr, asc);
            const endKey = key(aer, aec);

            // heuristic: octile distance
            function heuristic(r, c) {
                const dr = Math.abs(r - aer), dc = Math.abs(c - aec);
                return Math.max(dr, dc) + (SQRT2 - 1) * Math.min(dr, dc);
            }

            // Simple binary heap for open set
            const gScore = new Float32Array(mw * mh).fill(Infinity);
            const fScore = new Float32Array(mw * mh).fill(Infinity);
            const cameFrom = new Int32Array(mw * mh).fill(-1);
            const closed = new Uint8Array(mw * mh);

            gScore[startKey] = 0;
            fScore[startKey] = heuristic(asr, asc);

            // Min-heap using array
            const open = [[fScore[startKey], startKey]]; // [f, key]
            let found = false;
            let iterations = 0;
            const MAX_ITER = 50000; // prevent freezing on large maps

            while (open.length > 0 && iterations < MAX_ITER) {
                iterations++;
                // Find min f in open (simple for moderate sizes)
                let minIdx = 0;
                for (let i = 1; i < open.length; i++) {
                    if (open[i][0] < open[minIdx][0]) minIdx = i;
                }
                const [, curKey] = open[minIdx];
                open[minIdx] = open[open.length - 1];
                open.pop();

                if (curKey === endKey) { found = true; break; }
                if (closed[curKey]) continue;
                closed[curKey] = 1;

                const cr = Math.floor(curKey / mw), cc = curKey % mw;
                for (const [dr, dc, cost] of dirs) {
                    const nr = cr + dr, nc = cc + dc;
                    if (!inBounds(nr, nc)) continue;
                    const nk = key(nr, nc);
                    if (closed[nk] || blocked[nk]) continue;
                    const ng = gScore[curKey] + cost;
                    if (ng < gScore[nk]) {
                        gScore[nk] = ng;
                        fScore[nk] = ng + heuristic(nr, nc);
                        cameFrom[nk] = curKey;
                        open.push([fScore[nk], nk]);
                    }
                }
            }

            if (!found) return null; // no path found — fallback to straight

            // Reconstruct path
            const rawPath = [];
            let ck = endKey;
            while (ck !== -1) {
                const cr = Math.floor(ck / mw), cc = ck % mw;
                rawPath.push(toWorld(cr, cc));
                ck = cameFrom[ck];
            }
            rawPath.reverse();

            // Path smoothing: line-of-sight skip
            if (rawPath.length <= 2) return rawPath;
            const smoothed = [rawPath[0]];
            let ci = 0;
            while (ci < rawPath.length - 1) {
                let furthest = ci + 1;
                for (let j = rawPath.length - 1; j > ci + 1; j--) {
                    if (lineOfSight(rawPath[ci], rawPath[j], blocked, mw, mh, mres, ox, oy)) {
                        furthest = j;
                        break;
                    }
                }
                smoothed.push(rawPath[furthest]);
                ci = furthest;
            }
            return smoothed;
        }

        // Bresenham line-of-sight check through the inflated grid
        function lineOfSight(a, b, blocked, mw, mh, mres, ox, oy) {
            const c0 = Math.round((a.y - oy) / mres);
            const r0 = Math.round((a.x - ox) / mres);
            const c1 = Math.round((b.y - oy) / mres);
            const r1 = Math.round((b.x - ox) / mres);
            let dr = Math.abs(r1 - r0), dc = Math.abs(c1 - c0);
            let r = r0, c = c0;
            let sr = r0 < r1 ? 1 : -1, sc = c0 < c1 ? 1 : -1;
            let err = dr - dc;
            while (true) {
                if (r < 0 || r >= mh || c < 0 || c >= mw) return false;
                if (blocked[r * mw + c]) return false;
                if (r === r1 && c === c1) break;
                const e2 = 2 * err;
                if (e2 > -dc) { err -= dc; r += sr; }
                if (e2 < dr) { err += dr; c += sc; }
            }
            return true;
        }

        function drawOdomMap(data) {
            lastOdomData = data;
            const W = odomCanvas.width, H = odomCanvas.height;
            odomCtx.clearRect(0, 0, W, H);

            // Determine auto-fit scale from trail + path + current position
            let maxR = 0.5;
            const allPts = (data.trail || []).concat(data.path || []);
            allPts.push([data.x, data.y]);
            allPts.push([0, 0]);
            for (const wp of odomWaypoints) allPts.push([wp.x, wp.y]);
            for (const pp of odomPlannedPath) allPts.push([pp.x, pp.y]);
            for (const p of allPts) {
                const ax = Math.abs(p[0]), ay = Math.abs(p[1]);
                if (ax > maxR) maxR = ax;
                if (ay > maxR) maxR = ay;
            }
            maxR *= 1.2;
            lastOdomMaxR = maxR;

            const autoScale = (Math.min(W, H) / 2 - 16) / maxR;
            lastOdomScale = autoScale;
            const scale = autoScale * odomZoom;

            const cx = W / 2 + odomPanX;
            const cy = H / 2 + odomPanY;

            function toCanvas(mx, my) {
                return [cx - my * scale, cy - mx * scale];
            }

            // ── SLAM occupancy grid overlay ──────────────────────────
            if (slamOverlayOn && lastMapData && lastMapData.available && lastMapBytes) {
                const mw = lastMapData.width, mh = lastMapData.height;
                const mres = lastMapData.resolution;
                const ox = lastMapData.origin_x, oy = lastMapData.origin_y;
                const cellPx = Math.max(1, mres * scale);
                let mappedCells = 0;

                for (let row = 0; row < mh; row++) {
                    for (let col = 0; col < mw; col++) {
                        const v = lastMapBytes[row * mw + col];
                        if (v === 255) continue;

                        const wx = ox + col * mres;
                        const wy = oy + row * mres;
                        const pcx = cx - wy * scale;
                        const pcy = cy - wx * scale;

                        if (pcx < -cellPx || pcx > W || pcy < -cellPx || pcy > H) continue;

                        const occ = v / 100.0;
                        const gray = Math.round(230 * (1 - occ));
                        odomCtx.fillStyle = 'rgba(' + gray + ',' + gray + ',' + gray + ',0.65)';
                        odomCtx.fillRect(pcx - cellPx / 2, pcy - cellPx / 2, cellPx, cellPx);
                        mappedCells++;
                    }
                }
                slamCellsEl.textContent = mappedCells;
            }

            // Grid lines
            const visibleRange = maxR / odomZoom * 2;
            let gridStep = 0.5;
            if (visibleRange > 10) gridStep = 2.0;
            else if (visibleRange > 4) gridStep = 1.0;

            odomCtx.strokeStyle = '#e8e8e8';
            odomCtx.lineWidth = 0.5;
            odomCtx.font = '9px ui-monospace, monospace';
            odomCtx.fillStyle = '#bbb';
            odomCtx.textAlign = 'left';
            const gridMax = maxR * Math.max(3, 1 / odomZoom + 1);
            for (let g = gridStep; g <= gridMax; g += gridStep) {
                const [, gy1] = toCanvas(g, 0);
                if (gy1 > 0 && gy1 < H) {
                    odomCtx.beginPath(); odomCtx.moveTo(0, gy1); odomCtx.lineTo(W, gy1); odomCtx.stroke();
                    odomCtx.fillText(g.toFixed(1) + 'm', 3, gy1 - 2);
                }
                const [, gy2] = toCanvas(-g, 0);
                if (gy2 > 0 && gy2 < H) {
                    odomCtx.beginPath(); odomCtx.moveTo(0, gy2); odomCtx.lineTo(W, gy2); odomCtx.stroke();
                }
                const [gx1] = toCanvas(0, g);
                if (gx1 > 0 && gx1 < W) {
                    odomCtx.beginPath(); odomCtx.moveTo(gx1, 0); odomCtx.lineTo(gx1, H); odomCtx.stroke();
                }
                const [gx2] = toCanvas(0, -g);
                if (gx2 > 0 && gx2 < W) {
                    odomCtx.beginPath(); odomCtx.moveTo(gx2, 0); odomCtx.lineTo(gx2, H); odomCtx.stroke();
                }
            }

            // Axes through origin
            odomCtx.strokeStyle = '#ccc';
            odomCtx.lineWidth = 1;
            const [axOx, axOy] = toCanvas(0, 0);
            odomCtx.beginPath(); odomCtx.moveTo(axOx, 0); odomCtx.lineTo(axOx, H); odomCtx.stroke();
            odomCtx.beginPath(); odomCtx.moveTo(0, axOy); odomCtx.lineTo(W, axOy); odomCtx.stroke();

            // Axis labels
            odomCtx.fillStyle = '#999';
            odomCtx.font = '10px ui-monospace, monospace';
            odomCtx.textAlign = 'center';
            if (axOy > 5 && axOy < H) odomCtx.fillText('+X (fwd)', axOx, Math.max(12, axOy - H/2 + 12));
            odomCtx.textAlign = 'right';

            // Origin marker (orange dot)
            odomCtx.beginPath();
            odomCtx.arc(axOx, axOy, 5, 0, 2 * Math.PI);
            odomCtx.fillStyle = '#fa0';
            odomCtx.fill();
            odomCtx.strokeStyle = '#c80';
            odomCtx.lineWidth = 1.5;
            odomCtx.stroke();

            // Recorded path (green, thicker)
            const path = data.path || [];
            if (path.length > 1) {
                odomCtx.beginPath();
                let [px0, py0] = toCanvas(path[0][0], path[0][1]);
                odomCtx.moveTo(px0, py0);
                for (let i = 1; i < path.length; i++) {
                    const [px, py] = toCanvas(path[i][0], path[i][1]);
                    odomCtx.lineTo(px, py);
                }
                odomCtx.strokeStyle = 'rgba(0, 170, 0, 0.8)';
                odomCtx.lineWidth = 3;
                odomCtx.stroke();
                if (path.length >= 2) {
                    const [sx, sy] = toCanvas(path[0][0], path[0][1]);
                    odomCtx.beginPath(); odomCtx.arc(sx, sy, 3, 0, 2*Math.PI);
                    odomCtx.fillStyle = '#0a0'; odomCtx.fill();
                    const last = path[path.length - 1];
                    const [ex, ey] = toCanvas(last[0], last[1]);
                    odomCtx.beginPath(); odomCtx.arc(ex, ey, 3, 0, 2*Math.PI);
                    odomCtx.fillStyle = '#070'; odomCtx.fill();
                }
            }

            // Live trail (blue)
            const trail = data.trail || [];
            if (trail.length > 1) {
                odomCtx.beginPath();
                let [tx0, ty0] = toCanvas(trail[0][0], trail[0][1]);
                odomCtx.moveTo(tx0, ty0);
                for (let i = 1; i < trail.length; i++) {
                    const [tx, ty] = toCanvas(trail[i][0], trail[i][1]);
                    odomCtx.lineTo(tx, ty);
                }
                odomCtx.strokeStyle = 'rgba(0, 128, 255, 0.6)';
                odomCtx.lineWidth = 1.5;
                odomCtx.stroke();
            }

            // ── A* planned path overlay (orange dashed) ─────────────
            if (odomPlannedPath.length > 1) {
                odomCtx.beginPath();
                let [px0, py0] = toCanvas(odomPlannedPath[0].x, odomPlannedPath[0].y);
                odomCtx.moveTo(px0, py0);
                for (let i = 1; i < odomPlannedPath.length; i++) {
                    const [px, py] = toCanvas(odomPlannedPath[i].x, odomPlannedPath[i].y);
                    odomCtx.lineTo(px, py);
                }
                odomCtx.setLineDash([6, 4]);
                odomCtx.strokeStyle = 'rgba(255, 136, 0, 0.7)';
                odomCtx.lineWidth = 2.5;
                odomCtx.stroke();
                odomCtx.setLineDash([]);
            }

            // ── Waypoint connecting lines (dashed magenta) ──────────
            if (odomWaypoints.length > 1) {
                odomCtx.beginPath();
                let [lx0, ly0] = toCanvas(odomWaypoints[0].x, odomWaypoints[0].y);
                odomCtx.moveTo(lx0, ly0);
                for (let i = 1; i < odomWaypoints.length; i++) {
                    const [lx, ly] = toCanvas(odomWaypoints[i].x, odomWaypoints[i].y);
                    odomCtx.lineTo(lx, ly);
                }
                odomCtx.setLineDash([4, 4]);
                odomCtx.strokeStyle = 'rgba(200, 0, 200, 0.35)';
                odomCtx.lineWidth = 1.5;
                odomCtx.stroke();
                odomCtx.setLineDash([]);
            }

            // ── Numbered waypoint markers ────────────────────────────
            for (let wi = 0; wi < odomWaypoints.length; wi++) {
                const wp = odomWaypoints[wi];
                const [wpx, wpy] = toCanvas(wp.x, wp.y);
                const r = 10;
                // Circle background
                odomCtx.beginPath();
                odomCtx.arc(wpx, wpy, r, 0, 2 * Math.PI);
                odomCtx.fillStyle = 'rgba(255, 0, 255, 0.5)';
                odomCtx.fill();
                odomCtx.strokeStyle = '#f0f';
                odomCtx.lineWidth = 2;
                odomCtx.stroke();
                // Number label
                odomCtx.fillStyle = '#fff';
                odomCtx.font = 'bold 10px ui-monospace, monospace';
                odomCtx.textAlign = 'center';
                odomCtx.textBaseline = 'middle';
                odomCtx.fillText(String(wi + 1), wpx, wpy);
                // Coordinate below
                odomCtx.fillStyle = '#c0c';
                odomCtx.font = '9px ui-monospace, monospace';
                odomCtx.textBaseline = 'top';
                odomCtx.fillText('(' + wp.x.toFixed(1) + ',' + wp.y.toFixed(1) + ')', wpx, wpy + r + 2);
            }
            odomCtx.textBaseline = 'alphabetic'; // reset

            // Robot position (red dot) + heading arrow
            const [rx, ry] = toCanvas(data.x, data.y);
            const yawRad = (data.yaw_deg || 0) * Math.PI / 180;
            const arrowLen = 18;
            const tipCx = rx - Math.sin(yawRad) * arrowLen;
            const tipCy = ry - Math.cos(yawRad) * arrowLen;

            odomCtx.beginPath();
            odomCtx.moveTo(rx, ry);
            odomCtx.lineTo(tipCx, tipCy);
            odomCtx.strokeStyle = '#e00';
            odomCtx.lineWidth = 2.5;
            odomCtx.lineCap = 'round';
            odomCtx.stroke();
            const aAngle = Math.atan2(tipCy - ry, tipCx - rx);
            odomCtx.beginPath();
            odomCtx.moveTo(tipCx, tipCy);
            odomCtx.lineTo(tipCx - 7*Math.cos(aAngle - 0.45), tipCy - 7*Math.sin(aAngle - 0.45));
            odomCtx.moveTo(tipCx, tipCy);
            odomCtx.lineTo(tipCx - 7*Math.cos(aAngle + 0.45), tipCy - 7*Math.sin(aAngle + 0.45));
            odomCtx.stroke();
            odomCtx.lineCap = 'butt';

            odomCtx.beginPath();
            odomCtx.arc(rx, ry, 5, 0, 2 * Math.PI);
            odomCtx.fillStyle = '#e00';
            odomCtx.fill();

            // Zoom indicator
            odomCtx.fillStyle = 'rgba(0,0,0,0.35)';
            odomCtx.font = '10px ui-monospace, monospace';
            odomCtx.textAlign = 'right';
            odomCtx.fillText('zoom: ' + odomZoom.toFixed(1) + 'x', W - 6, H - 6);
        }

        function updateOdomUI(d) {
            odomX.textContent = d.x != null ? d.x.toFixed(3) : '\\u2014';
            odomY.textContent = d.y != null ? d.y.toFixed(3) : '\\u2014';
            odomYaw.textContent = d.yaw_deg != null ? d.yaw_deg.toFixed(1) : '\\u2014';
            const spd = Math.sqrt((d.vx||0)*(d.vx||0) + (d.vy||0)*(d.vy||0));
            odomSpeed.textContent = spd.toFixed(3) + ' m/s';

            if (odomPrevX !== null && odomPrevY !== null) {
                const dx = (d.x||0) - odomPrevX;
                const dy = (d.y||0) - odomPrevY;
                const seg = Math.sqrt(dx*dx + dy*dy);
                if (seg < 0.5) odomTotalDist += seg;
            }
            odomPrevX = d.x || 0;
            odomPrevY = d.y || 0;
            odomDist.textContent = odomTotalDist.toFixed(3) + ' m';

            odomWpCount.textContent = String((d.path || []).length);
            odomTrailCount.textContent = String((d.trail || []).length);

            // Stuck detection banner
            if (d.stuck) {
                odomStuckBanner.style.display = '';
            } else {
                odomStuckBanner.style.display = 'none';
            }

            odomRecording = !!d.recording;
            odomReturning = !!d.returning;
            odomNavigating = !!d.navigating;

            if (odomNavigating) {
                const wpIdx = d.nav_wp_idx || 0;
                const wpTotal = d.nav_wp_total || 0;
                const progress = wpTotal > 0 ? (' ' + wpIdx + '/' + wpTotal) : '';
                odomStatus.textContent = '\\u27a1 NAVIGATING' + progress;
                odomStatus.style.color = '#a0a';
                odomNavCancelBtn.style.display = '';
                odomNavBtn.style.display = 'none';
            } else {
                odomNavCancelBtn.style.display = 'none';
                if (odomWaypoints.length > 0) odomNavBtn.style.display = '';
            }

            if (odomReturning) {
                odomStatus.textContent = '\\u21a9 RETURNING TO ORIGIN';
                odomStatus.style.color = '#05a';
                odomCancelBtn.style.display = '';
                odomReturnBtn.style.display = 'none';
            } else {
                odomCancelBtn.style.display = 'none';
                odomReturnBtn.style.display = '';
                if (odomRecording) {
                    odomStatus.textContent = '\\u23fa RECORDING';
                    odomStatus.style.color = '#b00';
                    odomStartRecBtn.style.display = 'none';
                    odomStopRecBtn.style.display = '';
                } else if (!odomNavigating) {
                    odomStartRecBtn.style.display = '';
                    odomStopRecBtn.style.display = 'none';
                    if (d.last_age_s != null && d.last_age_s < 2) {
                        odomStatus.textContent = 'active';
                        odomStatus.style.color = '#080';
                    } else {
                        odomStatus.textContent = 'waiting for data\\u2026';
                        odomStatus.style.color = '#888';
                    }
                }
            }

            drawOdomMap(d);
        }

        async function fetchOdom() {
            try {
                const r = await guardedFetch('/api/odom', {cache: 'no-store'});
                if (!r.ok) return;
                const d = await r.json();
                updateOdomUI(d);
            } catch(e) {}
        }

        polled(fetchOdom, 500);

        const allOdomBtns = [odomSetOriginBtn, odomStartRecBtn, odomStopRecBtn,
            odomReturnBtn, odomCancelBtn, odomNavBtn, odomNavCancelBtn, odomClearWpBtn, odomUndoWpBtn];

        async function odomServiceCall(endpoint) {
            if (odomBusy) return;
            odomBusy = true;
            allOdomBtns.forEach(b => b.disabled = true);
            odomStatus.textContent = '\\u231b working\\u2026';
            odomStatus.style.color = '#888';
            try {
                const r = await fetch(endpoint, {method: 'POST'});
                const d = await r.json();
                if (d.message) {
                    odomStatus.textContent = d.message;
                    odomStatus.style.color = d.ok ? '#080' : '#b00';
                }
            } catch(e) {
                odomStatus.textContent = 'error';
                odomStatus.style.color = '#b00';
            } finally {
                odomBusy = false;
                allOdomBtns.forEach(b => b.disabled = false);
                await fetchOdom();
            }
        }

        async function navigatePath() {
            if (odomWaypoints.length === 0 || odomBusy) return;
            // Replan to get latest A* path, then send the planned waypoints
            replanPath();
            const waypoints = odomPlannedPath.length > 1 ? odomPlannedPath : odomWaypoints;
            odomBusy = true;
            allOdomBtns.forEach(b => b.disabled = true);
            odomStatus.textContent = '\\u27a1 NAVIGATING ' + odomWaypoints.length + ' waypoints\\u2026';
            odomStatus.style.color = '#a0a';
            try {
                const r = await fetch('/api/odom/navigate_path', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({waypoints: waypoints.map(w => ({x: w.x, y: w.y}))}),
                });
                const d = await r.json();
                if (d.message) {
                    odomStatus.textContent = d.message;
                    odomStatus.style.color = d.ok ? '#080' : '#b00';
                }
            } catch(e) {
                odomStatus.textContent = 'error';
                odomStatus.style.color = '#b00';
            } finally {
                odomBusy = false;
                allOdomBtns.forEach(b => b.disabled = false);
                await fetchOdom();
            }
        }

        odomSetOriginBtn.addEventListener('click', async () => {
            odomTotalDist = 0.0;
            odomPrevX = null;
            odomPrevY = null;
            odomWaypoints = [];
            odomPlannedPath = [];
            odomWpTarget.textContent = 'idle';
            odomNavBtn.style.display = 'none';
            odomClearWpBtn.style.display = 'none';
            odomUndoWpBtn.style.display = 'none';
            odomPanX = 0; odomPanY = 0; odomZoom = 1.0; odomUserZoom = false;
            odomStatus.textContent = '\\u231b resetting\\u2026';
            odomStatus.style.color = '#888';
            await odomServiceCall('/api/odom/set_origin');
        });

        odomStartRecBtn.addEventListener('click', async () => {
            odomStartRecBtn.style.display = 'none';
            odomStopRecBtn.style.display = '';
            odomStatus.textContent = '\\u23fa RECORDING';
            odomStatus.style.color = '#b00';
            await odomServiceCall('/api/odom/start_recording');
        });
        odomStopRecBtn.addEventListener('click', async () => {
            odomStopRecBtn.style.display = 'none';
            odomStartRecBtn.style.display = '';
            await odomServiceCall('/api/odom/stop_recording');
        });
        odomReturnBtn.addEventListener('click', async () => {
            odomReturnBtn.style.display = 'none';
            odomCancelBtn.style.display = '';
            odomStatus.textContent = '\\u21a9 RETURNING TO ORIGIN';
            odomStatus.style.color = '#05a';
            await odomServiceCall('/api/odom/return_to_origin');
        });
        odomCancelBtn.addEventListener('click', async () => {
            odomCancelBtn.style.display = 'none';
            odomReturnBtn.style.display = '';
            await odomServiceCall('/api/odom/cancel_return');
        });
        odomNavBtn.addEventListener('click', navigatePath);
        odomNavCancelBtn.addEventListener('click', async () => {
            odomNavCancelBtn.style.display = 'none';
            if (odomWaypoints.length > 0) odomNavBtn.style.display = '';
            await odomServiceCall('/api/odom/cancel_navigate');
        });

        // ── SLAM map polling (1 Hz) ──────────────────────────────────
        async function fetchMap() {
            try {
                const r = await guardedFetch('/api/map', {cache: 'no-store'}, 5000);
                if (!r.ok) return;
                const d = await r.json();
                if (d.available) {
                    lastMapData = d;
                    // Decode base64 to Uint8Array
                    const raw = atob(d.data);
                    const arr = new Uint8Array(raw.length);
                    for (let i = 0; i < raw.length; i++) arr[i] = raw.charCodeAt(i);
                    lastMapBytes = arr;
                    slamStatusEl.textContent = d.width + '\\u00d7' + d.height + ' (' + d.age_s + 's)';
                    slamStatusEl.style.color = '#080';
                } else {
                    slamStatusEl.textContent = 'no data';
                    slamStatusEl.style.color = '#888';
                }
            } catch(e) {
                slamStatusEl.textContent = 'error';
                slamStatusEl.style.color = '#b00';
            }
        }
        polled(fetchMap, 1000);

        slamResetBtn.addEventListener('click', async () => {
            slamResetBtn.disabled = true;
            slamStatusEl.textContent = '\\u231b resetting\\u2026';
            try {
                await fetch('/api/slam/reset', {method: 'POST'});
                lastMapData = null;
                lastMapBytes = null;
                slamCellsEl.textContent = '0';
            } catch(e) {}
            slamResetBtn.disabled = false;
            await fetchMap();
        });

        // ── Front calibration handlers ────────────────────────────────
        async function runCalibration(endpoint) {
            calStaticBtn.disabled = true;
            calFullBtn.disabled = true;
            calStatusEl.textContent = '\u23f3 calibrating\u2026';
            calStatusEl.style.color = '#c60';
            calResultEl.style.display = 'none';
            try {
                const resp = await fetch(endpoint, {method: 'POST'});
                const data = await resp.json();
                calStatusEl.textContent = data.success ? '\u2705 done' : '\u274c failed';
                calStatusEl.style.color = data.success ? '#0a0' : '#b00';
                let html = data.message || 'No response';
                try {
                    const msg = JSON.parse(data.message);
                    if (msg.offset_deg !== undefined) {
                        html = 'Offset: <b>' + msg.offset_deg.toFixed(1) + '\u00b0</b>';
                        if (msg.confidence) html += ' (confidence: ' + msg.confidence + ')';
                        if (msg.recommendation) html += '<br>' + msg.recommendation;
                    }
                } catch(_) {}
                calResultEl.innerHTML = html;
                calResultEl.style.display = 'block';
            } catch(e) {
                calStatusEl.textContent = 'error';
                calStatusEl.style.color = '#b00';
            }
            calStaticBtn.disabled = false;
            calFullBtn.disabled = false;
        }
        calStaticBtn.addEventListener('click', () => runCalibration('/api/calibrate_front_static'));
        calFullBtn.addEventListener('click', () => runCalibration('/api/calibrate_front'));

        // ──────────────────────────────────────────────────────────────
        // LiDAR Scan Visualisation
        // ──────────────────────────────────────────────────────────────
        const lidarCanvas = document.getElementById('lidarCanvas');
        const lidarCtx = lidarCanvas.getContext('2d');
        const lidarPointsEl = document.getElementById('lidarPoints');
        const lidarRangeEl = document.getElementById('lidarRange');
        const lidarFovEl = document.getElementById('lidarFov');
        const lidarStatusEl = document.getElementById('lidarStatus');
        const lidarZoomSlider = document.getElementById('lidarZoom');
        const lidarZoomVal = document.getElementById('lidarZoomVal');
        let lidarManualRange = 0; // 0 = auto
        let lidarSmoothedRange = 0; // EMA-smoothed auto range (0 = uninitialised)

        lidarZoomSlider.addEventListener('input', () => {
            lidarManualRange = parseFloat(lidarZoomSlider.value);
            lidarZoomVal.textContent = lidarManualRange > 0 ? lidarManualRange.toFixed(1) + ' m' : 'auto';
            if (lidarManualRange > 0) lidarSmoothedRange = 0; // reset when switching to manual
        });

        function drawLidarScan(data) {
            const W = lidarCanvas.width, H = lidarCanvas.height;
            const cx = W / 2, cy = H / 2;
            lidarCtx.clearRect(0, 0, W, H);
            lidarCtx.fillStyle = '#111';
            lidarCtx.fillRect(0, 0, W, H);

            const angles = data.angles || [];
            const ranges = data.ranges || [];
            const rangeMin = data.range_min || 0.05;
            const rangeMax = data.range_max || 12.0;
            const nPts = Math.min(angles.length, ranges.length);

            // Determine display range (max visible distance)
            let maxR = 1.0;
            if (lidarManualRange > 0) {
                maxR = lidarManualRange;
            } else {
                // Auto: find the 95th-percentile valid range for a nice view
                const valid = [];
                for (let i = 0; i < nPts; i++) {
                    const r = ranges[i];
                    if (r >= rangeMin && r <= rangeMax) valid.push(r);
                }
                let rawR = 1.0;
                if (valid.length > 0) {
                    valid.sort((a, b) => a - b);
                    rawR = valid[Math.floor(valid.length * 0.95)] * 1.15;
                    if (rawR < 0.5) rawR = 0.5;
                }
                // Smooth via EMA to prevent jittery zoom on noisy scans
                if (lidarSmoothedRange <= 0) {
                    lidarSmoothedRange = rawR; // first frame — snap
                } else {
                    const alpha = 0.08; // low alpha = smoother (0.05–0.15 is good)
                    lidarSmoothedRange += alpha * (rawR - lidarSmoothedRange);
                }
                maxR = lidarSmoothedRange;
            }
            const scale = (Math.min(W, H) / 2 - 20) / maxR;

            // Coordinate transform: LiDAR frame → canvas
            // +X (forward) = up on canvas, angles CCW from +X
            function toCanvas(dist, angleRad) {
                const lx = dist * Math.cos(angleRad);
                const ly = dist * Math.sin(angleRad);
                return [cx - ly * scale, cy - lx * scale];
            }

            // Range rings
            let ringStep = 1.0;
            if (maxR <= 2) ringStep = 0.5;
            else if (maxR <= 5) ringStep = 1.0;
            else if (maxR <= 10) ringStep = 2.0;
            else ringStep = 3.0;

            lidarCtx.strokeStyle = 'rgba(255,255,255,0.1)';
            lidarCtx.lineWidth = 0.5;
            lidarCtx.font = '10px ui-monospace, monospace';
            lidarCtx.fillStyle = 'rgba(255,255,255,0.3)';
            lidarCtx.textAlign = 'left';
            for (let r = ringStep; r <= maxR; r += ringStep) {
                const rPx = r * scale;
                lidarCtx.beginPath();
                lidarCtx.arc(cx, cy, rPx, 0, 2 * Math.PI);
                lidarCtx.stroke();
                lidarCtx.fillText(r.toFixed(1) + 'm', cx + 3, cy - rPx + 12);
            }

            // Cross-hair axes
            lidarCtx.strokeStyle = 'rgba(255,255,255,0.15)';
            lidarCtx.lineWidth = 0.5;
            lidarCtx.beginPath(); lidarCtx.moveTo(cx, 0); lidarCtx.lineTo(cx, H); lidarCtx.stroke();
            lidarCtx.beginPath(); lidarCtx.moveTo(0, cy); lidarCtx.lineTo(W, cy); lidarCtx.stroke();

            // Cardinal labels
            lidarCtx.fillStyle = 'rgba(255,255,255,0.4)';
            lidarCtx.font = '11px ui-monospace, monospace';
            lidarCtx.textAlign = 'center';
            lidarCtx.fillText('FWD', cx, 14);
            lidarCtx.fillText('BACK', cx, H - 6);
            lidarCtx.textAlign = 'right';
            lidarCtx.fillText('RIGHT', W - 6, cy - 4);
            lidarCtx.textAlign = 'left';
            lidarCtx.fillText('LEFT', 6, cy - 4);

            // Draw scan points with distance-based colouring
            for (let i = 0; i < nPts; i++) {
                const r = ranges[i];
                if (r < rangeMin || r > rangeMax) continue;
                const a = angles[i];
                const [px, py] = toCanvas(r, a);
                // Colour: green (close) → yellow → red (far)
                const t = Math.min(1.0, r / maxR);
                const red = Math.floor(255 * t);
                const grn = Math.floor(255 * (1 - t * 0.5));
                lidarCtx.fillStyle = 'rgb(' + red + ',' + grn + ',0)';
                lidarCtx.fillRect(px - 1.5, py - 1.5, 3, 3);
            }

            // Robot marker at centre (red circle with heading line)
            lidarCtx.beginPath();
            lidarCtx.arc(cx, cy, 5, 0, 2 * Math.PI);
            lidarCtx.fillStyle = '#e00';
            lidarCtx.fill();
            lidarCtx.strokeStyle = '#e00';
            lidarCtx.lineWidth = 2;
            lidarCtx.beginPath();
            lidarCtx.moveTo(cx, cy);
            lidarCtx.lineTo(cx, cy - 14);  // forward arrow up
            lidarCtx.stroke();
            // Arrowhead
            lidarCtx.beginPath();
            lidarCtx.moveTo(cx, cy - 14);
            lidarCtx.lineTo(cx - 4, cy - 9);
            lidarCtx.moveTo(cx, cy - 14);
            lidarCtx.lineTo(cx + 4, cy - 9);
            lidarCtx.stroke();
        }

        function updateLidarUI(d) {
            const nPts = (d.angles || []).length;
            lidarPointsEl.textContent = String(nPts);
            if (d.range_max != null) {
                lidarRangeEl.textContent = (d.range_min || 0).toFixed(2) + ' – ' + d.range_max.toFixed(1) + ' m';
            }
            if (d.angle_min != null && d.angle_max != null) {
                const fov = ((d.angle_max - d.angle_min) * 180 / Math.PI).toFixed(0);
                lidarFovEl.textContent = fov + '°';
            }
            if (d.last_age_s != null && d.last_age_s < 2) {
                lidarStatusEl.textContent = 'active (' + nPts + ' pts)';
                lidarStatusEl.style.color = '#0f0';
            } else {
                lidarStatusEl.textContent = 'waiting for data…';
                lidarStatusEl.style.color = '#888';
            }
            drawLidarScan(d);
        }

        async function fetchLidar() {
            try {
                const r = await guardedFetch('/api/lidar', {cache: 'no-store'});
                if (!r.ok) return;
                const d = await r.json();
                updateLidarUI(d);
            } catch(e) {}
        }

        polled(fetchLidar, 500);

        // ── LiDAR obstacle zone polling (5 Hz alongside lidar scan) ─
        const ozFrontEl = document.getElementById('ozFront');
        const ozLeftEl = document.getElementById('ozLeft');
        const ozRightEl = document.getElementById('ozRight');
        const ozRearEl = document.getElementById('ozRear');

        function formatZone(val) {
            if (val >= 90) return '\\u2014 (clear)';
            const color = val < 0.2 ? '#d00' : val < 0.5 ? '#c80' : '#080';
            return '<span style=\"color:' + color + ';font-weight:bold;\">' + val.toFixed(2) + ' m</span>';
        }

        async function fetchLidarZones() {
            try {
                const r = await guardedFetch('/api/lidar_zones', {cache: 'no-store'});
                if (!r.ok) return;
                const d = await r.json();
                ozFrontEl.innerHTML = formatZone(d.front || 99);
                ozLeftEl.innerHTML = formatZone(d.left || 99);
                ozRightEl.innerHTML = formatZone(d.right || 99);
                ozRearEl.innerHTML = formatZone(d.rear || 99);
            } catch(e) {}
        }
        polled(fetchLidarZones, 500);

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

        // ── Face Recognition UI ─────────────────────────────────
        const faceListEl = document.getElementById('faceList');
        const faceCountEl = document.getElementById('faceCount');
        const faceRefreshBtn = document.getElementById('faceRefreshBtn');
        const faceClearBtn = document.getElementById('faceClearBtn');
        const faceMergePanel = document.getElementById('faceMergePanel');
        const faceMergeBtn = document.getElementById('faceMergeBtn');
        const faceKeepId = document.getElementById('faceKeepId');
        const faceMergeId = document.getElementById('faceMergeId');

        async function loadFaces() {
            try {
                const r = await guardedFetch('/api/faces', {cache: 'no-store'});
                const j = await r.json();
                const faces = j.faces || [];
                faceCountEl.textContent = String(faces.length);
                faceListEl.innerHTML = '';
                faceMergePanel.style.display = faces.length >= 2 ? '' : 'none';
                for (const f of faces) {
                    const card = document.createElement('div');
                    card.style.cssText = 'border:1px solid #dde0e4;border-radius:8px;padding:8px;background:#fafafa;';
                    const thumbUrl = '/api/faces/thumbnail?id=' + f.face_id;
                    card.innerHTML = `
                        <img src="${thumbUrl}" alt="face" style="width:100%;height:80px;object-fit:cover;border-radius:4px;background:#eee;" onerror="this.style.background='#ddd';this.alt='No photo';" />
                        <div style="margin-top:4px;font-size:12px;">
                            <strong>ID ${f.face_id}</strong> &middot; ${f.num_embeddings} emb
                        </div>
                        <input type="text" value="${(f.name||'').replace(/"/g,'&quot;')}" data-fid="${f.face_id}"
                            style="width:100%;margin-top:4px;padding:3px 6px;border:1px solid #ccc;border-radius:4px;font-size:12px;"
                            placeholder="Name" />
                        <div style="display:flex;gap:4px;margin-top:4px;">
                            <button class="faceRenameBtn primary" data-fid="${f.face_id}" style="flex:1;font-size:11px;padding:3px 6px;">Save</button>
                            <button class="faceDeleteBtn danger" data-fid="${f.face_id}" style="flex:1;font-size:11px;padding:3px 6px;">Delete</button>
                        </div>`;
                    faceListEl.appendChild(card);
                }
                // Bind rename buttons
                for (const btn of faceListEl.querySelectorAll('.faceRenameBtn')) {
                    btn.addEventListener('click', async () => {
                        const fid = Number(btn.dataset.fid);
                        const inp = faceListEl.querySelector(`input[data-fid="${fid}"]`);
                        if (!inp) return;
                        await fetch('/api/faces/rename', {
                            method: 'POST',
                            headers: {'Content-Type': 'application/json'},
                            body: JSON.stringify({face_id: fid, name: inp.value})
                        });
                        loadFaces();
                    });
                }
                // Bind delete buttons
                for (const btn of faceListEl.querySelectorAll('.faceDeleteBtn')) {
                    btn.addEventListener('click', async () => {
                        const fid = Number(btn.dataset.fid);
                        if (!confirm(`Delete face ID ${fid}?`)) return;
                        await fetch('/api/faces/delete', {
                            method: 'POST',
                            headers: {'Content-Type': 'application/json'},
                            body: JSON.stringify({face_id: fid})
                        });
                        loadFaces();
                    });
                }
            } catch (e) {
                faceCountEl.textContent = 'error';
            }
        }

        faceRefreshBtn.addEventListener('click', loadFaces);
        faceClearBtn.addEventListener('click', async () => {
            if (!confirm('Delete ALL enrolled faces?')) return;
            await fetch('/api/faces/clear', {method: 'POST'});
            loadFaces();
        });
        faceMergeBtn.addEventListener('click', async () => {
            const kid = Number(faceKeepId.value);
            const mid = Number(faceMergeId.value);
            if (!kid || !mid || kid === mid) { alert('Enter two different face IDs.'); return; }
            await fetch('/api/faces/merge', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({keep_id: kid, merge_id: mid})
            });
            loadFaces();
        });
        // Auto-load on page open, then poll every 10s
        polled(loadFaces, 10000);

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
        angles = [msg.angle_min + i * msg.angle_increment for i in range(n)]
        self._scan_angles = angles
        self._scan_ranges = list(msg.ranges)
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
        # Round values to reduce JSON payload size (~430 points per scan)
        angles = [round(a, 4) for a in self._scan_angles]
        ranges = [round(r, 4) for r in self._scan_ranges]
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
        self._map_data = list(msg.data)
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
        if not self._map_data or self._map_last_monotonic == 0.0:
            return {'available': False}

        # Only re-encode when map data changes
        if self._map_dirty:
            import base64
            data_bytes = bytes(
                255 if v < 0 else min(v, 100) for v in self._map_data
            )
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
    logger,
    status_provider=None,
    detections_provider=None,
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
                    self.connection.settimeout(10.0)
                except Exception:
                    pass

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
                except (BrokenPipeError, ConnectionResetError, TimeoutError):
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
                except (BrokenPipeError, ConnectionResetError, TimeoutError):
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
                    self.connection.settimeout(10.0)
                except Exception:
                    pass

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
                except (BrokenPipeError, ConnectionResetError, TimeoutError):
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
                except (BrokenPipeError, ConnectionResetError, TimeoutError):
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
                    self.connection.settimeout(10.0)
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
