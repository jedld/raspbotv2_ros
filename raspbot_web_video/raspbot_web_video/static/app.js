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

        function updateStatusUI(j) {
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
                if (typeof j.tracking.pan_sign === 'number') {
                    invPanChk.checked = (Number(j.tracking.pan_sign) < 0);
                }
                if (typeof j.tracking.tilt_sign === 'number') {
                    invTiltChk.checked = (Number(j.tracking.tilt_sign) < 0);
                }
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
        }

        function updateMapUI(d) {
            if (d.available) {
                lastMapData = d;
                const raw = atob(d.data);
                const arr = new Uint8Array(raw.length);
                for (let i = 0; i < raw.length; i++) arr[i] = raw.charCodeAt(i);
                lastMapBytes = arr;
                slamStatusEl.textContent = d.width + '\u00d7' + d.height + ' (' + d.age_s + 's)';
                slamStatusEl.style.color = '#080';
            } else {
                slamStatusEl.textContent = 'no data';
                slamStatusEl.style.color = '#888';
            }
        }

        function updateDetectionsUI(j) {
            latestDet = j;
            lastDetFetchMs = Date.now();
            const n = Array.isArray(j.detections) ? j.detections.length : 0;
            detStateEl.textContent = `n=${n}`;
        }

        async function fetchStatus() {
            try {
                const r = await guardedFetch('/status');
                const j = await r.json();
                updateStatusUI(j);
                if (j.map) updateMapUI(j.map);
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
            // Legacy stub — now handled by fetchFast()
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
        // Polling is replaced by WebSocket — see connectWebSocket() at end of file
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

        function updateImuUI(d) {
            imuData = d;
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

            const yaw = d.yaw_deg != null ? Number(d.yaw_deg) : 0;
            imuYawLabel.textContent = yaw.toFixed(1);

            drawRobotTopDown(d);
            drawCompass(yaw, d.rotate_target_deg);
            updateBnoCal(d);
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
        }

        // ── Combined fast endpoint: detections + imu (3 Hz) ──────────
        async function fetchFast() {
            try {
                const r = await guardedFetch('/api/fast', {cache: 'no-store'});
                if (!r.ok) return;
                const j = await r.json();

                // Update detections
                if (j.detections) {
                    latestDet = j.detections;
                    lastDetFetchMs = Date.now();
                    const n = Array.isArray(j.detections.detections) ? j.detections.detections.length : 0;
                    detStateEl.textContent = `n=${n}`;
                }

                // Update IMU
                if (j.imu) {
                    updateImuUI(j.imu);
                }
            } catch (e) {
                if (e.name !== 'AbortError') detStateEl.textContent = 'error';
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

        // fetchFast polling replaced by WebSocket

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
            // Legacy stub — now handled by fetchSensors()
        }

        // fetchOdom is now handled by fetchSensors (combined endpoint)

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

        // fetchMap is now handled by fetchStatus (map data included in /status response)

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

        // ── LiDAR obstacle zone helpers ─────────────────────────────
        const ozFrontEl = document.getElementById('ozFront');
        const ozLeftEl = document.getElementById('ozLeft');
        const ozRightEl = document.getElementById('ozRight');
        const ozRearEl = document.getElementById('ozRear');

        function formatZone(val) {
            if (val >= 90) return '\\u2014 (clear)';
            const color = val < 0.2 ? '#d00' : val < 0.5 ? '#c80' : '#080';
            return '<span style=\"color:' + color + ';font-weight:bold;\">' + val.toFixed(2) + ' m</span>';
        }

        function updateLidarZonesUI(d) {
            ozFrontEl.innerHTML = formatZone(d.front || 99);
            ozLeftEl.innerHTML = formatZone(d.left || 99);
            ozRightEl.innerHTML = formatZone(d.right || 99);
            ozRearEl.innerHTML = formatZone(d.rear || 99);
        }

        // ── Combined sensor endpoint: odom + lidar + zones (2 Hz) ───
        async function fetchSensors() {
            try {
                const r = await guardedFetch('/api/sensors', {cache: 'no-store'});
                if (!r.ok) return;
                const j = await r.json();

                if (j.odom) updateOdomUI(j.odom);
                if (j.lidar) updateLidarUI(j.lidar);
                if (j.lidar_zones) updateLidarZonesUI(j.lidar_zones);
            } catch(e) {}
        }
        // fetchSensors polling replaced by WebSocket

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
        // Auto-load on page open; subsequent updates via WebSocket
        loadFaces();

        function renderFaces(facesData) {
            const faces = facesData.faces || [];
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
        }

        // ── WebSocket ───────────────────────────────────────────────
        let _ws = null;
        let _wsTimer = null;
        const _wsIndicator = document.getElementById('status');

        function connectWebSocket() {
            if (_ws && (_ws.readyState === WebSocket.CONNECTING || _ws.readyState === WebSocket.OPEN)) return;
            const proto = location.protocol === 'https:' ? 'wss:' : 'ws:';
            _ws = new WebSocket(`${proto}//${location.host}/ws`);

            _ws.onopen = () => {
                console.log('[ws] connected');
            };

            _ws.onmessage = (ev) => {
                try {
                    const msg = JSON.parse(ev.data);
                    if (msg.detections) updateDetectionsUI(msg.detections);
                    if (msg.imu)         updateImuUI(msg.imu);
                    if (msg.odom)        updateOdomUI(msg.odom);
                    if (msg.lidar)       updateLidarUI(msg.lidar);
                    if (msg.lidar_zones) updateLidarZonesUI(msg.lidar_zones);
                    if (msg.status)      updateStatusUI(msg.status);
                    if (msg.map)         updateMapUI(msg.map);
                    if (msg.faces)       renderFaces(msg.faces);
                } catch (e) {
                    console.warn('[ws] parse error', e);
                }
            };

            _ws.onclose = () => {
                console.log('[ws] closed — reconnecting in 2s');
                _ws = null;
                clearTimeout(_wsTimer);
                _wsTimer = setTimeout(connectWebSocket, 2000);
            };

            _ws.onerror = () => {
                _ws.close();
            };
        }

        connectWebSocket();
