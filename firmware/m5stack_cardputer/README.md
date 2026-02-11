# M5Stack Cardputer — Raspbot V2 Teleop Controller

Turns the M5Stack Cardputer into a handheld controller for the Raspbot V2,
providing live camera view and keyboard-driven teleop over WiFi.

## Features

| Feature | Details |
|---|---|
| **Camera view** | Live MJPEG stream on the 240×135 display (front or gimbal cam) |
| **Keyboard teleop** | WASD = drive, QE = strafe, Space = stop |
| **Speed control** | `+`/`-` keys to adjust speed (25%/50%/75%/100%) |
| **Camera toggle** | `TAB` to switch between front (Pi Camera) and gimbal (USB webcam) |
| **HUD overlay** | Camera source, FPS, ultrasonic distance, collision status, speed |
| **Collision indicator** | Distance turns red/orange when close to obstacles |

## Hardware

- M5Stack Cardputer (ESP32-S3, 8 MB flash, WiFi, 240×135 TFT, 56-key KB)
- Same WiFi network as the Raspberry Pi

## Architecture

```
Cardputer (ESP32-S3)          Raspberry Pi 5
┌─────────────────┐           ┌──────────────────────────┐
│  Keyboard ──────┼── WiFi ──▸│ POST /api/cmd_vel        │──▸ cmd_vel topic
│                 │           │                          │
│  TFT Display ◂──┼── WiFi ──│ GET /stream_*_lo.mjpg    │◂── camera topics
│  (240×135)      │           │     (240×180, q=50)      │    (resized server-side)
│                 │           │                          │
│  HUD overlay ◂──┼── WiFi ──│ GET /status               │◂── ultrasonic etc.
└─────────────────┘           └──────────────────────────┘
```

The Pi's web server provides **low-resolution MJPEG streams** (`/stream_front_lo.mjpg`
and `/stream_lo.mjpg`) that resize the camera frames server-side to 240×180 at JPEG
quality 50. This minimises WiFi bandwidth (~3-5 KB/frame vs 20-40 KB full-res) and
decode time on the ESP32.

No Bluetooth needed — WiFi reuses the existing web server endpoints.

## Quick Start

### 1. Install PlatformIO

```bash
pip install platformio
# or install the PlatformIO VS Code extension
```

### 2. Build & Flash

Connect the Cardputer via USB-C. If it doesn't show up, put it in download
mode: switch power OFF → hold G0 button → switch power ON → release G0.

```bash
cd firmware/m5stack_cardputer
pio run -t upload
```

### 3. First-boot WiFi Setup Wizard

On first power-on (or whenever NVS is empty) the Cardputer automatically
launches an interactive setup wizard:

1. **Scan** — Scans for nearby WiFi networks
2. **Select** — Scroll with `[` / `]` (or `,` / `.`), press **Enter** to pick
3. **Password** — Type the WiFi password on the keyboard, **Enter** to confirm
4. **Connect** — Attempts to join the network; retries the wizard if it fails
5. **Raspbot IP** — Enter the Pi’s IP address (pre-filled with `192.168.1.100`)
6. **Port** — Confirm or change the port (default `8080`)

All settings are saved to flash (NVS) and reused on subsequent boots.

> **To re-run the wizard later**: hold the **G0** button while powering on.

### 4. (Optional) Hard-code defaults

If you prefer, you can set fallback defaults in `src/config.h`.
These are only used when NVS is empty:

```c
static const char* DEFAULT_WIFI_SSID = "";
static const char* DEFAULT_WIFI_PASS = "";
static const char* DEFAULT_RASPBOT_HOST = "192.168.1.100";
static constexpr int DEFAULT_RASPBOT_PORT = 8080;
```

### 5. Monitor serial output (optional)

```bash
pio device monitor -b 115200
```

## Keyboard Map

### Teleop (main screen)

| Key | Action |
|---|---|
| `W` | Drive forward |
| `S` | Drive backward |
| `A` | Turn left |
| `D` | Turn right |
| `Q` | Strafe left (mecanum) |
| `E` | Strafe right (mecanum) |
| `Space` | Emergency stop |
| `TAB` | Toggle front/gimbal camera |
| `+` / `=` | Increase speed (25% steps) |
| `-` / `_` | Decrease speed (25% steps) |
| `;` (hold) | Boost: temporarily doubles speed |

### Setup wizard

| Key | Action |
|---|---|
| `[` or `,` | Scroll up in network list |
| `]` or `.` | Scroll down in network list |
| `Enter` | Select / confirm |
| `Backspace` | Delete character / go back |
| `R` | Rescan WiFi networks |

## HUD Layout

```
┌──────────────────────────────────────┐
│                                      │
│          MJPEG camera stream         │
│          (240 × 119 px)              │
│                                      │
├──────────────────────────────────────┤
│ FRT 12fps  │  0.42m  │  S:50%       │  ← 16 px HUD bar
└──────────────────────────────────────┘
  ↑ camera     ↑ ultrasonic  ↑ speed
  + FPS        + collision
               status
```

- **Camera label**: `FRT` = front camera, `GIM` = gimbal camera
- **Distance**: Green = clear, orange = slowdown zone, red = STOP
- **Speed**: Current speed scale percentage

## Troubleshooting

| Problem | Fix |
|---|---|
| Won't connect to WiFi | Re-run the setup wizard (hold G0 at boot). The ESP32-S3 only supports 2.4 GHz WiFi. |
| Wrong WiFi / IP saved | Hold G0 while powering on to re-run the setup wizard. |
| No camera stream | Make sure the Raspbot bringup is running. Check the Raspbot IP in the setup wizard. |
| Black screen with HUD | Camera stream is connecting but no frames yet. Wait a few seconds or check camera node. |
| Jerky driving | WiFi latency. Move closer to the router, or reduce traffic on the network. |
| Upload fails | Put Cardputer in download mode: power OFF → hold G0 → power ON → release G0. |
| Status shows `-- m` | Ultrasonic sensor not publishing. Check `enable_ultrasonic:=true` in bringup. |
| Want to clear all settings | Erase NVS: `pio run -t erase` then re-flash. |

## Dependencies

- [M5Cardputer Arduino library](https://github.com/m5stack/M5Cardputer)
- [TJpg_Decoder](https://github.com/Bodmer/TJpg_Decoder) (JPEG → RGB565 for display)
- PlatformIO (ESP32-S3 Arduino framework)
