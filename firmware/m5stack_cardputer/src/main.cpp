/*
 * Raspbot V2 — M5Stack Cardputer Teleop Controller
 *
 * Connects to the Raspbot's web video server over WiFi and provides:
 *   - Live MJPEG camera view (front or gimbal cam, toggled with TAB)
 *   - WASD keyboard teleop (same as web UI)
 *   - HUD overlay: ultrasonic distance, collision failsafe, battery
 *   - Camera switching, speed control, emergency stop
 *
 * Hardware: M5Stack Cardputer (ESP32-S3, 240×135 ST7789V2, 56-key KB)
 * Protocol: HTTP — reuses existing /stream_*_lo.mjpg, /api/cmd_vel, /status
 *
 * Configuration: Edit config.h or hold 'fn' on boot for setup menu.
 */

#include <M5Cardputer.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <TJpg_Decoder.h>
#include <Preferences.h>
#include <vector>
#include <algorithm>
#include "config.h"

// ── Runtime credentials (loaded from NVS, fallback to config.h) ─────
static Preferences prefs;
static char  rtSSID[64]  = "";
static char  rtPASS[64]  = "";
static char  rtHOST[64]  = "";
static int   rtPORT      = 8080;

// ── Display layout ──────────────────────────────────────────────────
// 240 wide × 135 tall (landscape).  We use the full area for camera,
// with a semi-transparent HUD bar at the bottom (16 px).
static constexpr int DISP_W   = 240;
static constexpr int DISP_H   = 135;
static constexpr int HUD_H    = 16;
static constexpr int CAM_H    = DISP_H - HUD_H;  // 119 px for camera image

// ── State ───────────────────────────────────────────────────────────
static WiFiClient   streamClient;
static bool          streamConnected = false;
static unsigned long streamConnectMs = 0;
static const unsigned long STREAM_RECONNECT_MS = 3000;

enum CameraSource { CAM_FRONT = 0, CAM_GIMBAL = 1 };
static CameraSource  currentCam = CAM_FRONT;

// Teleop state (-1.0 … +1.0 normalized)
static float cmdLinearX  = 0.0f;
static float cmdLinearY  = 0.0f;
static float cmdAngularZ = 0.0f;
static bool  cmdDirty    = false;
static unsigned long lastCmdSendMs = 0;
static constexpr unsigned long CMD_SEND_INTERVAL_MS = 100;  // 10 Hz
static float speedScale  = 0.5f;   // 0.25 / 0.50 / 0.75 / 1.00
static bool  shiftHeld   = false;

// Status polling
static float ultrasonicDist  = -1.0f;   // metres, -1 = unknown
static bool  collisionActive = false;
static bool  collisionEnabled = true;
static unsigned long lastStatusMs = 0;
static constexpr unsigned long STATUS_POLL_MS = 500;

// HUD
static unsigned long lastHudMs  = 0;
static constexpr unsigned long HUD_REFRESH_MS = 200;

// MJPEG parser state
static enum { MJPEG_SEARCH_BOUNDARY, MJPEG_SKIP_HEADERS, MJPEG_READ_JPEG }
    mjpegState = MJPEG_SEARCH_BOUNDARY;
static constexpr size_t JPEG_BUF_SIZE = 16384;  // 16 KB — server sends 240×180 low-res
static uint8_t* jpegBuf = nullptr;
static size_t   jpegLen = 0;
// Small line buffer for header parsing
static constexpr size_t LINE_BUF_SIZE = 256;
static char lineBuf[LINE_BUF_SIZE];
static size_t lineLen = 0;
static int jpegContentLength = -1;

// JPEG decode geometry
// Server low-res endpoints send 240×180 (matching display width).
// We use TJpgDec scale=1 and clip vertically at CAM_H (119 px).

// Frame stats
static unsigned long frameCount = 0;
static unsigned long lastFpsMs  = 0;
static float         fps        = 0.0f;

// ── NVS persistence ─────────────────────────────────────────────────
static void loadCredentials() {
    prefs.begin("raspbot", true);  // read-only
    String s;
    s = prefs.getString("ssid", DEFAULT_WIFI_SSID);
    strncpy(rtSSID, s.c_str(), sizeof(rtSSID) - 1);
    s = prefs.getString("pass", DEFAULT_WIFI_PASS);
    strncpy(rtPASS, s.c_str(), sizeof(rtPASS) - 1);
    s = prefs.getString("host", DEFAULT_RASPBOT_HOST);
    strncpy(rtHOST, s.c_str(), sizeof(rtHOST) - 1);
    rtPORT = prefs.getInt("port", DEFAULT_RASPBOT_PORT);
    prefs.end();
}

static void saveCredentials() {
    prefs.begin("raspbot", false);  // read-write
    prefs.putString("ssid", rtSSID);
    prefs.putString("pass", rtPASS);
    prefs.putString("host", rtHOST);
    prefs.putInt("port", rtPORT);
    prefs.end();
}

static bool hasStoredCredentials() {
    return strlen(rtSSID) > 0;
}

// ── Text input helper ───────────────────────────────────────────────
// Draws a prompt + editable text field.  User types on the keyboard,
// Backspace deletes, Enter confirms.  Returns the entered string in buf.
static void textInput(const char* prompt, char* buf, size_t bufLen,
                      bool masked = false) {
    size_t pos = strlen(buf);  // allow pre-filled value for editing
    bool done = false;

    while (!done) {
        // Draw
        M5Cardputer.Display.fillScreen(TFT_BLACK);
        M5Cardputer.Display.setTextSize(1);
        M5Cardputer.Display.setTextColor(TFT_CYAN, TFT_BLACK);
        M5Cardputer.Display.setCursor(4, 4);
        M5Cardputer.Display.println(prompt);

        M5Cardputer.Display.setCursor(4, 24);
        M5Cardputer.Display.setTextColor(TFT_WHITE, TFT_BLACK);
        if (masked) {
            for (size_t i = 0; i < pos; i++) M5Cardputer.Display.print('*');
        } else {
            M5Cardputer.Display.print(buf);
        }
        M5Cardputer.Display.print('_');  // cursor

        M5Cardputer.Display.setCursor(4, DISP_H - 20);
        M5Cardputer.Display.setTextColor(TFT_DARKGREY, TFT_BLACK);
        M5Cardputer.Display.print("[Enter] OK  [Bksp] Del");

        // Wait for key
        while (true) {
            M5Cardputer.update();
            if (M5Cardputer.Keyboard.isChange() && M5Cardputer.Keyboard.isPressed()) {
                break;
            }
            delay(20);
        }

        Keyboard_Class::KeysState ks = M5Cardputer.Keyboard.keysState();

        // Enter = confirm
        if (ks.enter) {
            done = true;
        }
        // Backspace = delete last char
        else if (ks.del && pos > 0) {
            pos--;
            buf[pos] = '\0';
        }
        // Printable character
        else if (ks.word.length() > 0) {
            for (size_t i = 0; i < ks.word.length() && pos < bufLen - 1; i++) {
                buf[pos++] = ks.word[i];
            }
            buf[pos] = '\0';
        }
    }
}

// ── WiFi scan + select UI ───────────────────────────────────────────
// Scans for networks, shows a scrollable list, returns selected SSID.
static bool wifiScanSelect(char* ssidOut, size_t ssidLen) {
    M5Cardputer.Display.fillScreen(TFT_BLACK);
    M5Cardputer.Display.setTextSize(1);
    M5Cardputer.Display.setTextColor(TFT_YELLOW, TFT_BLACK);
    M5Cardputer.Display.setCursor(4, 4);
    M5Cardputer.Display.println("Scanning WiFi...");

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(200);

    int n = WiFi.scanNetworks();
    if (n <= 0) {
        M5Cardputer.Display.setTextColor(TFT_RED, TFT_BLACK);
        M5Cardputer.Display.setCursor(4, 24);
        M5Cardputer.Display.println("No networks found!");
        M5Cardputer.Display.println("Press any key to rescan...");
        while (true) {
            M5Cardputer.update();
            if (M5Cardputer.Keyboard.isChange() && M5Cardputer.Keyboard.isPressed()) break;
            delay(20);
        }
        WiFi.scanDelete();
        return false;  // caller can retry
    }

    // Deduplicate + sort by RSSI (strongest first)
    struct NetInfo { String ssid; int rssi; bool enc; };
    std::vector<NetInfo> nets;
    for (int i = 0; i < n; i++) {
        String s = WiFi.SSID(i);
        if (s.length() == 0) continue;  // skip hidden
        bool dup = false;
        for (auto& e : nets) {
            if (e.ssid == s) { dup = true; if (WiFi.RSSI(i) > e.rssi) e.rssi = WiFi.RSSI(i); break; }
        }
        if (!dup) {
            nets.push_back({s, WiFi.RSSI(i), WiFi.encryptionType(i) != WIFI_AUTH_OPEN});
        }
    }
    WiFi.scanDelete();

    // Sort by signal strength
    std::sort(nets.begin(), nets.end(), [](const NetInfo& a, const NetInfo& b) {
        return a.rssi > b.rssi;
    });

    int sel = 0;
    int scroll = 0;
    const int ROWS = 10;      // visible rows
    const int ROW_H = 11;     // pixels per row
    const int LIST_Y = 16;    // top of list area

    while (true) {
        // Draw list
        M5Cardputer.Display.fillScreen(TFT_BLACK);
        M5Cardputer.Display.setTextSize(1);
        M5Cardputer.Display.setTextColor(TFT_CYAN, TFT_BLACK);
        M5Cardputer.Display.setCursor(4, 2);
        M5Cardputer.Display.printf("WiFi (%d found) [Up/Dn] [Enter]", (int)nets.size());

        for (int i = 0; i < ROWS && (scroll + i) < (int)nets.size(); i++) {
            int idx = scroll + i;
            int y = LIST_Y + i * ROW_H;
            bool isSel = (idx == sel);

            if (isSel) {
                M5Cardputer.Display.fillRect(0, y, DISP_W, ROW_H, TFT_DARKGREY);
                M5Cardputer.Display.setTextColor(TFT_WHITE, TFT_DARKGREY);
            } else {
                M5Cardputer.Display.setTextColor(TFT_WHITE, TFT_BLACK);
            }

            M5Cardputer.Display.setCursor(4, y + 1);
            // Signal strength bars (1-4)
            int bars = 1;
            if (nets[idx].rssi > -50) bars = 4;
            else if (nets[idx].rssi > -65) bars = 3;
            else if (nets[idx].rssi > -75) bars = 2;
            char barCh[5] = "....";
            for (int b = 0; b < bars; b++) barCh[b] = '|';
            M5Cardputer.Display.printf("%s%s %s",
                barCh,
                nets[idx].enc ? "*" : " ",
                nets[idx].ssid.substring(0, 28).c_str());
        }

        // Footer
        M5Cardputer.Display.setCursor(4, DISP_H - 10);
        M5Cardputer.Display.setTextColor(TFT_DARKGREY, TFT_BLACK);
        M5Cardputer.Display.print("[R] Rescan  [Esc] Back");

        // Wait for key
        while (true) {
            M5Cardputer.update();
            if (M5Cardputer.Keyboard.isChange() && M5Cardputer.Keyboard.isPressed()) break;
            delay(20);
        }

        Keyboard_Class::KeysState ks = M5Cardputer.Keyboard.keysState();

        // Navigate
        if (ks.word == ";" || ks.word == ",") {  // ";" as up, "," as down on Cardputer
            // Fallthrough — also check arrow-like keys below
        }
        // The Cardputer has no dedicated arrow keys.  Use:
        //   "," or "[" = up    "." or "]" = down
        bool goUp   = (ks.word.indexOf(',') >= 0) || (ks.word.indexOf('[') >= 0);
        bool goDown = (ks.word.indexOf('.') >= 0) || (ks.word.indexOf(']') >= 0);

        if (goUp && sel > 0) {
            sel--;
            if (sel < scroll) scroll = sel;
        }
        if (goDown && sel < (int)nets.size() - 1) {
            sel++;
            if (sel >= scroll + ROWS) scroll = sel - ROWS + 1;
        }

        // Enter = select
        if (ks.enter) {
            strncpy(ssidOut, nets[sel].ssid.c_str(), ssidLen - 1);
            ssidOut[ssidLen - 1] = '\0';
            return true;
        }

        // R = rescan
        if (ks.word.indexOf('r') >= 0 || ks.word.indexOf('R') >= 0) {
            return false;  // caller retries
        }

        // ESC / Backspace = abort (use stored creds if any)
        if (ks.del) {
            ssidOut[0] = '\0';
            return true;  // empty = caller skips
        }
    }
}

// ── Full setup wizard ───────────────────────────────────────────────
// Guides the user through: WiFi selection → password → Raspbot host → port.
// Saves results to NVS.
static void runSetupWizard() {
    M5Cardputer.Display.fillScreen(TFT_BLACK);
    M5Cardputer.Display.setTextSize(1);
    M5Cardputer.Display.setTextColor(TFT_GREEN, TFT_BLACK);
    M5Cardputer.Display.setCursor(4, 4);
    M5Cardputer.Display.println("=== WiFi Setup Wizard ===");
    M5Cardputer.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5Cardputer.Display.println();
    M5Cardputer.Display.println("  Select a WiFi network,");
    M5Cardputer.Display.println("  enter the password, and");
    M5Cardputer.Display.println("  set the Raspbot IP.");
    M5Cardputer.Display.println();
    M5Cardputer.Display.setTextColor(TFT_DARKGREY, TFT_BLACK);
    M5Cardputer.Display.println("  Hold G0 at boot to");
    M5Cardputer.Display.println("  re-run this wizard.");
    M5Cardputer.Display.println();
    M5Cardputer.Display.setTextColor(TFT_YELLOW, TFT_BLACK);
    M5Cardputer.Display.println("  Press any key to start...");

    while (true) {
        M5Cardputer.update();
        if (M5Cardputer.Keyboard.isChange() && M5Cardputer.Keyboard.isPressed()) break;
        delay(20);
    }

    // Step 1: WiFi scan + select (retry loop)
    bool selected = false;
    while (!selected) {
        selected = wifiScanSelect(rtSSID, sizeof(rtSSID));
    }
    if (strlen(rtSSID) == 0) return;  // user aborted

    // Step 2: Password
    rtPASS[0] = '\0';
    char prompt[128];
    snprintf(prompt, sizeof(prompt), "Password for '%s':", rtSSID);
    textInput(prompt, rtPASS, sizeof(rtPASS), true);

    // Step 3: Try connecting
    M5Cardputer.Display.fillScreen(TFT_BLACK);
    M5Cardputer.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5Cardputer.Display.setCursor(4, 4);
    M5Cardputer.Display.printf("Connecting to %s...", rtSSID);

    WiFi.mode(WIFI_STA);
    WiFi.begin(rtSSID, rtPASS);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        M5Cardputer.Display.print(".");
        attempts++;
    }

    if (WiFi.status() != WL_CONNECTED) {
        M5Cardputer.Display.println();
        M5Cardputer.Display.setTextColor(TFT_RED, TFT_BLACK);
        M5Cardputer.Display.println("Connection failed!");
        M5Cardputer.Display.println("Check password & retry.");
        M5Cardputer.Display.setTextColor(TFT_DARKGREY, TFT_BLACK);
        M5Cardputer.Display.println("\nPress any key...");
        WiFi.disconnect();
        while (true) {
            M5Cardputer.update();
            if (M5Cardputer.Keyboard.isChange() && M5Cardputer.Keyboard.isPressed()) break;
            delay(20);
        }
        // Restart wizard
        runSetupWizard();
        return;
    }

    M5Cardputer.Display.println();
    M5Cardputer.Display.setTextColor(TFT_GREEN, TFT_BLACK);
    M5Cardputer.Display.printf("WiFi OK!  IP: %s\n", WiFi.localIP().toString().c_str());
    delay(800);

    // Step 4: Raspbot host
    if (strlen(rtHOST) == 0) {
        strncpy(rtHOST, DEFAULT_RASPBOT_HOST, sizeof(rtHOST) - 1);
    }
    textInput("Raspbot IP address:", rtHOST, sizeof(rtHOST), false);

    // Step 5: Port (pre-filled with current value)
    char portBuf[8];
    snprintf(portBuf, sizeof(portBuf), "%d", rtPORT);
    textInput("Raspbot port (default 8080):", portBuf, sizeof(portBuf), false);
    rtPORT = atoi(portBuf);
    if (rtPORT <= 0 || rtPORT > 65535) rtPORT = DEFAULT_RASPBOT_PORT;

    // Save to NVS
    saveCredentials();

    M5Cardputer.Display.fillScreen(TFT_BLACK);
    M5Cardputer.Display.setTextColor(TFT_GREEN, TFT_BLACK);
    M5Cardputer.Display.setCursor(4, 4);
    M5Cardputer.Display.println("Setup complete!");
    M5Cardputer.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5Cardputer.Display.printf("\n  SSID: %s\n  Host: %s:%d\n",
        rtSSID, rtHOST, rtPORT);
    M5Cardputer.Display.println("\nSaved to flash.");
    M5Cardputer.Display.setTextColor(TFT_DARKGREY, TFT_BLACK);
    M5Cardputer.Display.println("\nStarting in 2s...");
    delay(2000);
}

// ── JPEG decoder callback ───────────────────────────────────────────
// TJpg_Decoder calls this for each MCU block decoded.
// Image is already 240px wide (matching display). We just clip vertically.
static bool onJpegBlock(int16_t x, int16_t y, uint16_t w, uint16_t h,
                        uint16_t* bitmap) {
    // Skip blocks entirely below camera area
    if (y >= CAM_H) return false;

    // Clip bottom edge to camera area
    uint16_t drawH = h;
    if (y + drawH > CAM_H) drawH = CAM_H - y;

    M5Cardputer.Display.pushImage(x, y, w, drawH, bitmap);
    return true;
}

// ── WiFi connection (uses runtime credentials) ─────────────────────
static void connectWiFi() {
    // If already connected from the setup wizard, just show status
    if (WiFi.status() == WL_CONNECTED) {
        M5Cardputer.Display.fillScreen(TFT_BLACK);
        M5Cardputer.Display.setCursor(4, 4);
        M5Cardputer.Display.setTextColor(TFT_GREEN);
        M5Cardputer.Display.printf("WiFi OK!\nIP: %s\nRaspbot: %s:%d",
            WiFi.localIP().toString().c_str(), rtHOST, rtPORT);
        delay(1000);
        return;
    }

    M5Cardputer.Display.fillScreen(TFT_BLACK);
    M5Cardputer.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5Cardputer.Display.setTextSize(1);
    M5Cardputer.Display.setCursor(4, 4);
    M5Cardputer.Display.printf("Connecting to WiFi...\n  %s", rtSSID);

    WiFi.mode(WIFI_STA);
    WiFi.begin(rtSSID, rtPASS);

    int dots = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        M5Cardputer.Display.print(".");
        dots++;
        if (dots > 20) {
            // Stored credentials failed — offer the wizard
            WiFi.disconnect();
            M5Cardputer.Display.fillScreen(TFT_BLACK);
            M5Cardputer.Display.setCursor(4, 4);
            M5Cardputer.Display.setTextColor(TFT_RED, TFT_BLACK);
            M5Cardputer.Display.printf("Cannot connect to '%s'", rtSSID);
            M5Cardputer.Display.setCursor(4, 24);
            M5Cardputer.Display.setTextColor(TFT_WHITE, TFT_BLACK);
            M5Cardputer.Display.println("\nPress [Enter] for setup wizard");
            M5Cardputer.Display.println("Press [R] to retry");
            while (true) {
                M5Cardputer.update();
                if (M5Cardputer.Keyboard.isChange() && M5Cardputer.Keyboard.isPressed()) {
                    Keyboard_Class::KeysState ks = M5Cardputer.Keyboard.keysState();
                    if (ks.enter) {
                        runSetupWizard();
                        return;  // wizard already connected
                    }
                    if (ks.word.indexOf('r') >= 0 || ks.word.indexOf('R') >= 0) {
                        dots = 0;
                        M5Cardputer.Display.fillScreen(TFT_BLACK);
                        M5Cardputer.Display.setTextColor(TFT_WHITE, TFT_BLACK);
                        M5Cardputer.Display.setCursor(4, 4);
                        M5Cardputer.Display.printf("Connecting to WiFi...\n  %s", rtSSID);
                        WiFi.begin(rtSSID, rtPASS);
                        break;
                    }
                }
                delay(20);
            }
        }
    }

    M5Cardputer.Display.fillScreen(TFT_BLACK);
    M5Cardputer.Display.setCursor(4, 4);
    M5Cardputer.Display.setTextColor(TFT_GREEN);
    M5Cardputer.Display.printf("WiFi OK!\nIP: %s\nRaspbot: %s:%d",
        WiFi.localIP().toString().c_str(), rtHOST, rtPORT);
    delay(1000);
}

// ── MJPEG stream connection ─────────────────────────────────────────
static void connectStream() {
    if (streamClient.connected()) {
        streamClient.stop();
    }
    streamConnected = false;
    mjpegState = MJPEG_SEARCH_BOUNDARY;
    jpegLen = 0;
    lineLen = 0;
    jpegContentLength = -1;

    const char* path = (currentCam == CAM_FRONT)
        ? "/stream_front_lo.mjpg?w=240&h=180&q=50"
        : "/stream_lo.mjpg?w=240&h=180&q=50";

    if (!streamClient.connect(rtHOST, rtPORT)) {
        return;
    }

    // Send HTTP GET
    streamClient.printf(
        "GET %s HTTP/1.1\r\n"
        "Host: %s:%d\r\n"
        "Connection: keep-alive\r\n"
        "\r\n",
        path, rtHOST, rtPORT
    );

    // Wait for first response byte (up to 3 s)
    unsigned long t0 = millis();
    while (!streamClient.available() && (millis() - t0) < 3000) {
        delay(10);
    }

    if (streamClient.available()) {
        streamConnected = true;
        streamConnectMs = millis();
        // Skip the initial HTTP response line + headers until the first boundary.
        // The MJPEG parser will handle this via MJPEG_SEARCH_BOUNDARY state.
    }
}

// ── MJPEG frame reader ──────────────────────────────────────────────
// Reads available bytes from the stream and decodes one JPEG frame at a time.
static void processMjpegStream() {
    if (!streamClient.connected()) {
        streamConnected = false;
        return;
    }

    int avail = streamClient.available();
    if (avail <= 0) return;

    // Process in chunks to stay responsive
    int toRead = min(avail, 4096);

    while (toRead > 0) {
        switch (mjpegState) {
        case MJPEG_SEARCH_BOUNDARY: {
            // Read lines until we find one starting with "--"
            while (toRead > 0 && streamClient.available()) {
                int c = streamClient.read();
                toRead--;
                if (c < 0) break;
                if (c == '\n') {
                    lineBuf[lineLen] = '\0';
                    if (lineLen >= 2 && lineBuf[0] == '-' && lineBuf[1] == '-') {
                        // Found boundary, move to header reading
                        mjpegState = MJPEG_SKIP_HEADERS;
                        lineLen = 0;
                        jpegContentLength = -1;
                        break;
                    }
                    lineLen = 0;
                } else if (c != '\r') {
                    if (lineLen < LINE_BUF_SIZE - 1) {
                        lineBuf[lineLen++] = (char)c;
                    }
                }
            }
            break;
        }

        case MJPEG_SKIP_HEADERS: {
            // Read header lines until empty line (end of part headers)
            while (toRead > 0 && streamClient.available()) {
                int c = streamClient.read();
                toRead--;
                if (c < 0) break;
                if (c == '\n') {
                    lineBuf[lineLen] = '\0';
                    if (lineLen == 0) {
                        // Empty line = end of headers → JPEG data follows
                        mjpegState = MJPEG_READ_JPEG;
                        jpegLen = 0;
                        break;
                    }
                    // Check for Content-Length header
                    if (lineLen > 16) {
                        // Case-insensitive check for "Content-Length:"
                        char lower[LINE_BUF_SIZE];
                        for (size_t i = 0; i <= lineLen; i++)
                            lower[i] = tolower(lineBuf[i]);
                        char* cl = strstr(lower, "content-length:");
                        if (cl) {
                            jpegContentLength = atoi(cl + 15);
                        }
                    }
                    lineLen = 0;
                } else if (c != '\r') {
                    if (lineLen < LINE_BUF_SIZE - 1) {
                        lineBuf[lineLen++] = (char)c;
                    }
                }
            }
            break;
        }

        case MJPEG_READ_JPEG: {
            if (jpegContentLength > 0) {
                // We know the exact size — read it
                size_t remaining = (size_t)jpegContentLength - jpegLen;
                size_t chunk = min((size_t)toRead, remaining);
                chunk = min(chunk, JPEG_BUF_SIZE - jpegLen);
                if (chunk > 0 && streamClient.available()) {
                    size_t got = streamClient.readBytes(jpegBuf + jpegLen, chunk);
                    jpegLen += got;
                    toRead -= (int)got;
                }
                if ((int)jpegLen >= jpegContentLength) {
                    // Full JPEG received — decode & display (offset in callback)
                    TJpgDec.drawJpg(0, 0, jpegBuf, jpegLen);
                    frameCount++;
                    mjpegState = MJPEG_SEARCH_BOUNDARY;
                    lineLen = 0;
                }
            } else {
                // No Content-Length — scan for JPEG end marker (FF D9)
                while (toRead > 0 && streamClient.available()) {
                    if (jpegLen >= JPEG_BUF_SIZE) {
                        // Overflow — discard and search for next boundary
                        mjpegState = MJPEG_SEARCH_BOUNDARY;
                        jpegLen = 0;
                        lineLen = 0;
                        break;
                    }
                    int c = streamClient.read();
                    toRead--;
                    if (c < 0) break;
                    jpegBuf[jpegLen++] = (uint8_t)c;

                    // Check for JPEG EOI marker (0xFF 0xD9)
                    if (jpegLen >= 2 &&
                        jpegBuf[jpegLen - 2] == 0xFF &&
                        jpegBuf[jpegLen - 1] == 0xD9) {
                        TJpgDec.drawJpg(0, 0, jpegBuf, jpegLen);
                        frameCount++;
                        mjpegState = MJPEG_SEARCH_BOUNDARY;
                        lineLen = 0;
                        break;
                    }
                }
            }
            break;
        }
        }  // switch
    }  // while
}

// ── Teleop HTTP sender ──────────────────────────────────────────────
static void sendCmdVel(float lx, float ly, float az) {
    HTTPClient http;
    char url[128];
    snprintf(url, sizeof(url), "http://%s:%d/api/cmd_vel", rtHOST, rtPORT);
    http.begin(url);
    http.addHeader("Content-Type", "application/json");

    char body[128];
    snprintf(body, sizeof(body),
        "{\"linear_x\":%.3f,\"linear_y\":%.3f,\"angular_z\":%.3f}",
        lx, ly, az);
    http.POST((uint8_t*)body, strlen(body));
    http.end();
}

static void sendStop() {
    HTTPClient http;
    char url[128];
    snprintf(url, sizeof(url), "http://%s:%d/api/cmd_vel/stop", rtHOST, rtPORT);
    http.begin(url);
    http.POST("", 0);
    http.end();
}

// ── Status poller ───────────────────────────────────────────────────
static void pollStatus() {
    HTTPClient http;
    char url[128];
    snprintf(url, sizeof(url), "http://%s:%d/status", rtHOST, rtPORT);
    http.begin(url);
    http.setTimeout(500);
    int code = http.GET();
    if (code == 200) {
        String payload = http.getString();
        // Lightweight JSON parsing for the fields we care about.
        // We avoid a full JSON library to save memory.

        // collision_failsafe.distance_m
        int idx = payload.indexOf("\"distance_m\"");
        if (idx >= 0) {
            idx = payload.indexOf(':', idx);
            if (idx >= 0) {
                String val = payload.substring(idx + 1, payload.indexOf(',', idx));
                val.trim();
                if (val == "null") {
                    ultrasonicDist = -1.0f;
                } else {
                    ultrasonicDist = val.toFloat();
                }
            }
        }

        // collision_failsafe.active
        idx = payload.indexOf("\"active\"");
        if (idx >= 0) {
            collisionActive = payload.indexOf("true", idx) < payload.indexOf(',', idx)
                              && payload.indexOf("true", idx) >= idx;
        }

        // collision_failsafe.enabled
        idx = payload.indexOf("\"enabled\"");
        if (idx >= 0) {
            // Find the closest "enabled" inside collision_failsafe block
            int cfIdx = payload.indexOf("collision_failsafe");
            if (cfIdx >= 0) {
                int eIdx = payload.indexOf("\"enabled\"", cfIdx);
                if (eIdx >= 0) {
                    collisionEnabled = payload.indexOf("true", eIdx) < payload.indexOf(',', eIdx)
                                       && payload.indexOf("true", eIdx) >= eIdx;
                }
            }
        }
    }
    http.end();
}

// ── HUD drawing ─────────────────────────────────────────────────────
static void drawHud() {
    int y0 = CAM_H;  // HUD starts below camera area

    // Black bar background
    M5Cardputer.Display.fillRect(0, y0, DISP_W, HUD_H, TFT_BLACK);
    M5Cardputer.Display.setTextSize(1);
    M5Cardputer.Display.setTextDatum(TL_DATUM);

    // Left: camera source + FPS
    M5Cardputer.Display.setTextColor(TFT_CYAN, TFT_BLACK);
    const char* camLabel = (currentCam == CAM_FRONT) ? "FRT" : "GIM";
    M5Cardputer.Display.setCursor(2, y0 + 3);
    M5Cardputer.Display.printf("%s %.0ffps", camLabel, fps);

    // Center: ultrasonic distance + collision status
    char distStr[32];
    if (ultrasonicDist >= 0.0f) {
        snprintf(distStr, sizeof(distStr), "%.2fm", ultrasonicDist);
    } else {
        snprintf(distStr, sizeof(distStr), "-- m");
    }

    uint16_t distColor;
    if (collisionActive) {
        distColor = TFT_RED;
    } else if (ultrasonicDist >= 0 && ultrasonicDist < 0.35f) {
        distColor = TFT_ORANGE;
    } else {
        distColor = TFT_GREEN;
    }

    M5Cardputer.Display.setTextColor(distColor, TFT_BLACK);
    M5Cardputer.Display.setCursor(80, y0 + 3);
    M5Cardputer.Display.print(distStr);
    if (collisionActive) {
        M5Cardputer.Display.setTextColor(TFT_RED, TFT_BLACK);
        M5Cardputer.Display.print(" STOP");
    } else if (!collisionEnabled) {
        M5Cardputer.Display.setTextColor(TFT_DARKGREY, TFT_BLACK);
        M5Cardputer.Display.print(" OFF");
    }

    // Right: speed scale
    M5Cardputer.Display.setTextColor(TFT_YELLOW, TFT_BLACK);
    M5Cardputer.Display.setCursor(195, y0 + 3);
    M5Cardputer.Display.printf("S:%.0f%%", speedScale * 100.0f);
}

// ── Keyboard processing ─────────────────────────────────────────────
static void processKeyboard() {
    if (!M5Cardputer.Keyboard.isChange()) return;

    // Reset motion each scan — we only hold while keys are physically down
    float lx = 0.0f, ly = 0.0f, az = 0.0f;

    // Check individual keys via isKeyPressed
    // WASD = drive, QE = strafe, Space = stop
    if (M5Cardputer.Keyboard.isKeyPressed('w')) lx += 1.0f;
    if (M5Cardputer.Keyboard.isKeyPressed('s')) lx -= 1.0f;
    if (M5Cardputer.Keyboard.isKeyPressed('a')) az += 1.0f;    // turn left
    if (M5Cardputer.Keyboard.isKeyPressed('d')) az -= 1.0f;    // turn right
    if (M5Cardputer.Keyboard.isKeyPressed('q')) ly += 1.0f;    // strafe left
    if (M5Cardputer.Keyboard.isKeyPressed('e')) ly -= 1.0f;    // strafe right

    // Speed modifier: hold any of the top-row number keys (or '/') as shift
    // The Fn key is mapped to KEY_FN in M5Cardputer library
    shiftHeld = M5Cardputer.Keyboard.isKeyPressed(';');

    float scale = shiftHeld ? min(speedScale * 2.0f, 1.0f) : speedScale;
    cmdLinearX  = lx * scale;
    cmdLinearY  = ly * scale;
    cmdAngularZ = az * scale;
    cmdDirty = true;

    // Special keys (only on press, not hold)
    if (M5Cardputer.Keyboard.isKeyPressed(' ')) {
        // Emergency stop
        cmdLinearX = cmdLinearY = cmdAngularZ = 0.0f;
        cmdDirty = true;
        sendStop();
    }

    // TAB (mapped to KEY_TAB) = toggle camera
    if (M5Cardputer.Keyboard.isKeyPressed(KEY_TAB)) {
        currentCam = (currentCam == CAM_FRONT) ? CAM_GIMBAL : CAM_FRONT;
        streamConnected = false;
        streamClient.stop();
        M5Cardputer.Display.fillScreen(TFT_BLACK);
        M5Cardputer.Display.setCursor(4, 60);
        M5Cardputer.Display.setTextColor(TFT_CYAN, TFT_BLACK);
        M5Cardputer.Display.printf("Switching to %s cam...",
            currentCam == CAM_FRONT ? "front" : "gimbal");
        connectStream();
    }

    // +/- speed adjustment
    if (M5Cardputer.Keyboard.isKeyPressed('+') || M5Cardputer.Keyboard.isKeyPressed('=')) {
        speedScale = min(speedScale + 0.25f, 1.0f);
    }
    if (M5Cardputer.Keyboard.isKeyPressed('-') || M5Cardputer.Keyboard.isKeyPressed('_')) {
        speedScale = max(speedScale - 0.25f, 0.25f);
    }
}

// ── Arduino setup ───────────────────────────────────────────────────
void setup() {
    auto cfg = M5.config();
    M5Cardputer.begin(cfg, true);

    M5Cardputer.Display.setRotation(1);  // Landscape
    M5Cardputer.Display.fillScreen(TFT_BLACK);
    M5Cardputer.Display.setTextSize(1);
    M5Cardputer.Display.setTextColor(TFT_WHITE);
    M5Cardputer.Display.setCursor(4, 4);
    M5Cardputer.Display.println("Raspbot V2 Controller");
    M5Cardputer.Display.println("Initializing...");

    // Allocate JPEG buffer in PSRAM if available, else heap
    jpegBuf = (uint8_t*)ps_malloc(JPEG_BUF_SIZE);
    if (!jpegBuf) {
        jpegBuf = (uint8_t*)malloc(JPEG_BUF_SIZE);
    }
    if (!jpegBuf) {
        M5Cardputer.Display.setTextColor(TFT_RED);
        M5Cardputer.Display.println("JPEG buf alloc FAILED!");
        while (1) delay(1000);
    }

    // Configure JPEG decoder
    // Server low-res endpoints send 240×180.  Scale 1 = no decode scaling needed.
    TJpgDec.setJpgScale(1);
    TJpgDec.setSwapBytes(true);      // ST7789 needs byte-swapped RGB565
    TJpgDec.setCallback(onJpegBlock);

    // ── Load saved credentials from NVS flash ────────────────────────
    loadCredentials();

    // Check if G0 button is held at boot → force setup wizard
    // G0 is active-low on GPIO 0
    pinMode(0, INPUT_PULLUP);
    delay(50);  // debounce
    bool forceSetup = (digitalRead(0) == LOW);

    if (forceSetup || !hasStoredCredentials()) {
        // First boot or user requested reconfiguration
        runSetupWizard();
    }

    // Connect to WiFi (may already be connected from wizard)
    connectWiFi();

    // Start MJPEG stream
    connectStream();
}

// ── Arduino main loop ───────────────────────────────────────────────
void loop() {
    M5Cardputer.update();

    unsigned long now = millis();

    // ── 1. Keyboard input ────────────────────────────────────────────
    processKeyboard();

    // ── 2. Send teleop commands at 10 Hz ─────────────────────────────
    if ((now - lastCmdSendMs) >= CMD_SEND_INTERVAL_MS) {
        lastCmdSendMs = now;
        if (cmdDirty ||
            fabsf(cmdLinearX) > 0.01f ||
            fabsf(cmdLinearY) > 0.01f ||
            fabsf(cmdAngularZ) > 0.01f) {
            sendCmdVel(cmdLinearX, cmdLinearY, cmdAngularZ);
            cmdDirty = false;
        }
    }

    // ── 3. Process MJPEG stream ──────────────────────────────────────
    if (streamConnected) {
        processMjpegStream();
    } else if ((now - streamConnectMs) > STREAM_RECONNECT_MS) {
        connectStream();
        streamConnectMs = now;
    }

    // ── 4. FPS counter ───────────────────────────────────────────────
    if ((now - lastFpsMs) >= 1000) {
        fps = (float)frameCount * 1000.0f / (float)(now - lastFpsMs);
        frameCount = 0;
        lastFpsMs = now;
    }

    // ── 5. Poll status ───────────────────────────────────────────────
    if ((now - lastStatusMs) >= STATUS_POLL_MS) {
        lastStatusMs = now;
        pollStatus();
    }

    // ── 6. Redraw HUD ────────────────────────────────────────────────
    if ((now - lastHudMs) >= HUD_REFRESH_MS) {
        lastHudMs = now;
        drawHud();
    }
}
