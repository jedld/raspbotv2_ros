/*
 * Raspbot Cardputer BLE Teleop  –  Enhanced UI with panels + camera capture
 *
 * Display:  240 × 135  (ST7789, rotation 1 = landscape)
 * Panels :  [1] Teleop   – drive telemetry & controls
 *           [2] Orient   – graphical compass + heading
 *           [3] Camera   – capture photo, view thumbnail, save to SD
 *
 * Keys:
 *   1/2/3       – switch panel directly
 *   Tab         – cycle to next panel
 *   WASD / JL   – drive (hold‑to‑move)
 *   F           – toggle follow mode
 *   G           – toggle gimbal mode
 *   Fn + ,./;   – gimbal pan/tilt
 *   Fn + Tab    – center gimbal
 *   P (or Enter on Camera panel) – capture photo
 */

#include <M5Cardputer.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <SD.h>
#include <FS.h>
#include <SPI.h>

// ── Display constants ──────────────────────────────────────────────────
static const int DISP_W = 240;
static const int DISP_H = 135;
static const int HEADER_H = 14;           // top status bar height
static const int TAB_H  = 12;             // bottom tab bar height
static const int BODY_Y = HEADER_H + 2;   // body area Y start
static const int BODY_H = DISP_H - HEADER_H - TAB_H - 4; // usable body height

// ── BLE UUIDs ──────────────────────────────────────────────────────────
static const char *DEVICE_NAME   = "RaspbotCardputer";
static const char *SERVICE_UUID  = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char *TX_NOTIFY_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";
static const char *RX_WRITE_UUID  = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";

// ── BLE objects ────────────────────────────────────────────────────────
BLECharacteristic *txNotifyChar = nullptr;
BLECharacteristic *rxWriteChar  = nullptr;
volatile bool centralConnected = false;
volatile uint32_t connectTimeMs = 0;
String localMac = "";

// ── Drive / command state ──────────────────────────────────────────────
float cmdVx = 0.0f, cmdVy = 0.0f, cmdWz = 0.0f;
float gimbalPan = 90.0f, gimbalTilt = 45.0f;
bool followEnabled = false, gimbalMode = false;

// ── Telemetry from Pi ──────────────────────────────────────────────────
float telVx = 0.0f, telVy = 0.0f, telWz = 0.0f;
float telSpeed = 0.0f, telGyro = 0.0f;
float telCompass = 0.0f, telHeading = 0.0f;
float telRoll = 0.0f, telPitch = 0.0f;
bool  telFollow = false, telGimbalMode = false;

// ── Timing ─────────────────────────────────────────────────────────────
uint32_t lastSendMs = 0, lastUiMs = 0;
bool prevFPressed = false, prevGPressed = false;
bool prevPPressed = false;
bool prev1Pressed = false, prev2Pressed = false, prev3Pressed = false;
bool prevTabPressed = false;

// ── Panel system ───────────────────────────────────────────────────────
enum Panel { PANEL_TELEOP = 0, PANEL_ORIENT = 1, PANEL_CAMERA = 2, PANEL_COUNT = 3 };
Panel currentPanel = PANEL_TELEOP;
bool  panelDirty  = true;      // force full redraw on panel switch

// ── Camera / image transfer state ──────────────────────────────────────
enum PhotoState { PHOTO_IDLE, PHOTO_REQUESTED, PHOTO_RECEIVING, PHOTO_DONE, PHOTO_ERROR };
PhotoState photoState = PHOTO_IDLE;
String photoStatusMsg = "Press P or ENTER to capture";

// Image receive buffer  (max ~60 KB JPEG, fits in PSRAM)
static const size_t IMG_BUF_MAX = 65536;
uint8_t *imgBuf = nullptr;
size_t   imgBufLen = 0;
size_t   imgExpectedLen = 0;
bool     imgDisplayed = false;

// SD card
bool sdReady = false;
int  photoCounter = 0;

// ── Helpers ────────────────────────────────────────────────────────────
static bool keyPressedAny(char a, char b = '\0') {
  return M5Cardputer.Keyboard.isKeyPressed(a) || (b != '\0' && M5Cardputer.Keyboard.isKeyPressed(b));
}

static float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

// ── BLE helpers ────────────────────────────────────────────────────────
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) override {
    connectTimeMs = millis();
    centralConnected = true;
  }
  void onDisconnect(BLEServer *pServer) override {
    centralConnected = false;
    connectTimeMs = 0;
    pServer->startAdvertising();
  }
};

static bool bleReady() {
  return centralConnected && connectTimeMs > 0 && (millis() - connectTimeMs) > 1500;
}

static void notifyLine(const String &line) {
  if (!bleReady() || txNotifyChar == nullptr) return;
  txNotifyChar->setValue((uint8_t *)line.c_str(), line.length());
  txNotifyChar->notify();
}

// ── Parse incoming data from Pi ────────────────────────────────────────
// Binary image data buffer for chunked receive
static String rxLineBuf = "";

static void parseTelemetryLine(const String &line) {
  if (line.startsWith("TEL|")) {
    // TEL|vx|vy|wz|speed|gyro|compass|heading|follow|gmode|roll|pitch
    String fields[12];
    int fc = 0; int s = 0;
    while (fc < 12) {
      int sep = line.indexOf('|', s);
      if (sep < 0) { fields[fc++] = line.substring(s); break; }
      fields[fc++] = line.substring(s, sep); s = sep + 1;
    }
    if (fc < 10) return;
    telVx      = fields[1].toFloat();
    telVy      = fields[2].toFloat();
    telWz      = fields[3].toFloat();
    telSpeed   = fields[4].toFloat();
    telGyro    = fields[5].toFloat();
    telCompass = fields[6].toFloat();
    telHeading = fields[7].toFloat();
    telFollow  = fields[8].toInt() == 1;
    telGimbalMode = fields[9].toInt() == 1;
    if (fc >= 12) {
      telRoll  = fields[10].toFloat();
      telPitch = fields[11].toFloat();
    }
  }
}

static void handleImgHeader(const String &line) {
  // IMG|BEGIN|<total_bytes>
  if (line.startsWith("IMG|BEGIN|")) {
    imgExpectedLen = (size_t)line.substring(10).toInt();
    if (imgExpectedLen == 0 || imgExpectedLen > IMG_BUF_MAX) {
      photoState = PHOTO_ERROR;
      photoStatusMsg = "Image too large or invalid";
      return;
    }
    imgBufLen = 0;
    photoState = PHOTO_RECEIVING;
    imgDisplayed = false;
    photoStatusMsg = "Receiving...";
  } else if (line.startsWith("IMG|ERROR|")) {
    photoState = PHOTO_ERROR;
    photoStatusMsg = line.substring(10);
  }
}

// Callback for binary image chunk data written to RX characteristic
static void handleRxData(const uint8_t *data, size_t len) {
  // Check for text lines first (TEL, IMG|BEGIN, IMG|ERROR)
  // We detect text lines by looking for newline chars and ASCII content
  // Binary JPEG data starts after IMG|BEGIN and arrives as raw bytes

  if (photoState == PHOTO_RECEIVING && imgBuf != nullptr) {
    // Receiving raw JPEG bytes
    size_t toWrite = len;
    if (imgBufLen + toWrite > imgExpectedLen) {
      toWrite = imgExpectedLen - imgBufLen;
    }
    if (toWrite > 0) {
      memcpy(imgBuf + imgBufLen, data, toWrite);
      imgBufLen += toWrite;
    }
    if (imgBufLen >= imgExpectedLen) {
      photoState = PHOTO_DONE;
      photoStatusMsg = "Photo received!";
      // If SD is available save it
      if (sdReady) {
        char fname[32];
        snprintf(fname, sizeof(fname), "/photo_%03d.jpg", photoCounter++);
        File f = SD.open(fname, FILE_WRITE);
        if (f) {
          f.write(imgBuf, imgBufLen);
          f.close();
          photoStatusMsg = String("Saved ") + fname;
        }
      }
    } else {
      char pct[24];
      snprintf(pct, sizeof(pct), "Receiving %d%%", (int)(100 * imgBufLen / imgExpectedLen));
      photoStatusMsg = String(pct);
    }
    return;
  }

  // Text-mode: buffer lines
  for (size_t i = 0; i < len; i++) {
    char c = (char)data[i];
    if (c == '\n') {
      rxLineBuf.trim();
      if (rxLineBuf.length() > 0) {
        if (rxLineBuf.startsWith("IMG|")) {
          handleImgHeader(rxLineBuf);
        } else {
          parseTelemetryLine(rxLineBuf);
        }
      }
      rxLineBuf = "";
    } else {
      rxLineBuf += c;
    }
  }
}

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    std::string v = pCharacteristic->getValue();
    if (v.empty()) return;
    handleRxData((const uint8_t *)v.data(), v.size());
  }
};

// ── Send commands to Pi ────────────────────────────────────────────────
static void sendState() {
  char buf[96];
  snprintf(buf, sizeof(buf), "CMD|VEL|%.3f|%.3f|%.3f\n", cmdVx, cmdVy, cmdWz);
  notifyLine(String(buf));
  snprintf(buf, sizeof(buf), "CMD|GMB|%.1f|%.1f\n", gimbalPan, gimbalTilt);
  notifyLine(String(buf));
  snprintf(buf, sizeof(buf), "CMD|FOL|%d\n", followEnabled ? 1 : 0);
  notifyLine(String(buf));
  snprintf(buf, sizeof(buf), "CMD|GMODE|%d\n", gimbalMode ? 1 : 0);
  notifyLine(String(buf));
  notifyLine("CMD|PING\n");
}

// ── Keyboard handling ──────────────────────────────────────────────────
static void handleKeyboard() {
  Keyboard_Class::KeysState status = M5Cardputer.Keyboard.keysState();

  // ── Panel switching: 1 / 2 / 3  or  Tab to cycle ──
  bool k1 = keyPressedAny('1', '!');
  bool k2 = keyPressedAny('2', '@');
  bool k3 = keyPressedAny('3', '#');
  // Also check status.word for number chars (more reliable with modifiers)
  for (char c : status.word) {
    if (c == '1' || c == '!') k1 = true;
    if (c == '2' || c == '@') k2 = true;
    if (c == '3' || c == '#') k3 = true;
  }
  if (k1 && !prev1Pressed && currentPanel != PANEL_TELEOP) {
    currentPanel = PANEL_TELEOP;
    panelDirty = true;
    M5.Speaker.tone(2000, 50);
  }
  if (k2 && !prev2Pressed && currentPanel != PANEL_ORIENT) {
    currentPanel = PANEL_ORIENT;
    panelDirty = true;
    M5.Speaker.tone(2000, 50); // Beep on switch
  }
  if (k3 && !prev3Pressed && currentPanel != PANEL_CAMERA) {
    currentPanel = PANEL_CAMERA;
    panelDirty = true;
    M5.Speaker.tone(2000, 50);
  }
  prev1Pressed = k1; prev2Pressed = k2; prev3Pressed = k3;
  // Tab cycles panels forward
  bool tabNow = status.tab;
  if (tabNow && !prevTabPressed) {
    currentPanel = (Panel)(((int)currentPanel + 1) % PANEL_COUNT);
    panelDirty = true;
    M5.Speaker.tone(2000, 50);
  }
  prevTabPressed = tabNow;

  // ── Drive (all panels) ──
  float targetVx = 0.0f, targetVy = 0.0f, targetWz = 0.0f;
  if (keyPressedAny('w', 'W')) targetVx += 0.25f;
  if (keyPressedAny('s', 'S')) targetVx -= 0.25f;
  if (keyPressedAny('j', 'J')) targetVy += 0.20f;
  if (keyPressedAny('l', 'L')) targetVy -= 0.20f;
  if (keyPressedAny('a', 'A')) targetWz += 1.00f;
  if (keyPressedAny('d', 'D')) targetWz -= 1.00f;
  if (keyPressedAny('x', 'X') || status.space || status.del) {
    targetVx = 0.0f; targetVy = 0.0f; targetWz = 0.0f;
  }
  cmdVx = targetVx; cmdVy = targetVy; cmdWz = targetWz;

  // ── Follow / Gimbal toggle ──
  bool fPressed = keyPressedAny('f', 'F');
  bool gPressed = keyPressedAny('g', 'G');
  if (fPressed && !prevFPressed) followEnabled = !followEnabled;
  if (gPressed && !prevGPressed) gimbalMode = !gimbalMode;
  prevFPressed = fPressed; prevGPressed = gPressed;

  // ── Gimbal fine control (Fn + keys) ──
  if (M5Cardputer.Keyboard.isChange() && M5Cardputer.Keyboard.isPressed() && status.fn) {
    for (char c : status.word) {
      if (c == ';' || c == ':') gimbalTilt = clampf(gimbalTilt + 3.0f, 0.0f, 110.0f);
      if (c == '.' || c == '>') gimbalTilt = clampf(gimbalTilt - 3.0f, 0.0f, 110.0f);
      if (c == ',' || c == '<') gimbalPan  = clampf(gimbalPan  - 3.0f, 0.0f, 180.0f);
      if (c == '/' || c == '?') gimbalPan  = clampf(gimbalPan  + 3.0f, 0.0f, 180.0f);
    }
    if (status.tab) { gimbalPan = 90.0f; gimbalTilt = 45.0f; }
  }

  // ── Photo capture: P key (or Enter on camera panel) ──
  bool pPressed = keyPressedAny('p', 'P');
  if (currentPanel == PANEL_CAMERA && status.enter) pPressed = true;
  if (pPressed && !prevPPressed) {
    if (photoState != PHOTO_RECEIVING) {
      photoState = PHOTO_REQUESTED;
      photoStatusMsg = "Requesting photo...";
      notifyLine("CMD|PHOTO\n");
      panelDirty = true;
    }
  }
  prevPPressed = pPressed;
}

// ── Drawing: header bar ────────────────────────────────────────────────
static void drawHeader() {
  M5Cardputer.Display.fillRect(0, 0, DISP_W, HEADER_H, TFT_NAVY);
  M5Cardputer.Display.setTextColor(WHITE, TFT_NAVY);
  M5Cardputer.Display.setCursor(2, 1);
  M5Cardputer.Display.setTextSize(1);
  String hdr = String(centralConnected ? "BT:OK" : "BT:--");
  hdr += "  " + localMac.substring(localMac.length() > 8 ? localMac.length() - 8 : 0);
  if (telFollow) hdr += " [F]";
  if (telGimbalMode) hdr += " [G]";
  M5Cardputer.Display.print(hdr);
}

// ── Drawing: tab bar ───────────────────────────────────────────────────
static void drawTabBar() {
  int tabW = DISP_W / PANEL_COUNT;
  int y = DISP_H - TAB_H;
  const char *labels[] = { "1:Teleop", "2:Orient", "3:Camera" };
  for (int i = 0; i < PANEL_COUNT; i++) {
    uint16_t bg = (i == (int)currentPanel) ? TFT_BLUE : TFT_DARKGREY;
    uint16_t fg = (i == (int)currentPanel) ? TFT_YELLOW : TFT_WHITE;
    M5Cardputer.Display.fillRect(i * tabW, y, tabW, TAB_H, bg);
    M5Cardputer.Display.setTextColor(fg, bg);
    int tw = strlen(labels[i]) * 6;
    M5Cardputer.Display.setCursor(i * tabW + (tabW - tw) / 2, y + 2);
    M5Cardputer.Display.print(labels[i]);
  }
}

// ── Panel 1: Teleop ────────────────────────────────────────────────────
static void drawPanelTeleop() {
  if (panelDirty) {
    M5Cardputer.Display.fillRect(0, BODY_Y, DISP_W, BODY_H, BLACK);
  }
  static String prev[7];
  String line[7];
  char buf[64];

  line[0] = String("Follow:") + (telFollow ? "ON " : "OFF") + "  Gimbal:" + (telGimbalMode ? "ON " : "OFF");
  snprintf(buf, sizeof(buf), "Spd %.2f V(%.2f %.2f %.2f)", telSpeed, telVx, telVy, telWz);
  line[1] = String(buf);
  snprintf(buf, sizeof(buf), "Gyro %.2f  Hdg %.1f", telGyro, telHeading);
  line[2] = String(buf);
  snprintf(buf, sizeof(buf), "Compass %.1f", telCompass);
  line[3] = String(buf);
  snprintf(buf, sizeof(buf), "Pan %.0f  Tilt %.0f", gimbalPan, gimbalTilt);
  line[4] = String(buf);
  snprintf(buf, sizeof(buf), "Roll %.1f  Pitch %.1f", telRoll, telPitch);
  line[5] = String(buf);
  line[6] = "WS:Dr AD:Tn JL:Str Tab:Sw";

  M5Cardputer.Display.setTextColor(TFT_GREEN, BLACK);
  for (int i = 0; i < 7; i++) {
    if (!panelDirty && line[i] == prev[i]) continue;
    int y = BODY_Y + i * 14;
    if (y + 12 > DISP_H - TAB_H) break;
    M5Cardputer.Display.fillRect(0, y, DISP_W, 13, BLACK);
    M5Cardputer.Display.setCursor(2, y + 1);
    M5Cardputer.Display.print(line[i]);
    prev[i] = line[i];
  }
}

// ── Panel 2: Orientation ───────────────────────────────────────────────
static void drawPanelOrient() {
  // Graphical compass in center of body area
  int cx = DISP_W / 2;
  int cy = BODY_Y + BODY_H / 2;
  int r  = min(BODY_H / 2 - 4, (DISP_W / 2) - 40);
  if (r < 20) r = 20;

  if (panelDirty) {
    M5Cardputer.Display.fillRect(0, BODY_Y, DISP_W, BODY_H, BLACK);
  }

  // Erase previous compass area
  M5Cardputer.Display.fillCircle(cx, cy, r + 6, BLACK);

  // Draw compass ring
  M5Cardputer.Display.drawCircle(cx, cy, r, TFT_DARKGREY);
  M5Cardputer.Display.drawCircle(cx, cy, r + 1, TFT_DARKGREY);

  // Cardinal labels around edge
  M5Cardputer.Display.setTextColor(TFT_CYAN, BLACK);
  M5Cardputer.Display.setCursor(cx - 2, cy - r - 10); M5Cardputer.Display.print("N");
  M5Cardputer.Display.setCursor(cx - 2, cy + r + 3);  M5Cardputer.Display.print("S");
  M5Cardputer.Display.setCursor(cx + r + 4, cy - 3);  M5Cardputer.Display.print("E");
  M5Cardputer.Display.setCursor(cx - r - 10, cy - 3); M5Cardputer.Display.print("W");

  // Heading needle
  float headRad = telHeading * DEG_TO_RAD;
  int nx = cx + (int)(sin(headRad) * (r - 4));
  int ny = cy - (int)(cos(headRad) * (r - 4));
  M5Cardputer.Display.drawLine(cx, cy, nx, ny, TFT_RED);
  // Rear half
  int bx = cx - (int)(sin(headRad) * (r / 3));
  int by = cy + (int)(cos(headRad) * (r / 3));
  M5Cardputer.Display.drawLine(cx, cy, bx, by, TFT_DARKGREY);

  // Center dot
  M5Cardputer.Display.fillCircle(cx, cy, 3, TFT_RED);

  // Heading text left side
  char buf[32];
  M5Cardputer.Display.setTextColor(TFT_GREEN, BLACK);
  snprintf(buf, sizeof(buf), "H:%.0f", telHeading);
  M5Cardputer.Display.fillRect(2, BODY_Y + 2, 56, 12, BLACK);
  M5Cardputer.Display.setCursor(2, BODY_Y + 2);
  M5Cardputer.Display.print(buf);

  // Compass text
  snprintf(buf, sizeof(buf), "C:%.0f", telCompass);
  M5Cardputer.Display.fillRect(2, BODY_Y + 16, 56, 12, BLACK);
  M5Cardputer.Display.setCursor(2, BODY_Y + 16);
  M5Cardputer.Display.print(buf);

  // Roll / Pitch right side
  snprintf(buf, sizeof(buf), "R:%.1f", telRoll);
  M5Cardputer.Display.fillRect(DISP_W - 60, BODY_Y + 2, 58, 12, BLACK);
  M5Cardputer.Display.setCursor(DISP_W - 58, BODY_Y + 2);
  M5Cardputer.Display.print(buf);

  snprintf(buf, sizeof(buf), "P:%.1f", telPitch);
  M5Cardputer.Display.fillRect(DISP_W - 60, BODY_Y + 16, 58, 12, BLACK);
  M5Cardputer.Display.setCursor(DISP_W - 58, BODY_Y + 16);
  M5Cardputer.Display.print(buf);

  // Gyro bottom‑left
  snprintf(buf, sizeof(buf), "Gz:%.2f", telGyro);
  M5Cardputer.Display.fillRect(2, BODY_Y + BODY_H - 14, 70, 12, BLACK);
  M5Cardputer.Display.setCursor(2, BODY_Y + BODY_H - 14);
  M5Cardputer.Display.print(buf);
}

// ── Panel 3: Camera ────────────────────────────────────────────────────
static void drawPanelCamera() {
  if (panelDirty) {
    M5Cardputer.Display.fillRect(0, BODY_Y, DISP_W, BODY_H, BLACK);
  }

  // If we have a received image and haven't displayed it yet, draw JPEG
  if (photoState == PHOTO_DONE && imgBuf && imgBufLen > 0 && !imgDisplayed) {
    // Display the JPEG thumbnail centred in body area
    // M5GFX drawJpg can decode in place
    int maxW = DISP_W;
    int maxH = BODY_H - 16;  // leave room for status text
    M5Cardputer.Display.fillRect(0, BODY_Y, DISP_W, BODY_H - 14, BLACK);
    M5Cardputer.Display.drawJpg(imgBuf, imgBufLen, 0, BODY_Y, maxW, maxH, 0, 0, JPEG_DIV_NONE);
    imgDisplayed = true;
  }

  // Status message at bottom of body
  static String prevStatus = "";
  if (panelDirty || photoStatusMsg != prevStatus) {
    int sy = BODY_Y + BODY_H - 13;
    M5Cardputer.Display.fillRect(0, sy, DISP_W, 12, BLACK);
    uint16_t col = TFT_WHITE;
    if (photoState == PHOTO_ERROR) col = TFT_RED;
    else if (photoState == PHOTO_DONE) col = TFT_GREEN;
    else if (photoState == PHOTO_RECEIVING) col = TFT_YELLOW;
    M5Cardputer.Display.setTextColor(col, BLACK);
    M5Cardputer.Display.setCursor(2, sy);
    M5Cardputer.Display.print(photoStatusMsg.substring(0, 38));
    prevStatus = photoStatusMsg;
  }

  // If idle draw instructions
  if (photoState == PHOTO_IDLE && panelDirty) {
    M5Cardputer.Display.setTextColor(TFT_CYAN, BLACK);
    int iy = BODY_Y + BODY_H / 2 - 12;
    M5Cardputer.Display.setCursor(30, iy);
    M5Cardputer.Display.print("Press  P  or  ENTER");
    M5Cardputer.Display.setCursor(40, iy + 14);
    M5Cardputer.Display.print("to capture photo");
  }
}

// ── Main draw dispatcher ───────────────────────────────────────────────
static void drawUi() {
  M5Cardputer.Display.startWrite();

  drawHeader();
  drawTabBar();

  switch (currentPanel) {
    case PANEL_TELEOP: drawPanelTeleop(); break;
    case PANEL_ORIENT: drawPanelOrient(); break;
    case PANEL_CAMERA: drawPanelCamera(); break;
    default: break;
  }

  panelDirty = false;
  M5Cardputer.Display.endWrite();
}

// ── Setup ──────────────────────────────────────────────────────────────
void setup() {
  auto cfg = M5.config();
  M5Cardputer.begin(cfg, true);

  M5Cardputer.Display.setRotation(1);
  M5Cardputer.Display.setTextSize(1);
  M5Cardputer.Display.fillScreen(BLACK);

  // Allocate image buffer (prefer PSRAM if available)
  imgBuf = (uint8_t *)ps_malloc(IMG_BUF_MAX);
  if (!imgBuf) imgBuf = (uint8_t *)malloc(IMG_BUF_MAX);

  // Try to initialise SD card (Cardputer uses SPI SD on GPIO12 CS)
  // M5Cardputer may not have SD slot populated; be tolerant
  sdReady = SD.begin(12, SPI, 10000000);

  // BLE init
  BLEDevice::init(DEVICE_NAME);
  BLEDevice::setMTU(185);
  localMac = String(BLEDevice::getAddress().toString().c_str());
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService *service = server->createService(SERVICE_UUID);
  txNotifyChar = service->createCharacteristic(TX_NOTIFY_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  txNotifyChar->addDescriptor(new BLE2902());
  rxWriteChar = service->createCharacteristic(RX_WRITE_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rxWriteChar->setCallbacks(new RxCallbacks());
  service->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  adv->setMaxPreferred(0x12);
  adv->start();

  panelDirty = true;
  drawUi();
}

// ── Loop ───────────────────────────────────────────────────────────────
void loop() {
  M5Cardputer.update();
  handleKeyboard();

  uint32_t now = millis();
  if (now - lastSendMs >= 100) {
    lastSendMs = now;
    sendState();
  }
  if (now - lastUiMs >= 150) {
    lastUiMs = now;
    drawUi();
  }
  delay(5);
}
