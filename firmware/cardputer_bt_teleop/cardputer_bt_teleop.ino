#include <M5Cardputer.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

static const char *DEVICE_NAME = "RaspbotCardputer";
static const char *SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char *TX_NOTIFY_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"; // Cardputer -> Pi
static const char *RX_WRITE_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";  // Pi -> Cardputer

BLECharacteristic *txNotifyChar = nullptr;
BLECharacteristic *rxWriteChar = nullptr;
volatile bool centralConnected = false;
volatile uint32_t connectTimeMs = 0;
String localMac = "";

float cmdVx = 0.0f;
float cmdVy = 0.0f;
float cmdWz = 0.0f;
float gimbalPan = 90.0f;
float gimbalTilt = 45.0f;
bool followEnabled = false;
bool gimbalMode = false;

float telVx = 0.0f;
float telVy = 0.0f;
float telWz = 0.0f;
float telSpeed = 0.0f;
float telGyro = 0.0f;
float telCompass = 0.0f;
float telHeading = 0.0f;
bool telFollow = false;
bool telGimbalMode = false;

uint32_t lastSendMs = 0;
uint32_t lastUiMs = 0;
bool prevFPressed = false;
bool prevGPressed = false;

static bool keyPressedAny(char a, char b = '\0') {
  return M5Cardputer.Keyboard.isKeyPressed(a) || (b != '\0' && M5Cardputer.Keyboard.isKeyPressed(b));
}

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

static void parseTelemetryLine(const String &line) {
  // TEL|vx|vy|wz|speed|gyro|compass|heading|follow|gmode
  if (!line.startsWith("TEL|")) {
    return;
  }

  String fields[10];
  int fieldCount = 0;
  int start = 0;
  while (fieldCount < 10) {
    int sep = line.indexOf('|', start);
    if (sep < 0) {
      fields[fieldCount++] = line.substring(start);
      break;
    }
    fields[fieldCount++] = line.substring(start, sep);
    start = sep + 1;
  }
  if (fieldCount < 10) {
    return;
  }

  telVx = fields[1].toFloat();
  telVy = fields[2].toFloat();
  telWz = fields[3].toFloat();
  telSpeed = fields[4].toFloat();
  telGyro = fields[5].toFloat();
  telCompass = fields[6].toFloat();
  telHeading = fields[7].toFloat();
  telFollow = fields[8].toInt() == 1;
  telGimbalMode = fields[9].toInt() == 1;
}

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    std::string v = pCharacteristic->getValue();
    if (v.empty()) {
      return;
    }
    String payload = String(v.c_str());
    int start = 0;
    while (true) {
      int nl = payload.indexOf('\n', start);
      if (nl < 0) {
        String line = payload.substring(start);
        line.trim();
        if (line.length() > 0) {
          parseTelemetryLine(line);
        }
        break;
      }
      String line = payload.substring(start, nl);
      line.trim();
      if (line.length() > 0) {
        parseTelemetryLine(line);
      }
      start = nl + 1;
    }
  }
};

static float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

static void notifyLine(const String &line) {
  if (!bleReady() || txNotifyChar == nullptr) {
    return;
  }
  txNotifyChar->setValue((uint8_t *)line.c_str(), line.length());
  txNotifyChar->notify();
}

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

static void handleKeyboard() {
  Keyboard_Class::KeysState status = M5Cardputer.Keyboard.keysState();

  float targetVx = 0.0f;
  float targetVy = 0.0f;
  float targetWz = 0.0f;

  if (keyPressedAny('w', 'W')) targetVx += 0.25f;
  if (keyPressedAny('s', 'S')) targetVx -= 0.25f;
  if (keyPressedAny('j', 'J')) targetVy += 0.20f;
  if (keyPressedAny('l', 'L')) targetVy -= 0.20f;
  if (keyPressedAny('a', 'A')) targetWz += 1.00f;
  if (keyPressedAny('d', 'D')) targetWz -= 1.00f;

  if (keyPressedAny('x', 'X') || status.space || status.del) {
    targetVx = 0.0f;
    targetVy = 0.0f;
    targetWz = 0.0f;
  }

  cmdVx = targetVx;
  cmdVy = targetVy;
  cmdWz = targetWz;

  bool fPressed = keyPressedAny('f', 'F');
  bool gPressed = keyPressedAny('g', 'G');

  if (fPressed && !prevFPressed) {
    followEnabled = !followEnabled;
  }
  if (gPressed && !prevGPressed) {
    gimbalMode = !gimbalMode;
  }
  prevFPressed = fPressed;
  prevGPressed = gPressed;

  if (M5Cardputer.Keyboard.isChange() && M5Cardputer.Keyboard.isPressed() && status.fn) {
    for (char c : status.word) {
      if (c == ';' || c == ':') gimbalTilt = clampf(gimbalTilt + 3.0f, 0.0f, 110.0f);
      if (c == '.' || c == '>') gimbalTilt = clampf(gimbalTilt - 3.0f, 0.0f, 110.0f);
      if (c == ',' || c == '<') gimbalPan = clampf(gimbalPan - 3.0f, 0.0f, 180.0f);
      if (c == '/' || c == '?') gimbalPan = clampf(gimbalPan + 3.0f, 0.0f, 180.0f);
    }
    if (status.tab) {
      gimbalPan = 90.0f;
      gimbalTilt = 45.0f;
    }
  }
}

static void drawUi() {
  static String prev[8];
  String line[8];

  line[0] = String("BT:") + (centralConnected ? "CONNECTED" : "WAITING") + "  MAC:" + localMac;
  line[1] = String("Follow:") + (telFollow ? "ON" : "OFF") + "  Gimbal:" + (telGimbalMode ? "ON" : "OFF");

  char buf[96];
  snprintf(buf, sizeof(buf), "SPD %.2f  V(%.2f %.2f %.2f)", telSpeed, telVx, telVy, telWz);
  line[2] = String(buf);
  snprintf(buf, sizeof(buf), "GyroZ %.3f", telGyro);
  line[3] = String(buf);
  snprintf(buf, sizeof(buf), "Compass %.1f", telCompass);
  line[4] = String(buf);
  snprintf(buf, sizeof(buf), "Heading %.1f", telHeading);
  line[5] = String(buf);
  snprintf(buf, sizeof(buf), "Pan %.1f Tilt %.1f", gimbalPan, gimbalTilt);
  line[6] = String(buf);
  line[7] = "Hold WASD/JL move, F/G toggle, Fn+,./; gimbal";

  M5Cardputer.Display.startWrite();
  M5Cardputer.Display.setTextColor(WHITE, BLACK);
  for (int i = 0; i < 8; ++i) {
    if (line[i] == prev[i]) {
      continue;
    }
    int y = 2 + i * 16;
    M5Cardputer.Display.fillRect(0, y, 240, 14, BLACK);
    M5Cardputer.Display.setCursor(2, y);
    M5Cardputer.Display.print(line[i]);
    prev[i] = line[i];
  }
  M5Cardputer.Display.endWrite();
}

void setup() {
  auto cfg = M5.config();
  M5Cardputer.begin(cfg, true);

  M5Cardputer.Display.setRotation(1);
  M5Cardputer.Display.setTextSize(1);

  BLEDevice::init(DEVICE_NAME);
  BLEDevice::setMTU(185);
  localMac = String(BLEDevice::getAddress().toString().c_str());
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService *service = server->createService(SERVICE_UUID);

  txNotifyChar = service->createCharacteristic(TX_NOTIFY_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  txNotifyChar->addDescriptor(new BLE2902());

  rxWriteChar = service->createCharacteristic(RX_WRITE_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rxWriteChar->setCallbacks(new RxCallbacks());

  service->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  adv->setMaxPreferred(0x12);
  adv->start();

  drawUi();
}

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
