/*
 * Raspbot — Pico RP2040 + BNO055 9-DOF IMU bridge
 *
 * Reads a BNO055 via I2C0 (GP4=SDA, GP5=SCL) and streams fused
 * orientation data over USB serial to the Raspberry Pi 5.
 *
 * The BNO055's on-chip Cortex-M0 fusion engine provides:
 *   • Absolute orientation as a quaternion (no drift)
 *   • Euler angles (heading / roll / pitch)
 *   • Calibrated accelerometer, gyroscope, magnetometer
 *   • Temperature
 *   • Per-sensor calibration status (0–3 each)
 *
 * ── Wire protocol ──────────────────────────────────────────────────
 *
 *   Each line is '\n'-terminated.  Fields are comma-separated.
 *
 *   Fused IMU line (sent at configured rate, default 100 Hz):
 *     $BNO,<qw>,<qx>,<qy>,<qz>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,
 *          <mx>,<my>,<mz>,<heading>,<roll>,<pitch>,<temp>,
 *          <cal_sys>,<cal_gyro>,<cal_accel>,<cal_mag>,<ms>\n
 *
 *       qw/qx/qy/qz : fused quaternion (unit, float)
 *       ax/ay/az     : linear acceleration, gravity removed (m/s², float)
 *       gx/gy/gz     : gyroscope (rad/s, float)
 *       mx/my/mz     : magnetometer (µT, float)
 *       heading      : Euler heading / yaw   (degrees, 0–360)
 *       roll         : Euler roll             (degrees, ±180)
 *       pitch        : Euler pitch            (degrees, ±90)
 *       temp         : chip temperature       (°C, int)
 *       cal_sys      : system calibration     (0–3, 3=fully calibrated)
 *       cal_gyro     : gyro calibration       (0–3)
 *       cal_accel    : accel calibration      (0–3)
 *       cal_mag      : magnetometer calibration (0–3)
 *       ms           : millis() timestamp
 *
 *   Gravity vector line (sent at 10 Hz, useful for tilt compensation):
 *     $GRV,<gx>,<gy>,<gz>\n
 *
 *       gx/gy/gz : gravity vector in body frame (m/s²)
 *
 *   On startup or when the host sends '?':
 *     $INFO,pico_bno055,<HZ>,<version>\n
 *
 *   Host commands (single character):
 *     '?'  → re-send $INFO
 *     'C'  → reset BNO055 calibration (rarely needed)
 *     'S'  → save current calibration offsets to BNO055 (persists on chip)
 *     'L'  → load/restore calibration offsets from flash
 *     'N'  → switch to NDOF mode   (9-DOF fusion, absolute orientation)
 *     'I'  → switch to IMU mode    (6-DOF fusion, no magnetometer)
 *     '1'  → set output rate to  50 Hz
 *     '2'  → set output rate to 100 Hz (default)
 *     'D'  → toggle debug output ($DBG lines)
 *
 * ── Required Arduino libraries ─────────────────────────────────────
 *
 *   • Adafruit BNO055 (install via Library Manager)
 *   • Adafruit Unified Sensor (dependency, auto-installed)
 *   • Wire (built-in)
 *
 * ── Board setup (Arduino IDE) ──────────────────────────────────────
 *
 *   1. Install "Raspberry Pi Pico/RP2040" board package:
 *      • Add to Additional Boards Manager URLs:
 *        https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
 *      • Board Manager → install "Raspberry Pi Pico/RP2040 by Earle F. Philhower"
 *   2. Select board: "Raspberry Pi Pico"
 *   3. Tools → USB Stack → "Pico SDK"
 *   4. Upload via USB
 *
 *   The Pi will see the Pico as /dev/ttyACM0 or /dev/ttyACM1.
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// ── Tunables ─────────────────────────────────────────────────────────
#define FIRMWARE_VERSION   "1.0.0"
#define DEFAULT_HZ         100
#define SERIAL_BAUD        115200
#define BNO055_ADDRESS     0x28     // ADR pin LOW or floating
#define I2C_SDA_PIN        4        // GP4, physical pin 6
#define I2C_SCL_PIN        5        // GP5, physical pin 7
#define LED_PIN            13       // GP13, optional status LED (physical pin 17)
#define GRAVITY_HZ         10       // $GRV output rate

// ── BNO055 ───────────────────────────────────────────────────────────
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS, &Wire);

// ── Globals ──────────────────────────────────────────────────────────
static unsigned long g_interval_us   = 1000000UL / DEFAULT_HZ;
static unsigned long g_last_send_us  = 0;
static unsigned long g_last_grv_us   = 0;
static unsigned long g_grv_interval  = 1000000UL / GRAVITY_HZ;
static bool          g_debug         = false;
static bool          g_bno_ok        = false;

// Calibration offsets storage (saved to Pico flash via EEPROM emulation)
// The BNO055 has 22 bytes of calibration offsets.
#include <EEPROM.h>
#define CAL_EEPROM_ADDR    0
#define CAL_MAGIC          0xB055CA1B   // magic number to validate stored cal
#define CAL_DATA_SIZE      22

struct CalStorage {
  uint32_t magic;
  uint8_t  data[CAL_DATA_SIZE];
  uint8_t  mode;   // Adafruit_BNO055::adafruit_bno055_opmode_t
};

// ── LED helper ───────────────────────────────────────────────────────
static inline void ledOn()  { digitalWrite(LED_PIN, HIGH); }
static inline void ledOff() { digitalWrite(LED_PIN, LOW);  }

static void ledBlink(int count, int on_ms, int off_ms) {
  for (int i = 0; i < count; i++) {
    ledOn();  delay(on_ms);
    ledOff(); delay(off_ms);
  }
}

// ── Send $INFO ───────────────────────────────────────────────────────
static void sendInfo() {
  int hz = (int)(1000000UL / g_interval_us);
  Serial.print("$INFO,pico_bno055,");
  Serial.print(hz);
  Serial.print(",");
  Serial.println(FIRMWARE_VERSION);
}

// ── Save calibration to EEPROM (flash) ───────────────────────────────
static bool saveCalibration() {
  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);

  if (sys < 1) {
    Serial.println("$CAL,save_fail,system_not_calibrated");
    return false;
  }

  // Read the sensor offsets from the BNO055
  adafruit_bno055_offsets_t offsets;
  if (!bno.getSensorOffsets(offsets)) {
    Serial.println("$CAL,save_fail,cannot_read_offsets");
    return false;
  }

  CalStorage cal;
  cal.magic = CAL_MAGIC;
  cal.mode  = bno.getMode();
  memcpy(cal.data, &offsets, CAL_DATA_SIZE);

  EEPROM.begin(256);
  EEPROM.put(CAL_EEPROM_ADDR, cal);
  EEPROM.commit();
  EEPROM.end();

  Serial.print("$CAL,saved,sys=");
  Serial.print(sys);
  Serial.print(",gyro=");
  Serial.print(gyro);
  Serial.print(",accel=");
  Serial.print(accel);
  Serial.print(",mag=");
  Serial.println(mag);

  return true;
}

// ── Load calibration from EEPROM (flash) ─────────────────────────────
static bool loadCalibration() {
  CalStorage cal;

  EEPROM.begin(256);
  EEPROM.get(CAL_EEPROM_ADDR, cal);
  EEPROM.end();

  if (cal.magic != CAL_MAGIC) {
    Serial.println("$CAL,load_fail,no_stored_calibration");
    return false;
  }

  adafruit_bno055_offsets_t offsets;
  memcpy(&offsets, cal.data, CAL_DATA_SIZE);

  // Must be in CONFIG mode to set offsets
  bno.setMode(Adafruit_BNO055::OPERATION_MODE_CONFIG);
  delay(25);

  bno.setSensorOffsets(offsets);

  // Switch back to fusion mode
  bno.setMode(Adafruit_BNO055::OPERATION_MODE_NDOF);
  delay(20);

  Serial.println("$CAL,loaded");
  return true;
}

// ── BNO055 initialization ────────────────────────────────────────────
static bool initBNO055() {
  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();
  Wire.setClock(400000);  // 400 kHz fast mode

  if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF)) {
    return false;
  }

  delay(100);

  // Use external crystal for better accuracy (the GY-BNO055 board has one)
  bno.setExtCrystalUse(true);

  return true;
}

// ── Setup ────────────────────────────────────────────────────────────
void setup() {
  pinMode(LED_PIN, OUTPUT);
  ledOff();

  Serial.begin(SERIAL_BAUD);
  // Wait up to 3 s for serial host
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0 < 3000)) { delay(10); }

  Serial.println("$MSG,Pico BNO055 bridge starting...");

  // ── Init BNO055 with retries ──
  for (int attempt = 1; attempt <= 5; attempt++) {
    Serial.print("$MSG,BNO055 init attempt ");
    Serial.println(attempt);

    if (initBNO055()) {
      g_bno_ok = true;
      Serial.println("$MSG,BNO055 initialized OK");
      break;
    }

    Serial.println("$ERR,bno055_init_failed");
    ledBlink(3, 100, 100);  // error blink
    delay(500);
  }

  if (!g_bno_ok) {
    Serial.println("$ERR,bno055_init_failed_all_attempts");
    // Keep running — we'll retry in the main loop
  }

  // ── Try loading saved calibration ──
  if (g_bno_ok) {
    loadCalibration();
  }

  sendInfo();
  ledBlink(2, 150, 150);  // ready blink
  g_last_send_us = micros();
  g_last_grv_us  = micros();
}

// ── Main loop ────────────────────────────────────────────────────────
void loop() {

  // ── Handle host commands ──
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case '?':
        sendInfo();
        break;
      case 'C':
        // Full reset: re-init BNO055, clears all calibration
        Serial.println("$CAL,resetting");
        bno.setMode(Adafruit_BNO055::OPERATION_MODE_CONFIG);
        delay(25);
        bno.setMode(Adafruit_BNO055::OPERATION_MODE_NDOF);
        delay(20);
        Serial.println("$CAL,reset_done");
        break;
      case 'S':
        saveCalibration();
        break;
      case 'L':
        loadCalibration();
        break;
      case 'N':
        bno.setMode(Adafruit_BNO055::OPERATION_MODE_NDOF);
        delay(20);
        Serial.println("$MODE,NDOF");
        break;
      case 'I':
        bno.setMode(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
        delay(20);
        Serial.println("$MODE,IMU");
        break;
      case '1':
        g_interval_us = 1000000UL / 50;
        Serial.println("$RATE,50");
        break;
      case '2':
        g_interval_us = 1000000UL / 100;
        Serial.println("$RATE,100");
        break;
      case 'D':
        g_debug = !g_debug;
        Serial.print("$DBG,");
        Serial.println(g_debug ? "on" : "off");
        break;
      default:
        break;
    }
  }

  // ── If BNO055 not initialized, retry periodically ──
  if (!g_bno_ok) {
    static unsigned long last_retry = 0;
    if (millis() - last_retry > 2000) {
      last_retry = millis();
      if (initBNO055()) {
        g_bno_ok = true;
        Serial.println("$MSG,BNO055 initialized OK (retry)");
        loadCalibration();
        sendInfo();
      }
    }
    return;
  }

  unsigned long now_us = micros();

  // ── Gravity vector at lower rate ──
  if ((now_us - g_last_grv_us) >= g_grv_interval) {
    g_last_grv_us = now_us;

    imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    Serial.print("$GRV,");
    Serial.print(gravity.x(), 3);  Serial.print(",");
    Serial.print(gravity.y(), 3);  Serial.print(",");
    Serial.println(gravity.z(), 3);
  }

  // ── Main IMU data rate limit ──
  if ((now_us - g_last_send_us) < g_interval_us) {
    return;
  }
  g_last_send_us = now_us;

  // ── Read all BNO055 data ──

  // Fused quaternion (absolute orientation)
  imu::Quaternion quat = bno.getQuat();

  // Linear acceleration (gravity removed)
  imu::Vector<3> linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  // Gyroscope (rad/s)
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  // Magnetometer (µT)
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  // Euler angles (degrees) — heading(yaw), roll, pitch
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  // Temperature
  int8_t temp = bno.getTemp();

  // Calibration status (0-3 each)
  uint8_t cal_sys, cal_gyro, cal_accel, cal_mag;
  bno.getCalibration(&cal_sys, &cal_gyro, &cal_accel, &cal_mag);

  unsigned long now_ms = millis();

  // ── LED indicates calibration state ──
  if (cal_sys >= 2) {
    ledOn();   // solid = well calibrated
  } else {
    // Blink slowly when poorly calibrated
    digitalWrite(LED_PIN, (now_ms / 500) & 1);
  }

  // ── Build & send $BNO line ──
  // $BNO,qw,qx,qy,qz,ax,ay,az,gx,gy,gz,mx,my,mz,heading,roll,pitch,temp,
  //      cal_sys,cal_gyro,cal_accel,cal_mag,ms
  Serial.print("$BNO,");
  Serial.print(quat.w(), 4);     Serial.print(",");
  Serial.print(quat.x(), 4);     Serial.print(",");
  Serial.print(quat.y(), 4);     Serial.print(",");
  Serial.print(quat.z(), 4);     Serial.print(",");
  Serial.print(linAccel.x(), 3); Serial.print(",");
  Serial.print(linAccel.y(), 3); Serial.print(",");
  Serial.print(linAccel.z(), 3); Serial.print(",");
  Serial.print(gyro.x(), 4);     Serial.print(",");
  Serial.print(gyro.y(), 4);     Serial.print(",");
  Serial.print(gyro.z(), 4);     Serial.print(",");
  Serial.print(mag.x(), 2);      Serial.print(",");
  Serial.print(mag.y(), 2);      Serial.print(",");
  Serial.print(mag.z(), 2);      Serial.print(",");
  Serial.print(euler.x(), 2);    Serial.print(",");  // heading (0-360)
  Serial.print(euler.z(), 2);    Serial.print(",");  // roll (±180)
  Serial.print(euler.y(), 2);    Serial.print(",");  // pitch (±90)
  Serial.print(temp);            Serial.print(",");
  Serial.print(cal_sys);         Serial.print(",");
  Serial.print(cal_gyro);        Serial.print(",");
  Serial.print(cal_accel);       Serial.print(",");
  Serial.print(cal_mag);         Serial.print(",");
  Serial.println(now_ms);

  // ── Debug output ──
  if (g_debug) {
    // System status
    uint8_t sys_stat, self_test, sys_err;
    bno.getSystemStatus(&sys_stat, &self_test, &sys_err);
    Serial.print("$DBG,stat=");
    Serial.print(sys_stat);
    Serial.print(",self_test=");
    Serial.print(self_test);
    Serial.print(",err=");
    Serial.println(sys_err);
  }
}
