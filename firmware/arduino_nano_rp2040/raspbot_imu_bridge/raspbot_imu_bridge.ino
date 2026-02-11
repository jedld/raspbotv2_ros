/*
 * Raspbot — Arduino Nano RP2040 Connect sensor bridge
 *
 * Reads the on-board LSM6DSOX 6-axis IMU (accel + gyro + temperature),
 * an external BNO055 9-DOF sensor (via Wire/I2C0 on A4/A5), and
 * the MP34DT06JTR PDM microphone, then streams compact ASCII lines over
 * USB serial so a Raspberry Pi ROS 2 node can parse them.
 *
 * The LSM6DSOX uses an *internal* I2C bus (Wire1) — it does NOT
 * conflict with the external Wire (I2C0) bus used by the BNO055.
 *
 * ── Wire protocol ──────────────────────────────────────────────────
 *
 *   Each line is '\n'-terminated.  Fields are comma-separated.
 *
 *   On-board IMU line (sent at IMU_HZ, always):
 *     $IMU,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<temp>,<mic>,<ms>\n
 *
 *       ax/ay/az : accelerometer  (g,   float, ±16 g)
 *       gx/gy/gz : gyroscope      (°/s, float, ±2000 °/s)
 *       temp     : die temperature (°C,  float)
 *       mic      : microphone RMS level (0-32767, int)
 *       ms       : millis() timestamp
 *
 *   BNO055 fused line (sent at IMU_HZ when BNO055 is detected):
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
 *       temp         : BNO055 chip temperature (°C, int)
 *       cal_sys      : system calibration     (0–3, 3=fully calibrated)
 *       cal_gyro     : gyro calibration       (0–3)
 *       cal_accel    : accel calibration      (0–3)
 *       cal_mag      : magnetometer calibration (0–3)
 *       ms           : millis() timestamp
 *
 *   BNO055 gravity vector (sent at ~10 Hz when BNO055 is detected):
 *     $GRV,<gx>,<gy>,<gz>\n
 *
 *       gx/gy/gz : gravity vector in body frame (m/s²)
 *
 *   Audio data (sent when audio streaming is enabled via 'A' command):
 *     $AUD,<base64_pcm>\n
 *
 *       base64_pcm : base64-encoded 8-bit unsigned PCM at 8 kHz
 *                    512 samples per chunk (~64 ms)
 *
 *   On startup or when the host sends "?":
 *     $INFO,nano_rp2040,<IMU_HZ>,<version>,<bno055_detected>\n
 *
 *   The host can send single-character commands:
 *     '?'  → re-send $INFO
 *     'R'  → set RGB LED to red
 *     'G'  → set RGB LED to green
 *     'B'  → set RGB LED to blue
 *     'W'  → set RGB LED to white
 *     'O'  → turn RGB LED off
 *     'C'  → calibrate on-board gyro (hold still for 2 s)
 *     '1'  → set output rate to  50 Hz
 *     '2'  → set output rate to 100 Hz  (default)
 *     '3'  → set output rate to 200 Hz
 *     'A'  → enable audio streaming (8 kHz 8-bit unsigned PCM via $AUD lines)
 *     'a'  → disable audio streaming
 *     'S'  → save BNO055 calibration offsets (in RAM, reported to host)
 *     'L'  → restore BNO055 calibration offsets from RAM
 *     'N'  → switch BNO055 to NDOF mode  (9-DOF fusion, absolute orientation)
 *     'I'  → switch BNO055 to IMU mode   (6-DOF fusion, no magnetometer)
 *
 * ── Required Arduino libraries ─────────────────────────────────────
 *
 *   • Arduino_LSM6DSOX   (Board Manager "Arduino Mbed OS Nano" installs it)
 *   • PDM                (built-in with the Mbed core)
 *   • WiFiNINA           (needed only to drive the on-board RGB LED)
 *   • Adafruit BNO055    (install via Library Manager)
 *   • Adafruit Unified Sensor (dependency, auto-installed)
 *
 * ── Board Manager ──────────────────────────────────────────────────
 *
 *   Install "Arduino Mbed OS Nano Boards" from Board Manager, then
 *   select "Arduino Nano RP2040 Connect".
 *
 * ── BNO055 wiring ──────────────────────────────────────────────────
 *
 *   BNO055 VIN  → Nano 3.3V
 *   BNO055 GND  → Nano GND
 *   BNO055 SDA  → Nano A4  (Wire / I2C0 SDA)
 *   BNO055 SCL  → Nano A5  (Wire / I2C0 SCL)
 *   BNO055 ADR  → float or LOW  → address 0x28
 *
 * ── Flashing ───────────────────────────────────────────────────────
 *
 *   1. Connect the Nano RP2040 via USB to the Pi (or a computer)
 *   2. In Arduino IDE: Tools → Board → Arduino Nano RP2040 Connect
 *   3. Tools → Port → (the CDC serial port that appeared)
 *   4. Upload
 *
 *   Or via arduino-cli on the Pi:
 *     arduino-cli compile --fqbn arduino:mbed_nano:nanorp2040connect .
 *     arduino-cli upload  --fqbn arduino:mbed_nano:nanorp2040connect -p /dev/ttyACM0 .
 *
 *   The Pi will see it as /dev/ttyACM0 (or /dev/ttyACM1).
 */

#include <Arduino_LSM6DSOX.h>
#include <PDM.h>
#include <WiFiNINA.h>          // for the on-board RGB LED only
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// ── Tunables ─────────────────────────────────────────────────────────
#define FIRMWARE_VERSION   "2.0.0"
#define DEFAULT_HZ         100        // default output rate
#define SERIAL_BAUD        115200
#define MIC_SAMPLE_RATE    16000      // Hz
#define MIC_CHANNELS       1
#define MIC_BUF_SAMPLES    512
#define BNO055_ADDRESS     0x29       // ADR pin HIGH (or pulled up on GY-BNO055 board)
#define GRAVITY_HZ         10         // $GRV output rate

// ── BNO055 calibration RAM storage ───────────────────────────────────
#define CAL_DATA_SIZE      22

struct BnoCalData {
  bool    valid;
  uint8_t data[CAL_DATA_SIZE];
};
static BnoCalData g_bno_cal = { false, {0} };

// ── BNO055 ───────────────────────────────────────────────────────────
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS, &Wire);
static bool g_bno_ok = false;

// ── Globals ──────────────────────────────────────────────────────────
static volatile int     g_mic_rms      = 0;
static int16_t          g_mic_buf[MIC_BUF_SAMPLES];

static float  g_gyro_offset[3]  = {0.0f, 0.0f, 0.0f};
static float  g_accel_offset[3] = {0.0f, 0.0f, 0.0f};
static bool   g_calibrated      = false;

static unsigned long    g_interval_us  = 1000000UL / DEFAULT_HZ;
static unsigned long    g_last_send_us = 0;
static unsigned long    g_last_grv_us  = 0;
static unsigned long    g_grv_interval = 1000000UL / GRAVITY_HZ;

// ── Audio streaming ──────────────────────────────────────────────────
#define AUD_RING_SIZE      2048
#define AUD_CHUNK_SIZE     512

static volatile bool     g_audio_stream = false;
static uint8_t           g_aud_ring[AUD_RING_SIZE];
static volatile uint16_t g_aud_wr = 0;
static uint16_t          g_aud_rd = 0;

// ── Base64 encoder ───────────────────────────────────────────────────
static const char g_b64[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static void sendBase64(const uint8_t* data, int len) {
  for (int i = 0; i < len; i += 3) {
    int rem = len - i;
    uint32_t n = (uint32_t)data[i] << 16;
    if (rem > 1) n |= (uint32_t)data[i + 1] << 8;
    if (rem > 2) n |= data[i + 2];
    Serial.write(g_b64[(n >> 18) & 0x3F]);
    Serial.write(g_b64[(n >> 12) & 0x3F]);
    Serial.write((rem > 1) ? g_b64[(n >> 6) & 0x3F] : '=');
    Serial.write((rem > 2) ? g_b64[n & 0x3F] : '=');
  }
}

static float g_last_temp      = 0.0f;
static unsigned long g_last_temp_ms = 0;
#define TEMP_INTERVAL_MS  500

// ── BNO055 retry ─────────────────────────────────────────────────────
static unsigned long g_bno_retry_ms = 0;
#define BNO_RETRY_INTERVAL_MS  3000

// ── Microphone callback (ISR context) ────────────────────────────────
void onPDMdata() {
  int bytesAvailable = PDM.available();
  int samplesToRead  = bytesAvailable / 2;
  if (samplesToRead > MIC_BUF_SAMPLES) samplesToRead = MIC_BUF_SAMPLES;

  PDM.read(g_mic_buf, samplesToRead * 2);

  long long sum = 0;
  for (int i = 0; i < samplesToRead; i++) {
    long s = (long)g_mic_buf[i];
    sum += s * s;
  }
  if (samplesToRead > 0) {
    g_mic_rms = (int)sqrt((double)sum / samplesToRead);
  }

  if (g_audio_stream) {
    for (int i = 0; i < samplesToRead; i += 2) {
      int16_t s = g_mic_buf[i];
      uint8_t u8 = (uint8_t)((s >> 8) + 128);
      uint16_t wr = g_aud_wr;
      g_aud_ring[wr] = u8;
      g_aud_wr = (wr + 1) % AUD_RING_SIZE;
    }
  }
}

// ── RGB LED helper ───────────────────────────────────────────────────
void setRGB(uint8_t r, uint8_t g, uint8_t b) {
  WiFiDrv::analogWrite(25, 255 - r);
  WiFiDrv::analogWrite(26, 255 - g);
  WiFiDrv::analogWrite(27, 255 - b);
}

// ── Gyro calibration (on-board LSM6DSOX) ─────────────────────────────
void calibrateGyro() {
  setRGB(255, 165, 0);  // orange while calibrating

  float sx = 0, sy = 0, sz = 0;
  float sax = 0, say = 0, saz = 0;
  int   ng = 0, na = 0;
  unsigned long start = millis();

  Serial.println("$CAL,start");

  while (millis() - start < 2000) {
    float gx, gy, gz;
    if (IMU.gyroscopeAvailable() && IMU.readGyroscope(gx, gy, gz)) {
      sx += gx; sy += gy; sz += gz; ng++;
    }
    float ax, ay, az;
    if (IMU.accelerationAvailable() && IMU.readAcceleration(ax, ay, az)) {
      sax += ax; say += ay; saz += az; na++;
    }
    delay(5);
  }

  if (ng > 0) {
    g_gyro_offset[0] = sx / ng;
    g_gyro_offset[1] = sy / ng;
    g_gyro_offset[2] = sz / ng;
    g_calibrated = true;
  }

  if (na > 0) {
    g_accel_offset[0] = sax / na;
    g_accel_offset[1] = say / na;
    g_accel_offset[2] = (saz / na) - 1.0f;
  }

  Serial.print("$CAL,done,");
  Serial.print(ng);                      Serial.print(",");
  Serial.print(g_gyro_offset[0], 4);    Serial.print(",");
  Serial.print(g_gyro_offset[1], 4);    Serial.print(",");
  Serial.print(g_gyro_offset[2], 4);    Serial.print(",");
  Serial.print(g_accel_offset[0], 4);   Serial.print(",");
  Serial.print(g_accel_offset[1], 4);   Serial.print(",");
  Serial.println(g_accel_offset[2], 4);

  setRGB(0, 0, 0);
}

// ── BNO055 initialization ────────────────────────────────────────────
bool initBNO055() {
  if (!bno.begin(OPERATION_MODE_NDOF)) {
    return false;
  }
  delay(100);
  bno.setExtCrystalUse(true);
  return true;
}

// ── BNO055 save calibration ──────────────────────────────────────────
void bnoSaveCal() {
  if (!g_bno_ok) {
    Serial.println("$BNOCAL,save_fail,no_bno055");
    return;
  }

  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);

  if (sys < 1) {
    Serial.println("$BNOCAL,save_fail,not_calibrated");
    return;
  }

  adafruit_bno055_offsets_t offsets;
  if (!bno.getSensorOffsets(offsets)) {
    Serial.println("$BNOCAL,save_fail,read_error");
    return;
  }

  memcpy(g_bno_cal.data, &offsets, CAL_DATA_SIZE);
  g_bno_cal.valid = true;

  Serial.print("$BNOCAL,saved,sys=");
  Serial.print(sys);    Serial.print(",gyro=");
  Serial.print(gyro);   Serial.print(",accel=");
  Serial.print(accel);  Serial.print(",mag=");
  Serial.println(mag);
}

// ── BNO055 load calibration ──────────────────────────────────────────
void bnoLoadCal() {
  if (!g_bno_ok) {
    Serial.println("$BNOCAL,load_fail,no_bno055");
    return;
  }
  if (!g_bno_cal.valid) {
    Serial.println("$BNOCAL,load_fail,no_saved_data");
    return;
  }

  adafruit_bno055_offsets_t offsets;
  memcpy(&offsets, g_bno_cal.data, CAL_DATA_SIZE);

  bno.setMode(OPERATION_MODE_CONFIG);
  delay(25);
  bno.setSensorOffsets(offsets);
  bno.setMode(OPERATION_MODE_NDOF);
  delay(20);

  Serial.println("$BNOCAL,loaded");
}

// ── Send $INFO ──────────────────────────────────────────────────────
void sendInfo() {
  int hz = (int)(1000000UL / g_interval_us);
  Serial.print("$INFO,nano_rp2040,");
  Serial.print(hz);
  Serial.print(",");
  Serial.print(FIRMWARE_VERSION);
  Serial.print(",bno055=");
  Serial.println(g_bno_ok ? "yes" : "no");
}

// ── Setup ────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(SERIAL_BAUD);
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0 < 3000)) { delay(10); }

  // ---- On-board IMU (LSM6DSOX, uses internal Wire1) ----
  if (!IMU.begin()) {
    Serial.println("$ERR,imu_init_failed");
    while (1) { delay(1000); }
  }

  // ---- External BNO055 (uses Wire / I2C0 on A4/A5) ----
  Wire.begin();
  Wire.setClock(400000);
  Serial.println("$MSG,Probing BNO055 on Wire (A4/A5)...");
  if (initBNO055()) {
    g_bno_ok = true;
    Serial.println("$MSG,BNO055 detected and initialized (NDOF mode)");
  } else {
    Serial.println("$MSG,BNO055 not detected — running without it");
  }

  // ---- Microphone ----
  PDM.onReceive(onPDMdata);
  PDM.setBufferSize(MIC_BUF_SAMPLES * 2);
  if (!PDM.begin(MIC_CHANNELS, MIC_SAMPLE_RATE)) {
    Serial.println("$ERR,mic_init_failed");
  }

  // ---- RGB LED (via NINA module) ----
  WiFiDrv::pinMode(25, OUTPUT);
  WiFiDrv::pinMode(26, OUTPUT);
  WiFiDrv::pinMode(27, OUTPUT);
  setRGB(0, 0, 0);

  // ---- Initial gyro calibration (on-board LSM6DSOX) ----
  delay(500);
  calibrateGyro();

  sendInfo();
  g_last_send_us = micros();
  g_last_grv_us  = micros();
}

// ── Main loop ────────────────────────────────────────────────────────
void loop() {

  // ---- Handle host commands ----
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case '?': sendInfo();            break;
      case 'C': calibrateGyro();       break;
      case 'R': setRGB(255,   0,   0); break;
      case 'G': setRGB(  0, 255,   0); break;
      case 'B': setRGB(  0,   0, 255); break;
      case 'W': setRGB(255, 255, 255); break;
      case 'O': setRGB(  0,   0,   0); break;
      case '1': g_interval_us = 1000000UL /  50; break;
      case '2': g_interval_us = 1000000UL / 100; break;
      case '3': g_interval_us = 1000000UL / 200; break;
      case 'A': g_audio_stream = true;  g_aud_rd = g_aud_wr; break;
      case 'a': g_audio_stream = false; break;
      case 'S': bnoSaveCal();          break;
      case 'L': bnoLoadCal();          break;
      case 'N':
        if (g_bno_ok) {
          bno.setMode(OPERATION_MODE_NDOF);
          delay(20);
          Serial.println("$MODE,NDOF");
        }
        break;
      case 'I':
        if (g_bno_ok) {
          bno.setMode(OPERATION_MODE_IMUPLUS);
          delay(20);
          Serial.println("$MODE,IMU");
        }
        break;
      default: break;
    }
  }

  // ---- Send audio chunk if enough data accumulated ----
  if (g_audio_stream) {
    uint16_t wr = g_aud_wr;
    uint16_t avail = (wr >= g_aud_rd)
                     ? (wr - g_aud_rd)
                     : (AUD_RING_SIZE - g_aud_rd + wr);
    if (avail >= AUD_CHUNK_SIZE) {
      uint8_t chunk[AUD_CHUNK_SIZE];
      for (int i = 0; i < AUD_CHUNK_SIZE; i++) {
        chunk[i] = g_aud_ring[g_aud_rd];
        g_aud_rd = (g_aud_rd + 1) % AUD_RING_SIZE;
      }
      Serial.print("$AUD,");
      sendBase64(chunk, AUD_CHUNK_SIZE);
      Serial.println();
    }
  }

  unsigned long now_us = micros();

  // ---- BNO055: retry init if not detected at startup ----
  if (!g_bno_ok) {
    unsigned long now_ms_retry = millis();
    if (now_ms_retry - g_bno_retry_ms >= BNO_RETRY_INTERVAL_MS) {
      g_bno_retry_ms = now_ms_retry;
      if (initBNO055()) {
        g_bno_ok = true;
        Serial.println("$MSG,BNO055 detected (late init)");
        sendInfo();
      }
    }
  }

  // ---- BNO055: gravity vector at lower rate ----
  if (g_bno_ok && (now_us - g_last_grv_us) >= g_grv_interval) {
    g_last_grv_us = now_us;
    imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    Serial.print("$GRV,");
    Serial.print(gravity.x(), 3);  Serial.print(",");
    Serial.print(gravity.y(), 3);  Serial.print(",");
    Serial.println(gravity.z(), 3);
  }

  // ---- Rate limit ----
  if ((now_us - g_last_send_us) < g_interval_us) {
    return;
  }
  g_last_send_us = now_us;

  // ---- Read on-board IMU (LSM6DSOX) ----
  float ax = 0, ay = 0, az = 0;
  float gx = 0, gy = 0, gz = 0;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    ax -= g_accel_offset[0];
    ay -= g_accel_offset[1];
    az -= g_accel_offset[2];
  }
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
    gx -= g_gyro_offset[0];
    gy -= g_gyro_offset[1];
    gz -= g_gyro_offset[2];
  }

  // ---- Read temperature (throttled) ----
  unsigned long now_ms = millis();
  if (now_ms - g_last_temp_ms >= TEMP_INTERVAL_MS) {
    if (IMU.temperatureAvailable()) {
      int temp_raw;
      IMU.readTemperatureFloat(g_last_temp);
    }
    g_last_temp_ms = now_ms;
  }

  // ---- Build & send $IMU line (on-board sensor, always) ----
  Serial.print("$IMU,");
  Serial.print(ax, 4);  Serial.print(",");
  Serial.print(ay, 4);  Serial.print(",");
  Serial.print(az, 4);  Serial.print(",");
  Serial.print(gx, 4);  Serial.print(",");
  Serial.print(gy, 4);  Serial.print(",");
  Serial.print(gz, 4);  Serial.print(",");
  Serial.print(g_last_temp, 1);  Serial.print(",");
  Serial.print(g_mic_rms);       Serial.print(",");
  Serial.println(now_ms);

  // ---- Build & send $BNO line (external sensor, if available) ----
  if (g_bno_ok) {
    imu::Quaternion quat = bno.getQuat();
    imu::Vector<3> linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    int8_t bno_temp = bno.getTemp();

    uint8_t cal_sys, cal_gyro, cal_accel, cal_mag;
    bno.getCalibration(&cal_sys, &cal_gyro, &cal_accel, &cal_mag);

    // $BNO,qw,qx,qy,qz,ax,ay,az,gx,gy,gz,mx,my,mz,heading,roll,pitch,
    //      temp,cal_sys,cal_gyro,cal_accel,cal_mag,ms
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
    Serial.print(bno_temp);        Serial.print(",");
    Serial.print(cal_sys);         Serial.print(",");
    Serial.print(cal_gyro);        Serial.print(",");
    Serial.print(cal_accel);       Serial.print(",");
    Serial.print(cal_mag);         Serial.print(",");
    Serial.println(now_ms);
  }
}
