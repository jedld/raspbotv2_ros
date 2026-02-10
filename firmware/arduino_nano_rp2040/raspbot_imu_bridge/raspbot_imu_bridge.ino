/*
 * Raspbot — Arduino Nano RP2040 Connect sensor bridge
 *
 * Reads the on-board LSM6DSOX 6-axis IMU (accel + gyro + temperature) and
 * the MP34DT06JTR PDM microphone, then streams compact ASCII lines over
 * USB serial so a Raspberry Pi ROS 2 node can parse them.
 *
 * ── Wire protocol ──────────────────────────────────────────────────
 *
 *   Each line is '\n'-terminated.  Fields are comma-separated.
 *
 *   Sensor line (sent at IMU_HZ):
 *     $IMU,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<temp>,<mic>,<ms>\n
 *
 *       ax/ay/az : accelerometer  (g,   float, ±16 g)
 *       gx/gy/gz : gyroscope      (°/s, float, ±2000 °/s)
 *       temp     : die temperature (°C,  float)
 *       mic      : microphone RMS level (0-32767, int)
 *       ms       : millis() timestamp
 *
 *   On startup or when the host sends "?":
 *     $INFO,nano_rp2040,<IMU_HZ>,<version>\n
 *
 *   The host can send single-character commands:
 *     '?'  → re-send $INFO
 *     'R'  → set RGB LED to red
 *     'G'  → set RGB LED to green
 *     'B'  → set RGB LED to blue
 *     'W'  → set RGB LED to white
 *     'O'  → turn RGB LED off
 *     'C'  → calibrate gyro (hold still for 2 s)
 *     '1'  → set output rate to  50 Hz
 *     '2'  → set output rate to 100 Hz  (default)
 *     '3'  → set output rate to 200 Hz
 *
 * ── Required Arduino libraries ─────────────────────────────────────
 *
 *   • Arduino_LSM6DSOX   (Board Manager "Arduino Mbed OS Nano" installs it)
 *   • PDM                (built-in with the Mbed core)
 *   • WiFiNINA           (needed only to drive the on-board RGB LED)
 *
 * ── Board Manager ──────────────────────────────────────────────────
 *
 *   Install "Arduino Mbed OS Nano Boards" from Board Manager, then
 *   select "Arduino Nano RP2040 Connect".
 *
 * ── Flashing ───────────────────────────────────────────────────────
 *
 *   1. Connect the Nano RP2040 via USB to your computer
 *   2. In Arduino IDE: Tools → Board → Arduino Nano RP2040 Connect
 *   3. Tools → Port → (the CDC serial port that appeared)
 *   4. Upload
 *
 *   The Pi will see it as /dev/ttyACM0 (or /dev/ttyACM1).
 */

#include <Arduino_LSM6DSOX.h>
#include <PDM.h>
#include <WiFiNINA.h>          // for the on-board RGB LED only

// ── Tunables ─────────────────────────────────────────────────────────
#define FIRMWARE_VERSION   "1.0.0"
#define DEFAULT_HZ         100        // default output rate
#define SERIAL_BAUD        115200
#define MIC_SAMPLE_RATE    16000      // Hz
#define MIC_CHANNELS       1
#define MIC_BUF_SAMPLES    512

// ── RGB LED pins (accent LED accent accent accent accent accent)
// On the Nano RP2040 Connect the RGB LED is active-LOW and routed
// through the NINA module.  WiFiNINA exposes them as:
//   LEDR (25), LEDG (26), LEDB (27)  — active LOW
// ─────────────────────────────────────────────────────────────────────

// ── Globals ──────────────────────────────────────────────────────────
static volatile int     g_mic_rms      = 0;
static int16_t          g_mic_buf[MIC_BUF_SAMPLES];

static float  g_gyro_offset[3] = {0.0f, 0.0f, 0.0f};
static bool   g_calibrated     = false;

static unsigned long    g_interval_us  = 1000000UL / DEFAULT_HZ;
static unsigned long    g_last_send_us = 0;

static float g_last_temp      = 0.0f;
static unsigned long g_last_temp_ms = 0;
#define TEMP_INTERVAL_MS  500   // read temperature every 500 ms (slow sensor)

// ── Microphone callback (ISR context) ────────────────────────────────
void onPDMdata() {
  int bytesAvailable = PDM.available();
  int samplesToRead  = bytesAvailable / 2;  // 16-bit samples
  if (samplesToRead > MIC_BUF_SAMPLES) samplesToRead = MIC_BUF_SAMPLES;

  PDM.read(g_mic_buf, samplesToRead * 2);

  // Compute RMS
  long long sum = 0;
  for (int i = 0; i < samplesToRead; i++) {
    long s = (long)g_mic_buf[i];
    sum += s * s;
  }
  if (samplesToRead > 0) {
    g_mic_rms = (int)sqrt((double)sum / samplesToRead);
  }
}

// ── RGB LED helper ───────────────────────────────────────────────────
void setRGB(uint8_t r, uint8_t g, uint8_t b) {
  // Active LOW — 255 = off, 0 = full brightness
  WiFiDrv::analogWrite(25, 255 - r);   // Red
  WiFiDrv::analogWrite(26, 255 - g);   // Green
  WiFiDrv::analogWrite(27, 255 - b);   // Blue
}

// ── Gyro calibration ─────────────────────────────────────────────────
void calibrateGyro() {
  setRGB(255, 165, 0);  // orange while calibrating

  float sx = 0, sy = 0, sz = 0;
  int   n  = 0;
  unsigned long start = millis();

  Serial.println("$CAL,start");

  while (millis() - start < 2000) {
    float gx, gy, gz;
    if (IMU.gyroscopeAvailable() && IMU.readGyroscope(gx, gy, gz)) {
      sx += gx;
      sy += gy;
      sz += gz;
      n++;
    }
    delay(5);
  }

  if (n > 0) {
    g_gyro_offset[0] = sx / n;
    g_gyro_offset[1] = sy / n;
    g_gyro_offset[2] = sz / n;
    g_calibrated = true;
  }

  Serial.print("$CAL,done,");
  Serial.print(n);
  Serial.print(",");
  Serial.print(g_gyro_offset[0], 4);
  Serial.print(",");
  Serial.print(g_gyro_offset[1], 4);
  Serial.print(",");
  Serial.println(g_gyro_offset[2], 4);

  setRGB(0, 0, 0);  // off after calibration
}

// ── Send $INFO ──────────────────────────────────────────────────────
void sendInfo() {
  int hz = (int)(1000000UL / g_interval_us);
  Serial.print("$INFO,nano_rp2040,");
  Serial.print(hz);
  Serial.print(",");
  Serial.println(FIRMWARE_VERSION);
}

// ── Setup ────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(SERIAL_BAUD);
  // Wait up to 3 s for serial — if no host, continue anyway so the
  // board doesn't hang when powered from a dumb USB supply.
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0 < 3000)) { delay(10); }

  // ---- IMU ----
  if (!IMU.begin()) {
    Serial.println("$ERR,imu_init_failed");
    while (1) { delay(1000); }
  }

  // ---- Microphone ----
  PDM.onReceive(onPDMdata);
  PDM.setBufferSize(MIC_BUF_SAMPLES * 2);
  if (!PDM.begin(MIC_CHANNELS, MIC_SAMPLE_RATE)) {
    Serial.println("$ERR,mic_init_failed");
    // non-fatal — continue without mic
  }

  // ---- RGB LED (via NINA module) ----
  WiFiDrv::pinMode(25, OUTPUT);
  WiFiDrv::pinMode(26, OUTPUT);
  WiFiDrv::pinMode(27, OUTPUT);
  setRGB(0, 0, 0);  // off

  // ---- Initial calibration ----
  delay(500);        // let sensor settle
  calibrateGyro();

  sendInfo();
  g_last_send_us = micros();
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
      default: break;
    }
  }

  // ---- Rate limit ----
  unsigned long now_us = micros();
  if ((now_us - g_last_send_us) < g_interval_us) {
    return;
  }
  g_last_send_us = now_us;

  // ---- Read IMU ----
  float ax = 0, ay = 0, az = 0;
  float gx = 0, gy = 0, gz = 0;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
  }
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
    // Subtract calibration offsets
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

  // ---- Build & send line ----
  // $IMU,ax,ay,az,gx,gy,gz,temp,mic,ms
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
}
