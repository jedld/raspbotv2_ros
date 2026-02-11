/*
 * I2C Scanner for Arduino Nano RP2040 Connect
 * Scans the external Wire (I2C0) bus on pins A4 (SDA) / A5 (SCL)
 * and reports all detected addresses over USB serial.
 */
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0 < 3000)) { delay(10); }

  Wire.begin();
  Wire.setClock(100000);  // start slow for reliability

  Serial.println("I2C Scanner — scanning Wire (A4/A5)...");
  Serial.println("=========================================");
}

void loop() {
  int found = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();

    if (err == 0) {
      Serial.print("  Found device at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      found++;
    } else if (err == 4) {
      Serial.print("  Error at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
    }
  }

  if (found == 0) {
    Serial.println("  No I2C devices found!");
    Serial.println("  Check wiring: BNO055 SDA→A4, SCL→A5, VIN→3.3V, GND→GND");
  } else {
    Serial.print("  Total: ");
    Serial.print(found);
    Serial.println(" device(s)");
  }

  Serial.println("-----------------------------------------");
  delay(3000);
}
