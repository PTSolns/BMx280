// Forced reading of BMx280 sensor module (wake from sleep) via I2C
//
// Last update: Aug 14, 2025
//
// Contact: https://ptsolns.com/contact-us
// Website: https://ptsolns.com
// Documentation Repsoitory: https://docs.ptsolns.com/
// See Tinker Thoughts Blog (TTB) discussing current draw of Normal Mode vs. Sleep Mode: 
//   https://ptsolns.com/blogs/tinker-thoughts/ttb16-measuring-the-current-draw-on-bmp280-sensor-module
//
// DESCRIPTION
// I2C_Forced – Demonstrates low-power operation of a BMP280 or BME280 in forced mode over I²C. 
// The sensor spends most of the time in sleep, taking a single “burst” measurement only when 
// readForced280() is called. This minimizes I²C bus usage and reduces power consumption, making 
// it ideal for battery-powered applications. The example configures oversampling, filtering, 
// and standby settings via setSampling() before triggering each measurement.
//
// WIRING (I2C) - Use logic level shifter (LLS) if using a 5V microcontroller!
// BMx280 -> microcontroller
// VCC -> 3.3V
// GND -> GND
// SDA -> SDA (A4 on classic Uno, etc)
// SCL -> SCL (A5 on classic Uno, etc)
// Address 0x76 (default), or 0x77 if SD0 is pulled high
//
// SETSAMPLING NOTE
//  setSampling(tempOS, pressOS, humOS, iirFilter, standbyTime, mode)
//    tempOS / pressOS / humOS (oversampling):
//      0=skip, 1=x1, 2=x2, 4=x4, 8=x8, 16=x16 (Bosch datasheet values)
//      Higher = smoother & more precise, but slower & more power.
//    iirFilter (IIR low-pass coefficient):
//      0=off, 1=2, 2=4, 3=8, 4=16. Helps smooth pressure spikes; increases response time.
//    standbyTime (NORMAL mode only; ignored in SLEEP/FORCED):
//      0=0.5ms, 1=62.5ms, 2=125ms, 3=250ms, 4=500ms, 5=1000ms, 6=10ms, 7=20ms
//    mode:
//      BMx280::MODE_SLEEP  - idle (use this for forced on-demand reads)
//      BMx280::MODE_FORCED - (used internally by readForced280)
//      BMx280::MODE_NORMAL - continuous conversions (for Continuous example)

#include <Wire.h>
#include <BMx280.h>

const uint8_t  I2C_ADDR    = 0x76; // Default address
const unsigned INTERVAL_MS = 2000; // [mS] How often to read sensor

BMx280 bmx;

void setup() {
  Serial.begin(115200);

  while (!Serial) {}
  // Wire.setClock(100000); // for 8MHz AVRs, or long wires
  if (!bmx.beginI2C(I2C_ADDR)) { 
    Serial.println("BMx280 not found"); 
    while(1){} 
  }

  bmx.setSampling(
    1, // temp oversampling x1
    1, // pressure oversampling x1
    1, // humidity oversampling x1 (ignored on BMP280)
    0, // IIR filter off
    0, // standby (ignored in sleep/forced)
    BMx280::MODE_SLEEP // start in sleep; each cycle uses readForced280()
  );

  Serial.print("chipID=0x"); Serial.println(bmx.chipID(), HEX);
  Serial.print("hasHumidity="); Serial.println(bmx.hasHumidity());
  Serial.println("I2C Forced (sleep/wake single-burst)");
}

void loop() {
  float T, P_hPa, H;
  if (bmx.readForced280(T, P_hPa, H)) {
    Serial.print("T="); 
    Serial.print(T,2); 
    Serial.print(" °C  ");
    if (bmx.hasHumidity()) { 
      Serial.print("H="); 
      Serial.print(H,1); 
      Serial.print(" %  "); }
    Serial.print("P="); 
    Serial.print(P_hPa,2); 
    Serial.println(" hPa");
  }
  delay(INTERVAL_MS);
}