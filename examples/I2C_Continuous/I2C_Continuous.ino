// Continuous reading of BMx280 sensor module via I2C
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
// I2C_Continuous – Demonstrates continuous-mode reading of a BMP280 or BME280 over I²C. 
// The sensor runs in normal mode, automatically performing measurements at a fixed standby interval. 
// Oversampling, filtering, and standby time are configured using setSampling() to improve accuracy and stability. 
// The example prints temperature, pressure, and (if available) humidity every update cycle without waking the sensor manually.
// 
// WIRING (I2C) - Use logic level shifter (LLS) if using a 5V microcontroller!
// BMx280 -> microcontroller
// VCC -> 3.3V
// GND -> GND
// SDA -> SDA (A4 on classic Uno, etc)
// SCL -> SCL (A5 on classic Uno, etc)
// Address 0x76 (default), or 0x77 if SD0 is pulled high

#include <Wire.h>
#include <BMx280.h>

const uint8_t  I2C_ADDR      = 0x76;     // Default address
const unsigned POLL_MS       = 2000;     // [mS] How often to read sensor
const float    SEA_LEVEL_HPA = 1013.25f;

BMx280 bmx;

void setup() {
  Serial.begin(115200);

  while (!Serial) {}
  // Wire.setClock(100000); // for 8MHz AVRs, or long wires
  if (!bmx.beginI2C(I2C_ADDR)) { 
    Serial.println("BMx280 not found"); 
    while(1){} 
  }

  bmx.setSampling(1,1,1, 0,0, BMx280::MODE_NORMAL); 

  Serial.print("chipID=0x"); Serial.println(bmx.chipID(), HEX);
  Serial.print("hasHumidity="); Serial.println(bmx.hasHumidity());
  Serial.println("I2C Continuous");
}

void loop() {
  float T, P_hPa, H;
  
  if (bmx.read280(T, P_hPa, H)) {
    Serial.print("T="); 
    Serial.print(T,2); 
    Serial.print(" °C  ");
    if (bmx.hasHumidity()) { 
      Serial.print("H="); 
      Serial.print(H,1); 
      Serial.print(" %  "); 
    }
    Serial.print("P="); 
    Serial.print(P_hPa,2); 
    Serial.print(" hPa  ");
    Serial.print("Alt="); 
    Serial.print(bmx.readAltitude(SEA_LEVEL_HPA),1); 
    Serial.println(" m");
  }
  delay(POLL_MS);
}
