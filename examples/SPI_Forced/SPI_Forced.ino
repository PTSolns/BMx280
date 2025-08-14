// Forced reading of BMx280 sensor module via SPI
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
// This example shows how to use the BMx280 sensor (BME280/BMP280) in forced measurement mode 
// over an SPI connection. The microcontroller communicates with the sensor via the hardware SPI pins 
// (SCK, MISO, MOSI) and a dedicated chip select (CS) pin. In forced mode, the sensor stays in sleep 
// between measurements, reducing power consumption to the sub-microamp range. When a reading is needed, 
// readForced280() wakes the sensor, takes a single burst measurement of temperature, pressure, 
// and humidity (BME280 only), and then automatically returns it to sleep. This mode is ideal for 
// battery-powered applications where measurements are taken at set intervals.
//
// WIRING (SPI) - Use logic level shifter (LLS) if using a 5V microcontroller!
// BMx280 -> microcontroller
// VCC -> 3.3V
// GND -> GND
// SDA -> COPI (D11 on Uno etc)
// SCL -> SCK (D13 on Uno etc)
// SD0 -> CIPO (D12 on Uno etc)
// CSK -> D10 (configurable below)

#include <SPI.h>
#include <BMx280.h>

const uint8_t  CS_PIN      = 10;
const uint32_t SPI_HZ      = 8000000UL;
const unsigned INTERVAL_MS = 2000;

BMx280 bmx;

void setup() {
  Serial.begin(115200);

  while (!Serial) {}
  if (!bmx.beginSPI(CS_PIN, &SPI, SPI_HZ)) { 
    Serial.println("BMx280 (SPI) not found"); 
    while(1){} 
  }

  bmx.setSampling(1,1,1, 0,0, BMx280::MODE_SLEEP);

  Serial.print("chipID=0x"); Serial.println(bmx.chipID(), HEX);
  Serial.print("hasHumidity="); Serial.println(bmx.hasHumidity());
  Serial.println("SPI Forced");
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
      Serial.print(" %  "); 
    }
    Serial.print("P="); 
    Serial.print(P_hPa,2); 
    Serial.println(" hPa");
  }
  delay(INTERVAL_MS);
}
