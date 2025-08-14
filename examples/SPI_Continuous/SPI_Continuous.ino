// Continuous reading of BMx280 sensor module via SPI
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
// This example demonstrates how to use the BMx280 sensor (BME280/BMP280) in continuous measurement mode 
// over an SPI connection. The microcontroller communicates with the sensor using the hardware SPI pins 
// (SCK, MISO, MOSI) plus a dedicated chip select (CS) pin. The setSampling() function configures 
// measurement parameters, and the sensor continuously updates temperature, pressure, and humidity 
// (BME280 only) in the background. The read280() function retrieves all available measurements in 
// a single SPI burst, minimizing communication overhead. Continuous mode keeps the sensor active at all times, 
// providing regular real-time updates without needing manual triggering.
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

const uint8_t  CS_PIN  = 10;        // can be changed
const uint32_t SPI_HZ  = 8000000UL; // reduce if wiring is long/noisy
const unsigned POLL_MS = 2000;

BMx280 bmx;

void setup() {
  Serial.begin(115200);

  while (!Serial) {}
  if (!bmx.beginSPI(CS_PIN, &SPI, SPI_HZ)) { 
    Serial.println("BMx280 (SPI) not found"); 
    while(1){} 
    }

  bmx.setSampling(1,1,1, 0,0, BMx280::MODE_NORMAL);

  Serial.print("chipID=0x"); Serial.println(bmx.chipID(), HEX);
  Serial.print("hasHumidity="); Serial.println(bmx.hasHumidity());
  Serial.println("SPI Continuous");
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
    Serial.println(" hPa");
  }
  delay(POLL_MS);
}