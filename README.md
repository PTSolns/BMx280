# BMx280 (Auto‑detect BME280/BMP280, I²C & SPI, Single‑Burst)

**Library for BMP280 and BME280 sensor modules with emphasis on efficient single‑burst reading to minimize I²C/SPI bus usage, while maintaining full compatibility with both continuous and forced‑mode measurements.**

The driver **auto‑detects** whether your sensor is **BME280** (Temperature + Pressure + Humidity) or **BMP280** (Temperature + Pressure only), then performs **single‑burst reads** starting at `0xF7` for minimal bus time. Works over **I²C** and **4‑wire SPI**.

- **BME280 detected (`chipID` = 0x60):** returns **T, P, H**
- **BMP280 detected (`chipID` = 0x58):** returns **T, P**; humidity is not present

## Public API (burst‑only)
```cpp
bool beginI2C(uint8_t addr=0x76, TwoWire *wire=&Wire);
bool beginSPI(uint8_t csPin, SPIClass* spi=&SPI, uint32_t spiHz=8000000UL);

void setSampling(uint8_t osrs_t=1, uint8_t osrs_p=1, uint8_t osrs_h=1,
                 uint8_t iir=0, uint8_t standby=0, uint8_t mode=MODE_NORMAL);

bool read280(float &T_c, float &P_hPa, float &H_rh);        // returns T,P,(H if present)
bool readForced280(float &T_c, float &P_hPa, float &H_rh);  // triggers one conversion then reads burst

float readAltitude(float seaLevel_hPa = 1013.25f);
uint8_t chipID() const;
bool hasHumidity() const;        // true on BME280, false on BMP280
uint8_t status() const; bool isMeasuring() const; bool isUpdatingNVM() const;
```

**Note:** On BMP280 (`hasHumidity()==false`), `H_rh` is set to `NAN` by `read280()` / `readForced280()`.

## Settings (Bosch‑style)
- **Mode:** `MODE_SLEEP` (idle), `MODE_FORCED` (one conversion), `MODE_NORMAL` (continuous)
- **Oversampling:** 0=skip, 1=x1, 2=x2, 4=x4, 8=x8, 16=x16  (all three on BME; BMP ignores `osrs_h`)
- **IIR filter:** 0=off, 1=2, 2=4, 3=8, 4=16
- **Standby (NORMAL mode only):** 0=0.5ms, 1=62.5ms, 2=125ms, 3=250ms, 4=500ms, 5=1000ms, 6=10ms, 7=20ms

## Wiring

### I²C
- VCC→3.3V, GND→GND, SDA→SDA, SCL→SCL
- Address: 0x76 (default) or 0x77 (SDO high on many boards)

### SPI (4‑wire)
- VCC→3.3V, GND→GND
- CS → your chosen digital pin (e.g., D10 on AVR)
- SCK→SCK, MOSI→MOSI (SDI), MISO→MISO (SDO)
- **3.3V logic only** — level‑shift if your MCU is 5 V

## Board Compatibility
Works on any Arduino‑compatible core with standard **Wire**/**SPI** and **C++11** support.

**Known‑good**: AVR (Uno/Nano/Pro Mini/Mega/Leonardo), SAMD (Zero/MKR), RP2040 (Pico), ESP8266/ESP32, Teensy 3/4, STM32 (Arduino core), nRF52, etc.  
**Edge cases**: very old cores lacking repeated‑start or SPI transactions; 3‑wire SPI only; 5 V logic without level shifting.

## Examples
- `I2C_Continuous` — NORMAL mode, periodic `read280()`
- `I2C_Forced` — SLEEP + `readForced280()` (detailed `setSampling` comments)
- `SPI_Continuous` — NORMAL mode over SPI
- `SPI_Forced` — forced mode over SPI

## License
MIT
