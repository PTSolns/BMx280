#pragma once
#include <Arduino.h>
#include <Wire.h>
class SPIClass; // forward decl

class BMx280 {
public:
  // Registers
  static constexpr uint8_t REG_ID        = 0xD0;
  static constexpr uint8_t REG_RESET     = 0xE0;
  static constexpr uint8_t REG_CTRL_HUM  = 0xF2;
  static constexpr uint8_t REG_STATUS    = 0xF3;
  static constexpr uint8_t REG_CTRL_MEAS = 0xF4;
  static constexpr uint8_t REG_CONFIG    = 0xF5;
  static constexpr uint8_t REG_PRESS_MSB = 0xF7; // P(3), T(3), H(2 on BME)

  enum : uint8_t { MODE_SLEEP=0, MODE_FORCED=1, MODE_NORMAL=3 };

  BMx280() : _wire(&Wire) {}

  // Choose a transport
  bool beginI2C(uint8_t addr=0x76, TwoWire *theWire=&Wire);
  bool beginSPI(uint8_t csPin, SPIClass* theSPI=nullptr, uint32_t spiHz=8000000UL); // 4‑wire SPI

  // Configuration
  void setSampling(uint8_t osrs_t=1, uint8_t osrs_p=1, uint8_t osrs_h=1,
                   uint8_t iir=0, uint8_t standby=0, uint8_t mode=MODE_NORMAL);

  // Burst-only API (auto BME/BMP)
  bool read280(float &T_c, float &P_hPa, float &H_rh);
  bool readForced280(float &T_c, float &P_hPa, float &H_rh); // trigger one conversion then burst-read

  // Helpers
  float readAltitude(float seaLevel_hPa = 1013.25f);
  uint8_t chipID() const { return _chipID; }
  bool hasHumidity() const { return _hasHumidity; }
  uint8_t status() const;        // content of REG_STATUS
  bool isMeasuring() const;      // STATUS[3]
  bool isUpdatingNVM() const;    // STATUS[0]

private:
  // Transport-agnostic register I/O
  void     write8(uint8_t reg, uint8_t val) const;
  uint8_t  read8(uint8_t reg) const;
  uint16_t read16LE(uint8_t reg) const;
  int16_t  readS16LE(uint8_t reg) const;
  void     readBurst(uint8_t reg, uint8_t *buf, uint8_t len) const;

  // Calibration & compensation
  void   readCalibration();
  float  compensate_T(int32_t adc_T);
  float  compensate_P(int32_t adc_P);
  float  compensate_H(int32_t adc_H);

  bool   burstReadRaw(int32_t &adc_P, int32_t &adc_T, int32_t &adc_H) const;
  void   applyConfig(); // write registers from shadow

  // Bus & device
  bool     _useSPI = false;
  TwoWire* _wire;
  uint8_t  _addr = 0x76;

  // SPI
  SPIClass* _spi = nullptr;
  uint8_t   _cs  = 255;
  uint32_t  _spiHz = 8000000UL;

  // Identity
  uint8_t   _chipID = 0x00;
  bool      _hasHumidity = false; // true for BME280, false for BMP280

  // Config shadow
  uint8_t   _osrs_t = 1, _osrs_p = 1, _osrs_h = 1;
  uint8_t   _iir = 0, _standby = 0, _mode = MODE_NORMAL;

  // Calibration
  uint16_t dig_T1;  int16_t dig_T2, dig_T3;
  uint16_t dig_P1;  int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
  uint8_t  dig_H1;  int16_t dig_H2;  uint8_t dig_H3;  int16_t dig_H4; int16_t dig_H5; int8_t dig_H6;

  // Shared state
  int32_t  t_fine = 0;
};
