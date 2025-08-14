#include "BMx280.h"
#include <SPI.h>

// ----------------------- Transport helpers -----------------------
void BMx280::write8(uint8_t reg, uint8_t val) const {
  if (!_useSPI) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write(val);
    _wire->endTransmission();
  } else {
    _spi->beginTransaction(SPISettings(_spiHz, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    // SPI write: MSB=0
    _spi->transfer(reg & 0x7F);
    _spi->transfer(val);
    digitalWrite(_cs, HIGH);
    _spi->endTransaction();
  }
}

uint8_t BMx280::read8(uint8_t reg) const {
  if (!_useSPI) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->endTransmission(false);
    _wire->requestFrom(_addr, (uint8_t)1);
    return _wire->read();
  } else {
    uint8_t v;
    _spi->beginTransaction(SPISettings(_spiHz, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    // SPI read: MSB=1
    _spi->transfer(reg | 0x80);
    v = _spi->transfer(0x00);
    digitalWrite(_cs, HIGH);
    _spi->endTransaction();
    return v;
  }
}

uint16_t BMx280::read16LE(uint8_t reg) const {
  if (!_useSPI) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->endTransmission(false);
    _wire->requestFrom(_addr, (uint8_t)2);
    uint8_t lo = _wire->read();
    uint8_t hi = _wire->read();
    return (uint16_t)hi << 8 | lo;
  } else {
    uint8_t b0, b1;
    _spi->beginTransaction(SPISettings(_spiHz, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    _spi->transfer(reg | 0x80);
    b0 = _spi->transfer(0x00);
    b1 = _spi->transfer(0x00);
    digitalWrite(_cs, HIGH);
    _spi->endTransaction();
    return (uint16_t)b1 << 8 | b0;
  }
}
int16_t BMx280::readS16LE(uint8_t reg) const { return (int16_t)read16LE(reg); }

void BMx280::readBurst(uint8_t reg, uint8_t *buf, uint8_t len) const {
  if (!_useSPI) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->endTransmission(false);
    _wire->requestFrom(_addr, len);
    for (uint8_t i = 0; i < len && _wire->available(); i++) buf[i] = _wire->read();
  } else {
    _spi->beginTransaction(SPISettings(_spiHz, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    _spi->transfer(reg | 0x80); // read
    for (uint8_t i = 0; i < len; i++) buf[i] = _spi->transfer(0x00);
    digitalWrite(_cs, HIGH);
    _spi->endTransaction();
  }
}

// ----------------------- Calibration -----------------------
void BMx280::readCalibration() {
  // Temp & Pressure (0x88..0xA1)
  dig_T1 = read16LE(0x88);
  dig_T2 = readS16LE(0x8A);
  dig_T3 = readS16LE(0x8C);

  dig_P1 = read16LE(0x8E);
  dig_P2 = readS16LE(0x90);
  dig_P3 = readS16LE(0x92);
  dig_P4 = readS16LE(0x94);
  dig_P5 = readS16LE(0x96);
  dig_P6 = readS16LE(0x98);
  dig_P7 = readS16LE(0x9A);
  dig_P8 = readS16LE(0x9C);
  dig_P9 = readS16LE(0x9E);

  if (_hasHumidity) {
    dig_H1 = read8(0xA1);
    // Humidity (0xE1..0xE7) with packed 12-bit signed H4/H5
    dig_H2 = readS16LE(0xE1);
    dig_H3 = read8(0xE3);
    uint8_t e4 = read8(0xE4);
    uint8_t e5 = read8(0xE5);
    uint8_t e6 = read8(0xE6);

    dig_H4 = (int16_t)((((int16_t)e4) << 4) | (e5 & 0x0F));
    if (dig_H4 & 0x0800) dig_H4 -= 4096;  // sign-extend 12-bit

    dig_H5 = (int16_t)((((int16_t)e6) << 4) | (e5 >> 4));
    if (dig_H5 & 0x0800) dig_H5 -= 4096;  // sign-extend 12-bit

    dig_H6 = (int8_t)read8(0xE7);
  }
}

// ----------------------- Compensation -----------------------
float BMx280::compensate_T(int32_t adc_T) {
  int32_t var1  = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  int32_t var2  = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
                   ((int32_t)dig_T3)) >> 14;
  t_fine = var1 + var2;
  int32_t T_01C = (t_fine * 5 + 128) >> 8;
  return T_01C / 100.0f; // °C
}

float BMx280::compensate_P(int32_t adc_P) {
  int64_t var1 = ((int64_t)t_fine) - 128000;
  int64_t var2 = var1 * var1 * (int64_t)dig_P6;
  var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
  var2 = var2 + (((int64_t)dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1) * (int64_t)dig_P1) >> 33;
  if (var1 == 0) return 0.0f;
  int64_t p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
  return (float)p / 256.0f; // Pa
}

float BMx280::compensate_H(int32_t adc_H) {
  // Bosch float formula (returns %RH)
  float var_H = (float)t_fine - 76800.0f;
  var_H = ( (float)adc_H
            - ( (float)dig_H4 * 64.0f
                + ( (float)dig_H5 / 16384.0f ) * var_H ) )
          * ( ( (float)dig_H2 / 65536.0f )
              * ( 1.0f + ( (float)dig_H6 / 67108864.0f ) * var_H
                    * ( 1.0f + ( (float)dig_H3 / 67108864.0f ) * var_H ) ) );
  var_H = var_H * ( 1.0f - ( (float)dig_H1 * var_H / 524288.0f ) );
  if (var_H > 100.0f) var_H = 100.0f;
  if (var_H <   0.0f) var_H =   0.0f;
  return var_H; // %RH
}

// ----------------------- Core -----------------------
bool BMx280::burstReadRaw(int32_t &adc_P, int32_t &adc_T, int32_t &adc_H) const {
  if (_hasHumidity) {
    uint8_t buf[8];
    readBurst(REG_PRESS_MSB, buf, 8); // F7..FE
    adc_P = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | (buf[2] >> 4);
    adc_T = ((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | (buf[5] >> 4);
    adc_H = ((int32_t)buf[6] << 8)  | (int32_t)buf[7];
  } else {
    uint8_t buf[6];
    readBurst(REG_PRESS_MSB, buf, 6); // BMP: only P(3),T(3)
    adc_P = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | (buf[2] >> 4);
    adc_T = ((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | (buf[5] >> 4);
    adc_H = 0;
  }
  return true;
}

void BMx280::applyConfig() {
  if (_hasHumidity) {
    // Humidity oversampling must be written before ctrl_meas
    write8(REG_CTRL_HUM, _osrs_h & 0x07);
  }
  // CONFIG (t_sb[7:5], filter[4:2], spi3w_en[0]=0)
  uint8_t config = (uint8_t)((_standby & 0x07) << 5 | (_iir & 0x07) << 2);
  write8(REG_CONFIG, config);
  // CTRL_MEAS (osrs_t[7:5], osrs_p[4:2], mode[1:0])
  uint8_t ctrl_meas = (uint8_t)((_osrs_t & 0x07) << 5 | (_osrs_p & 0x07) << 2 | (_mode & 0x03));
  write8(REG_CTRL_MEAS, ctrl_meas);
}

// ----------------------- Public -----------------------
bool BMx280::beginI2C(uint8_t addr, TwoWire *theWire) {
  _useSPI = false;
  _addr = addr;
  _wire = theWire;
  _wire->begin();
  _chipID = read8(REG_ID);
  if (_chipID != 0x60 && _chipID != 0x58) return false;
  _hasHumidity = (_chipID == 0x60);
  write8(REG_RESET, 0xB6);
  delay(3);
  while (read8(REG_STATUS) & 0x01) { delay(1); } // im_update
  readCalibration();
  setSampling(1,1, _hasHumidity ? 1 : 0, 0,0, MODE_NORMAL);
  return true;
}

bool BMx280::beginSPI(uint8_t csPin, SPIClass* theSPI, uint32_t spiHz) {
  _useSPI = true;
  _spi = theSPI ? theSPI : &SPI;
  _cs = csPin;
  _spiHz = spiHz;
  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, HIGH);
  _spi->begin();
  _chipID = read8(REG_ID);
  if (_chipID != 0x60 && _chipID != 0x58) return false;
  _hasHumidity = (_chipID == 0x60);
  write8(REG_RESET, 0xB6);
  delay(3);
  while (read8(REG_STATUS) & 0x01) { delay(1); }
  readCalibration();
  setSampling(1,1, _hasHumidity ? 1 : 0, 0,0, MODE_NORMAL);
  return true;
}

void BMx280::setSampling(uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h,
                         uint8_t iir, uint8_t standby, uint8_t mode) {
  _osrs_t = osrs_t & 7;
  _osrs_p = osrs_p & 7;
  _osrs_h = _hasHumidity ? (osrs_h & 7) : 0; // ignore on BMP
  _iir    = iir & 7;
  _standby= standby & 7;
  _mode   = mode & 3;
  applyConfig();
}

bool BMx280::read280(float &T_c, float &P_hPa, float &H_rh) {
  int32_t ap, at, ah;
  if (!burstReadRaw(ap, at, ah)) return false;
  T_c  = compensate_T(at);
  P_hPa= compensate_P(ap) / 100.0f;
  H_rh = _hasHumidity ? compensate_H(ah) : NAN;
  return true;
}

bool BMx280::readForced280(float &T_c, float &P_hPa, float &H_rh) {
  // Switch to forced mode using current OS/IIR settings
  uint8_t ctrl_meas = (uint8_t)((_osrs_t & 7) << 5 | (_osrs_p & 7) << 2 | MODE_FORCED);
  write8(REG_CTRL_MEAS, ctrl_meas);
  // Wait for measurement to complete (measuring bit clears)
  delay(1);
  while (isMeasuring()) { /* wait */ }
  return read280(T_c, P_hPa, H_rh);
}

float BMx280::readAltitude(float seaLevel_hPa) {
  float T, P_hPa, H;
  if (!read280(T, P_hPa, H)) return NAN;
  return 44330.0f * (1.0f - powf(P_hPa / seaLevel_hPa, 0.1903f));
}

uint8_t BMx280::status() const { return read8(REG_STATUS); }
bool BMx280::isMeasuring() const { return (read8(REG_STATUS) & 0x08) != 0; }
bool BMx280::isUpdatingNVM() const { return (read8(REG_STATUS) & 0x01) != 0; }
