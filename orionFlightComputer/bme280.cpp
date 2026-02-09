/*
* Copyright (c) 2026 Orion UAV Team
*
* This file is part of the OrionFlightComputer distribution (https://github.com/OrionIHA/OrionFlightComputer).
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, version 3.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
* General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include "bme280.hpp"

// Register Adresleri
#define BME280_REGISTER_DIG_T1 0x88
#define BME280_REGISTER_CHIPID 0xD0
#define BME280_REGISTER_CONTROL 0xF4
#define BME280_REGISTER_CONFIG 0xF5
#define BME280_REGISTER_PRESSUREDATA 0xF7

BME280::BME280() {
  m_tempOffset = 0.0f;
  m_pressOffset = 0.0f;
}

void BME280::begin() {
  Wire.setPins(Config::I2C_SDA, Config::I2C_SCL);
  Wire.begin();
  Wire.setClock(1000000);  // 1MHz Fast I2C

  writeRegister(0xE0, 0xB6);  // Reset
  delay(100);

  readCalibrationData();

  writeRegister(BME280_REGISTER_CONFIG, 0x00);   // Standby 0.5ms, Filter OFF
  writeRegister(BME280_REGISTER_CONTROL, 0x27);  // Mode Normal, Oversampling x1

  delay(100);
}

void BME280::setCalibrationOffsets(float tempOffsetDegC, float pressOffsetMbar) {
  m_tempOffset = tempOffsetDegC;
  m_pressOffset = pressOffsetMbar;
}

void BME280::readBurstData() {
  // Burst Read İşlemi (Aynen korundu)
  Wire.beginTransmission(BME280_ADD);
  Wire.write(BME280_REGISTER_PRESSUREDATA);
  Wire.endTransmission();

  Wire.requestFrom(BME280_ADD, (uint8_t)6);

  if (Wire.available() < 6) return;

  uint32_t p_msb = Wire.read();
  uint32_t p_lsb = Wire.read();
  uint32_t p_xlsb = Wire.read();
  uint32_t t_msb = Wire.read();
  uint32_t t_lsb = Wire.read();
  uint32_t t_xlsb = Wire.read();

  int32_t adc_P = (p_msb << 12) | (p_lsb << 4) | (p_xlsb >> 4);
  int32_t adc_T = (t_msb << 12) | (t_lsb << 4) | (t_xlsb >> 4);

  // Sıcaklık Hesabı
  float var1, var2, t_fine;
  var1 = (((float)adc_T) / 16384.0f - ((float)dig_T1) / 1024.0f) * ((float)dig_T2);
  var2 = ((((float)adc_T) / 131072.0f - ((float)dig_T1) / 8192.0f) * (((float)adc_T) / 131072.0f - ((float)dig_T1) / 8192.0f)) * ((float)dig_T3);
  t_fine = var1 + var2;
  float T = (var1 + var2) / 5120.0f;
  m_lastTemp = T + m_tempOffset;

  // Basınç Hesabı (Pa)
  float p;
  var1 = (t_fine / 2.0f) - 64000.0f;
  var2 = var1 * var1 * ((float)dig_P6) / 32768.0f;
  var2 = var2 + var1 * ((float)dig_P5) * 2.0f;
  var2 = (var2 / 4.0f) + (((float)dig_P4) * 65536.0f);
  var1 = (((float)dig_P3) * var1 * var1 / 524288.0f + ((float)dig_P2) * var1) / 524288.0f;
  var1 = (1.0f + var1 / 32768.0f) * ((float)dig_P1);

  if (var1 == 0.0f) {
    m_lastPress = 0.0f;
  } else {
    p = 1048576.0f - (float)adc_P;
    p = (p - (var2 / 4096.0f)) * 6250.0f / var1;
    var1 = ((float)dig_P9) * p * p / 2147483648.0f;
    var2 = p * ((float)dig_P8) / 32768.0f;
    p = p + (var1 + var2 + ((float)dig_P7)) / 16.0f;

    // Pa -> mbar Dönüşümü
    p = p / 100.0f;
    m_lastPress = p + m_pressOffset;
  }
}

float BME280::getPressure() {
  return m_lastPress;
}
float BME280::getTemperature() {
  return m_lastTemp;
}

/*!
 *   Calculates the altitude (in meters) from the specified atmospheric
 *   pressure (in hPa), and sea-level pressure (in hPa).
 *   @param  seaLevel      Sea-level pressure in hPa
 *   @returns the altitude value read from the device
 */
float BME280::readAltitude(float seaLevel) {
  return 44330.0 * (1.0 - pow(getPressure() / seaLevel, 0.1903));
}

/*!
 *   Calculates the pressure at sea level (in hPa) from the specified
 * altitude (in meters), and atmospheric pressure (in hPa).
 *   @param  altitude      Altitude in meters
 *   @param  atmospheric   Atmospheric pressure in hPa
 *   @returns the pressure at sea level (in hPa) from the specified altitude
 */
float BME280::calculateSeaLevelPressure(float altitude, float atmospheric) {
  return atmospheric / pow(1.0 - (altitude / 44330.0), 5.255);
}

// ----------------------------

BME280::BaroData BME280::getAllData() {
  readBurstData();

  BaroData data;
  data.pres = getPressure();
  data.temp = getTemperature();
  // Varsayılan olarak deniz seviyesi yüksekliği döner
  data.alti = readAltitude(SEALEVELPRESSURE_MBAR);
  return data;
}

float BME280::getSeaLevelPressure() {
  return SEALEVELPRESSURE_MBAR;
}

/*!
 * Set new sea level open air pressure. (Default 1013.25mbar)
 * @param seaLevelPressure custom Sea level pressure value
*/
void BME280::setSeaLevelPressure(float seaLevelPressure) {
  SEALEVELPRESSURE_MBAR = seaLevelPressure;
}

uint8_t BME280::whoAmI() {
  Wire.beginTransmission(BME280_ADD);
  Wire.write(BME280_REGISTER_CHIPID);
  Wire.endTransmission();
  Wire.requestFrom(BME280_ADD, (uint8_t)1);
  return Wire.read();
}

void BME280::readCalibrationData() {
  // Kalibrasyon verisi okuma (Aynen korundu)
  uint8_t buffer[26];
  Wire.beginTransmission(BME280_ADD);
  Wire.write(BME280_REGISTER_DIG_T1);
  Wire.endTransmission();
  Wire.requestFrom(BME280_ADD, (uint8_t)24);
  for (int i = 0; i < 24; i++) {
    if (Wire.available()) buffer[i] = Wire.read();
  }
  dig_T1 = (buffer[1] << 8) | buffer[0];
  dig_T2 = (int16_t)((buffer[3] << 8) | buffer[2]);
  dig_T3 = (int16_t)((buffer[5] << 8) | buffer[4]);
  dig_P1 = (buffer[7] << 8) | buffer[6];
  dig_P2 = (int16_t)((buffer[9] << 8) | buffer[8]);
  dig_P3 = (int16_t)((buffer[11] << 8) | buffer[10]);
  dig_P4 = (int16_t)((buffer[13] << 8) | buffer[12]);
  dig_P5 = (int16_t)((buffer[15] << 8) | buffer[14]);
  dig_P6 = (int16_t)((buffer[17] << 8) | buffer[16]);
  dig_P7 = (int16_t)((buffer[19] << 8) | buffer[18]);
  dig_P8 = (int16_t)((buffer[21] << 8) | buffer[20]);
  dig_P9 = (int16_t)((buffer[23] << 8) | buffer[22]);
}

void BME280::writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(BME280_ADD);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}