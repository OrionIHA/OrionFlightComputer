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

#include "lsm303agr.hpp"

LSM303AGR::LSM303AGR() {
  // Constructor
  m_data = { 0 };
}

bool LSM303AGR::begin() {
  Wire.begin(); 
  
  if (!checkConnection()) {
    return false;
  }

  // --- İVMEÖLÇER (ACCEL) AYARI: 1.344 kHz ---
  // CTRL_REG1_A (0x20): ODR=1.344kHz (1001), Normal Mode (0), XYZ Enabled (111) -> 0x97
  writeRegister(ACCEL_ADDRESS, REG_ACC_CTRL1, 0x97);
  
  // CTRL_REG4_A (0x23): BDU enabled, FS = +/-4g, HR enabled -> 0x98 (Eski ayarımız kalıyor)
  writeRegister(ACCEL_ADDRESS, REG_ACC_CTRL4, 0x98);

  // --- MANYETOMETRE (MAG) AYARI: 100 Hz ---
  writeRegister(MAG_ADDRESS, REG_ACC_CTRL1 + 0x40, 0x00); 
  
  // CFG_REG_A_M (0x60): ODR=100Hz (11), Mode=Continuous (00) -> 0x0C
  writeRegister(MAG_ADDRESS, REG_MAG_CFG_A, 0x0C);

  // CFG_REG_C_M (0x62): BDU Enabled -> 0x10
  writeRegister(MAG_ADDRESS, REG_MAG_CFG_C, 0x10);

  m_initialized = true;
  return true;
}

bool LSM303AGR::checkConnection() {
  // Accel WhoAmI: 0x33
  uint8_t wai_a = readRegister(ACCEL_ADDRESS, REG_ACC_WHO_AM_I);
  // Mag WhoAmI: 0x40
  uint8_t wai_m = readRegister(MAG_ADDRESS, REG_MAG_WHO_AM_I);

  return (wai_a == 0x33) && (wai_m == 0x40);
}

void LSM303AGR::readAccelerometer() {
  uint8_t rawData[6];
  // 0x28 adresinden başlayarak 6 byte oku (X_L, X_H, Y_L, Y_H, Z_L, Z_H)
  readBytes(ACCEL_ADDRESS, REG_ACC_OUT_X_L | 0x80, 6, rawData);

  // 12-bit Left Justified veriyi sağa kaydır
  int16_t rawX = (int16_t)((rawData[1] << 8) | rawData[0]) >> 4;
  int16_t rawY = (int16_t)((rawData[3] << 8) | rawData[2]) >> 4;
  int16_t rawZ = (int16_t)((rawData[5] << 8) | rawData[4]) >> 4;

  // Sensitivity: HR Mode +/- 4g için 2 mg/digit
  // 2 mg = 0.002 g
  float sensitivity = 0.002f;

  m_data.accelX = (float)rawX * sensitivity;
  m_data.accelY = (float)rawY * sensitivity;
  m_data.accelZ = (float)rawZ * sensitivity;

  m_data.accelReady = true;
}

void LSM303AGR::readMagnetometer() {
  uint8_t rawData[6];
  readBytes(MAG_ADDRESS, REG_MAG_OUT_X_L | 0x80, 6, rawData);

  int16_t rawX = (int16_t)((rawData[1] << 8) | rawData[0]);
  int16_t rawY = (int16_t)((rawData[3] << 8) | rawData[2]);
  int16_t rawZ = (int16_t)((rawData[5] << 8) | rawData[4]);

  // Sensitivity: 1.5 mGauss/LSB = 0.0015 Gauss/LSB
  float sensitivity = 0.0015f;

  m_data.magX = (float)rawX * sensitivity;
  m_data.magY = (float)rawY * sensitivity;
  m_data.magZ = (float)rawZ * sensitivity;

  m_data.magReady = true;
}

void LSM303AGR::readAllDatas() {
  static uint32_t lastReadTime = 0;
  LSM303AGR::readAccelerometer();
  LSM303AGR::readMagnetometer();

  if constexpr (Config::DEBUG_LSM303AGR) {
    if (millis() - lastReadTime > 100) {
      lastReadTime = millis();
      Serial.print("\taccel_X:\t"); Serial.print(m_data.accelX);
      Serial.print("\taccel_Y:\t"); Serial.print(m_data.accelY);
      Serial.print("\taccel_Z:\t"); Serial.println(m_data.accelZ);

      Serial.print("\tmag_X:\t");   Serial.print(m_data.magX);
      Serial.print("\tmag_Y:\t");   Serial.print(m_data.magY);
      Serial.print("\tmag_Z:\t");   Serial.println(m_data.magZ);
    }
  }
}

// --- I2C Helper Functions ---

void LSM303AGR::writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t LSM303AGR::readRegister(uint8_t address, uint8_t reg) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(address, (uint8_t)1);
  return (Wire.available()) ? Wire.read() : 0;
}

void LSM303AGR::readBytes(uint8_t address, uint8_t reg, uint8_t count, uint8_t* dest) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(false);

  uint8_t i = 0;
  Wire.requestFrom(address, count);
  while (Wire.available() && i < count) {
    dest[i++] = Wire.read();
  }
}