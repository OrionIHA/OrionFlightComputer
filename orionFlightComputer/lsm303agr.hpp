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

#pragma once

#include <Arduino.h>
#include "Config.hpp"
#include <Wire.h>

class LSM303AGR {
public:
  // Veri yapısı: Hem ham (int16) hem de işlenmiş (float) verileri tutar
  struct SensorData {
    // Accelerometer Data (g)
    float accelX;
    float accelY;
    float accelZ;

    // Magnetometer Data (Gauss)
    float magX;
    float magY;
    float magZ;

    // Status
    bool accelReady;
    bool magReady;
  };

  LSM303AGR();

  // Sensörü başlatır ve konfigüre eder
  bool begin();

  // Verileri okur
  void readAccelerometer();
  void readMagnetometer();
  void readAllDatas();

  // Veriye erişim
  SensorData* getData() {
    return &m_data;
  }

  // Bağlantı kontrolü (Who Am I)
  bool checkConnection();

private:
  // I2C Adresleri [Datasheet Table 24, 25]
  static constexpr uint8_t ACCEL_ADDRESS = 0x19;  // 0011001b
  static constexpr uint8_t MAG_ADDRESS = 0x1E;    // 0011110b

  // Register Adresleri - Accelerometer [Datasheet Table 26]
  static constexpr uint8_t REG_ACC_CTRL1 = 0x20;
  static constexpr uint8_t REG_ACC_CTRL4 = 0x23;    // BDU, High Res
  static constexpr uint8_t REG_ACC_OUT_X_L = 0x28;  // | 0x80 auto-increment için
  static constexpr uint8_t REG_ACC_WHO_AM_I = 0x0F;

  // Register Adresleri - Magnetometer [Datasheet Table 26]
  static constexpr uint8_t REG_MAG_CFG_A = 0x60;
  static constexpr uint8_t REG_MAG_CFG_C = 0x62;  // BDU
  static constexpr uint8_t REG_MAG_OUT_X_L = 0x68;
  static constexpr uint8_t REG_MAG_WHO_AM_I = 0x4F;

  // Yardımcı fonksiyonlar
  void writeRegister(uint8_t address, uint8_t reg, uint8_t value);
  uint8_t readRegister(uint8_t address, uint8_t reg);
  void readBytes(uint8_t address, uint8_t reg, uint8_t count, uint8_t* dest);

  SensorData m_data;
  bool m_initialized = false;
};