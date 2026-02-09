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
#include <Wire.h>
#include "Config.hpp"
#include <Deneyap_6EksenAtaletselOlcumBirimi.h>

class LSM6DSL {
public:
  struct RawImuData {
    float accel_X;
    float accel_Y;
    float accel_Z;
    float gyro_X;
    float gyro_Y;
    float gyro_Z;

    // Kalibrasyon döngüsü için gerekli raw değerler
    int16_t rawGyro_X;
    int16_t rawGyro_Y;
    int16_t rawGyro_Z;

    // Ofsetler
    int16_t gyroOffset_X;
    int16_t gyroOffset_Y;
    int16_t gyroOffset_Z;

    int16_t temperature;
  };

  static constexpr uint8_t LSM6DSL_ADD = 0x6B;

  LSM6DSL();
  void begin();
  void initialise();
  bool readData(RawImuData* const data);
  uint8_t whoAmI();

private:
  LSM6DSM m_lsm;

  // Config kontrolü
  static_assert((Config::USE_250_DEGS_SECOND || Config::USE_500_DEGS_SECOND), "Config.hpp icinde Gyro Scale secilmeli.");
};