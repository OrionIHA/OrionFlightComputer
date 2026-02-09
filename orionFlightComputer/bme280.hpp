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

class BME280 {
public:
  struct BaroData {
    float pres;  // Basınç (mbar)
    float temp;  // Sıcaklık (°C)
    float alti;  // Yükseklik (Metre - Deniz Seviyesi)
  };

  static constexpr uint8_t BME280_ADD = 0x76;

  BME280();

  void begin();

  void setCalibrationOffsets(float, float);
  void readBurstData();
  float getPressure();     // mbar
  float getTemperature();  // °C
  float readAltitude(float);
  float calculateSeaLevelPressure(float, float);
  BaroData getAllData();
  float getSeaLevelPressure();
  void setSeaLevelPressure(float);
  uint8_t whoAmI();

private:
  uint16_t dig_T1;
  int16_t dig_T2;
  int16_t dig_T3;

  uint16_t dig_P1;
  int16_t dig_P2;
  int16_t dig_P3;
  int16_t dig_P4;
  int16_t dig_P5;
  int16_t dig_P6;
  int16_t dig_P7;
  int16_t dig_P8;
  int16_t dig_P9;

  float m_lastTemp = 0.0f;
  float m_lastPress = 0.0f;

  float m_tempOffset = 0.0f;
  float m_pressOffset = 0.0f;

  float SEALEVELPRESSURE_MBAR = 1013.25f;

  void readCalibrationData();
  void writeRegister(uint8_t reg, uint8_t value);
};