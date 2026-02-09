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
#include "Utilities.hpp"
#include "Config.hpp"
#include "bme280.hpp"
#include "GNSS.hpp"

class BARO {
public:
  struct BaroData {
    float altitude;    // Filtered altitude (m)
    float velocity;    // Filtered vertical velocity (m/s)
    float rawBaroAlt;  // Raw barometer altitude
    float rawGnssAlt;  // Raw GNSS altitude
    bool gnssFix;
  };

  BARO();
  void begin();
  void operate();
  void setSimulatedFiltresizBaro(float pressurePa, float gnssAlt);

  BaroData getData() const {
    return m_data;
  };
  bool isFaulted() const {
    return m_fault;
  };

private:
  void ExtendedKalmanFilter(float dt, bool hasNewBaro, bool hasNewGnss);

  // Minimum number of satellite to operate GNSS
  static constexpr uint8_t MIN_SATE_TO_OP = 4;
  // Maximum number of vdop to read GNSS altitude
  static constexpr float MAX_VDOP_TO_OP = 5.0f;

  // External objects
  BME280 bme;
  GNSS gnss;

  // Data structers
  GNSS::GnssData gnssData = { 0 };
  BaroData m_data = { 0 };
  bool m_fault = false;

  // --- Value of Extended Kalman Filter ---
  // State Vector [x]
  // x[0] = Altitude
  // x[1] = Vertical Velocity
  float x[2] = { 0.0f, 0.0f };

  // Covariance Matrix [P] (Error Estimation)
  // 2x2 Matris: P[0][0], P[0][1], P[1][0], P[1][1]
  float P[2][2] = {
    { 1.0f, 0.0f },
    { 0.0f, 1.0f }
  };

  // Process Noise Matrix [Q]
  // This define how much we don't trust to calculations
  // High Q -> Less trust to calculations than measurements
  // Low Q -> More trust to calculations
  float Q[2][2] = {
    { 0.001f, 0.0001f },
    { 0.0001f, 0.01f }
  };

  // Measurement Noise Matix [R]
  // This define how much we don't trust to measurements
  float R_baro = 1.0f;   // Noise of Barometer (Low = Reliable)
  float R_gnss = 10.0f;  // Noise of GNSS (High = Unreliable)

  // Timing
  uint32_t lastUpdateMicros = 0;
};