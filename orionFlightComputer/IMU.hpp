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
#include "Lsm6dsl.hpp"
#include "Config.hpp"
#include <inttypes.h>
#include "DemandProcessor.hpp"

class IMU : public Utilities {
public:
  struct ImuData {
    // İsim değişikliği burada yapıldı:
    LSM6DSL::RawImuData imuRaw;

    float roll;
    float pitch;
    float yaw;
    float timeDelta;
    bool calibrated;
    bool fault;
  };

  static constexpr float MADGWICK_WARM_UP_WEIGHTING = 5.0f;
  static constexpr float MADGWICK_FLIGHT_WEIGHTING = 0.01f;

  IMU(){};
  void begin();
  void operate(const float tdelta, const DemandProcessor::FlightState* const flightState);
  void Madgwick6DOF(const DemandProcessor::FlightState* const flightState);
  bool calibrateGyro();
  void setMadgwickWeighting(float weight);
  bool calibrated();
  bool isFaulted();
  ImuData* const getImuData();
  bool isOk() const {
    return m_i2cReadOk;
  };

private:
  static constexpr uint32_t CALIBRATE_MAX_MOTION = 1000U;
  static constexpr uint32_t CALIBRATE_COUNTS = 1000U;
  static constexpr uint32_t I2C_CLK_1MHZ = 1000000U;
  static constexpr uint32_t CALIBRATION_TIMEOUT = 2000U;

  //Variables
  float m_bMadgwick = 0.0f;
  ImuData m_imu = { 0 };
  int64_t m_xGyroSum = 0;
  int64_t m_yGyroSum = 0;
  int64_t m_zGyroSum = 0;
  uint32_t m_calCount = 0U;
  bool m_i2cReadOk = true;
  uint64_t m_updateTime = 0U;

  float m_q0 = 1.0f;
  float m_q1 = 0.0f;
  float m_q2 = 0.0f;
  float m_q3 = 0.0f;

  //Objects
  LSM6DSL lsm6dsl;
};