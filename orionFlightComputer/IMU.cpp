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


#include "IMU.hpp"

void IMU::begin() {
  lsm6dsl.initialise();

  if (lsm6dsl.whoAmI() != 0x6A)  // WHO_AM_I register's value is fixed at 0x6A
  {
    Serial.println("IMU did not intitialise!");
    m_imu.fault = true;
  } else {
    m_imu.fault = false;
  }
}

bool IMU::isFaulted() {
  return m_imu.fault;
}

void IMU::setMadgwickWeighting(float weight) {
  m_bMadgwick = weight;
}

void IMU::Madgwick6DOF(const DemandProcessor::FlightState* const flightState) {
  // ... (Değişken tanımları aynı) ...
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  //Convert gyroscope degrees/sec to radians/sec
  float gyroX = m_imu.imuRaw.gyro_X * 0.0174533f;
  float gyroY = m_imu.imuRaw.gyro_Y * 0.0174533f;
  float gyroZ = m_imu.imuRaw.gyro_Z * 0.0174533f;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * ((-m_q1 * gyroX) - (m_q2 * gyroY) - (m_q3 * gyroZ));
  qDot2 = 0.5f * ((m_q0 * gyroX) + (m_q2 * gyroZ) - (m_q3 * gyroY));
  qDot3 = 0.5f * ((m_q0 * gyroY) - (m_q1 * gyroZ) + (m_q3 * gyroX));
  qDot4 = 0.5f * ((m_q0 * gyroZ) + (m_q1 * gyroY) - (m_q2 * gyroX));

  // VERİ ERİŞİMİ GÜNCELLENDİ:
  if (!((m_imu.imuRaw.accel_X == 0.0f) && (m_imu.imuRaw.accel_Y == 0.0f) && (m_imu.imuRaw.accel_Z == 0.0f))) {
    recipNorm = invSqrt((m_imu.imuRaw.accel_X * m_imu.imuRaw.accel_X) + (m_imu.imuRaw.accel_Y * m_imu.imuRaw.accel_Y) + (m_imu.imuRaw.accel_Z * m_imu.imuRaw.accel_Z));
    float accelX = m_imu.imuRaw.accel_X * recipNorm;
    float accelY = m_imu.imuRaw.accel_Y * recipNorm;
    float accelZ = m_imu.imuRaw.accel_Z * recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * m_q0;
    _2q1 = 2.0f * m_q1;
    _2q2 = 2.0f * m_q2;
    _2q3 = 2.0f * m_q3;
    _4q0 = 4.0f * m_q0;
    _4q1 = 4.0f * m_q1;
    _4q2 = 4.0f * m_q2;
    _8q1 = 8.0f * m_q1;
    _8q2 = 8.0f * m_q2;
    q0q0 = m_q0 * m_q0;
    q1q1 = m_q1 * m_q1;
    q2q2 = m_q2 * m_q2;
    q3q3 = m_q3 * m_q3;

    //Gradient decent algorithm corrective step
    s0 = (_4q0 * q2q2) + (_2q2 * accelX) + (_4q0 * q1q1) - (_2q1 * accelY);
    s1 = (_4q1 * q3q3) - (_2q3 * accelX) + (4.0f * q0q0 * m_q1) - (_2q0 * accelY) - _4q1 + (_8q1 * q1q1) + (_8q1 * q2q2) + (_4q1 * accelZ);
    s2 = (4.0f * q0q0 * m_q2) + (_2q0 * accelX) + (_4q2 * q3q3) - (_2q3 * accelY) - _4q2 + (_8q2 * q1q1) + (_8q2 * q2q2) + (_4q2 * accelZ);
    s3 = (4.0f * q1q1 * m_q3) - (_2q1 * accelX) + (4.0f * q2q2 * m_q3) - (_2q2 * accelY);
    recipNorm = invSqrt((s0 * s0) + (s1 * s1) + (s2 * s2) + (s3 * s3));  //normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Apply feedback step
    qDot1 -= m_bMadgwick * s0;
    qDot2 -= m_bMadgwick * s1;
    qDot3 -= m_bMadgwick * s2;
    qDot4 -= m_bMadgwick * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  m_q0 += qDot1 * m_imu.timeDelta;
  m_q1 += qDot2 * m_imu.timeDelta;
  m_q2 += qDot3 * m_imu.timeDelta;
  m_q3 += qDot4 * m_imu.timeDelta;

  //Normalise quaternion
  recipNorm = invSqrt((m_q0 * m_q0) + (m_q1 * m_q1) + (m_q2 * m_q2) + (m_q3 * m_q3));
  m_q0 *= recipNorm;
  m_q1 *= recipNorm;
  m_q2 *= recipNorm;
  m_q3 *= recipNorm;

  //Compute angles in degrees
  if (DemandProcessor::FlightState::PROP_HANG == *flightState) {
    m_imu.pitch = -fastAtan2(((m_q1 * m_q3) - (m_q0 * m_q2)), 0.5f - ((m_q1 * m_q1) + (m_q2 * m_q2))) * 57.29577951f;
    if constexpr (Config::REVERSE_PROP_HANG_PITCH_CORRECTIONS) {
      m_imu.pitch += 90.0f;
    } else {
      m_imu.pitch -= 90.0f;
    }
    const float rads = constrain(2.0f * ((m_q0 * m_q1) + (m_q2 * m_q3)), -0.999999f, 0.999999f);  //Prevent 'not a number' NaN.
    m_imu.yaw = asin(rads) * 57.29577951f;
    if constexpr (Config::REVERSE_PROP_HANG_YAW_CORRECTIONS) {
      m_imu.yaw = -m_imu.yaw;
    }
    m_imu.roll = 0.0f;
  } else {
    m_imu.roll = fastAtan2((m_q0 * m_q1) + (m_q2 * m_q3), 0.5f - (m_q1 * m_q1) - (m_q2 * m_q2)) * 57.29577951f;
    const float rads = constrain(-2.0f * ((m_q1 * m_q3) - (m_q0 * m_q2)), -0.999999f, 0.999999f);  //Prevent 'not a number' NaN.
    m_imu.pitch = asin(rads) * 57.29577951f;

    if (DemandProcessor::FlightState::AP_WIFI == *flightState) {
      const float rads = constrain(2.0f * ((m_q0 * m_q1) + (m_q2 * m_q3)), -0.999999f, 0.999999f);  //Prevent 'not a number' NaN.
      m_imu.yaw = asin(rads) * 57.29577951f;
    } else {
      m_imu.yaw = 0.0f;
    }
  }

  if constexpr (Config::REVERSE_ROLL_CORRECTIONS) {
    m_imu.roll = -m_imu.roll;
  }

  if constexpr (Config::REVERSE_PITCH_CORRECTIONS) {
    m_imu.pitch = -m_imu.pitch;
  }

  if constexpr (Config::REVERSE_YAW_CORRECTIONS && Config::USE_PROP_HANG_MODE) {
    m_imu.yaw = -m_imu.yaw;
  }

  if constexpr (Config::DEBUG_MADGWICK) {
    const uint64_t nowTime = millis();

    if (m_updateTime <= nowTime) {
      Serial.print("Roll: ");
      Serial.print(m_imu.roll);
      Serial.print(", Pitch: ");
      Serial.print(m_imu.pitch);
      if constexpr (Config::USE_PROP_HANG_MODE) {
        Serial.print(", yaw: ");
        Serial.print(m_imu.yaw);
      }
      Serial.println();
      m_updateTime = nowTime + 100U;
    }
  }
}

bool IMU::calibrateGyro() {
  if constexpr (Config::DEBUG_GYRO_CALIBRATION) { Serial.println("Calibration..."); }

  // VERİ ERİŞİMİ GÜNCELLENDİ: rawGyro değerlerine erişim
  bool motionDetected = ((abs(m_imu.imuRaw.rawGyro_X) + abs(m_imu.imuRaw.rawGyro_Y) + abs(m_imu.imuRaw.rawGyro_Z)) >= CALIBRATE_MAX_MOTION) ? true : false;

  if (motionDetected) {
    if constexpr (Config::DEBUG_GYRO_CALIBRATION) { Serial.println("Calibration reset !"); }
    m_xGyroSum = 0;
    m_yGyroSum = 0;
    m_zGyroSum = 0;
    m_calCount = 0U;
  } else {
    m_xGyroSum += m_imu.imuRaw.rawGyro_X;
    m_yGyroSum += m_imu.imuRaw.rawGyro_Y;
    m_zGyroSum += m_imu.imuRaw.rawGyro_Z;
    m_calCount++;
  }

  if (CALIBRATE_COUNTS > m_calCount) {
    m_imu.calibrated = false;
  } else {
    m_imu.calibrated = true;
    m_imu.imuRaw.gyroOffset_X = static_cast<int16_t>(m_xGyroSum / CALIBRATE_COUNTS);
    m_imu.imuRaw.gyroOffset_Y = static_cast<int16_t>(m_yGyroSum / CALIBRATE_COUNTS);
    m_imu.imuRaw.gyroOffset_Z = static_cast<int16_t>(m_zGyroSum / CALIBRATE_COUNTS);

    if constexpr (Config::DEBUG_GYRO_CALIBRATION) {
      Serial.println("Calibration complete...");
      Serial.print("x: ");
      Serial.print(m_imu.imuRaw.gyroOffset_X);
      Serial.print("\ty: ");
      Serial.print(m_imu.imuRaw.gyroOffset_Y);
      Serial.print("\tz: ");
      Serial.println(m_imu.imuRaw.gyroOffset_Z);
    }
  }

  return m_imu.calibrated;
}

bool IMU::calibrated() {
  return m_imu.calibrated;
}

void IMU::operate(const float tDelta, const DemandProcessor::FlightState* const flightState) {
  m_imu.timeDelta = tDelta;

  m_i2cReadOk = lsm6dsl.readData(&m_imu.imuRaw);

  // MADGWICK FİLTRESİ ÇALIŞMAYA DEVAM EDİYOR
  Madgwick6DOF(flightState);
}

IMU::ImuData* const IMU::getImuData() {
  return &m_imu;
}