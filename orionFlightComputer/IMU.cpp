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
  // 1. LSM6DSL Başlat
  lsm6dsl.initialise();

  // 2. LSM303AGR Başlat
  if (!lsm303.begin()) {
    Serial.println("LSM303AGR (Mag/Accel) did not initialise!");
    // Opsiyonel: m_imu.fault = true;
  }

  // LSM6DSL Kontrolü
  if (lsm6dsl.whoAmI() != 0x6A) {
    Serial.println("IMU (LSM6DSL) did not intitialise!");
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

void IMU::Madgwick9DOF(const DemandProcessor::FlightState* const flightState) {
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3;
  float q0q0, q1q1, q2q2, q3q3;

  // 1. Gyro rad/s dönüşümü
  float gyroX = m_imu.imuRaw.gyro_X * 0.0174533f;
  float gyroY = m_imu.imuRaw.gyro_Y * 0.0174533f;
  float gyroZ = m_imu.imuRaw.gyro_Z * 0.0174533f;

  // 2. Quaternion değişim hızı
  qDot1 = 0.5f * ((-m_q1 * gyroX) - (m_q2 * gyroY) - (m_q3 * gyroZ));
  qDot2 = 0.5f * ((m_q0 * gyroX) + (m_q2 * gyroZ) - (m_q3 * gyroY));
  qDot3 = 0.5f * ((m_q0 * gyroY) - (m_q1 * gyroZ) + (m_q3 * gyroX));
  qDot4 = 0.5f * ((m_q0 * gyroZ) + (m_q1 * gyroY) - (m_q2 * gyroX));

  // 3. İvmeölçer verisi varsa (0 değilse) düzeltme yap
  if (!((m_imu.imuRaw.accel_X == 0.0f) && (m_imu.imuRaw.accel_Y == 0.0f) && (m_imu.imuRaw.accel_Z == 0.0f))) {

    // İvme normalizasyon
    recipNorm = invSqrt((m_imu.imuRaw.accel_X * m_imu.imuRaw.accel_X) + (m_imu.imuRaw.accel_Y * m_imu.imuRaw.accel_Y) + (m_imu.imuRaw.accel_Z * m_imu.imuRaw.accel_Z));
    float ax = m_imu.imuRaw.accel_X * recipNorm;
    float ay = m_imu.imuRaw.accel_Y * recipNorm;
    float az = m_imu.imuRaw.accel_Z * recipNorm;

    // Manyetometre normalizasyon
    recipNorm = invSqrt((m_imu.magX * m_imu.magX) + (m_imu.magY * m_imu.magY) + (m_imu.magZ * m_imu.magZ));
    float mx = m_imu.magX * recipNorm;
    float my = m_imu.magY * recipNorm;
    float mz = m_imu.magZ * recipNorm;

    // Yardımcı değişkenler
    _2q0 = 2.0f * m_q0;
    _2q1 = 2.0f * m_q1;
    _2q2 = 2.0f * m_q2;
    _2q3 = 2.0f * m_q3;
    q0q0 = m_q0 * m_q0;
    q1q1 = m_q1 * m_q1;
    q2q2 = m_q2 * m_q2;
    q3q3 = m_q3 * m_q3;

    // Dünya manyetik alanı referansı
    hx = mx * q0q0 - _2q0 * my * m_q3 + _2q0 * mz * m_q2 + mx * q1q1 + 2.0f * m_q1 * my * m_q2 + 2.0f * m_q1 * mz * m_q3 - mx * q2q2 - mx * q3q3;
    hy = 2.0f * m_q0 * mx * m_q3 + my * q0q0 - _2q0 * mz * m_q1 + 2.0f * m_q1 * mx * m_q2 - my * q1q1 + my * q2q2 + 2.0f * m_q2 * mz * m_q3 - my * q3q3;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -2.0f * m_q0 * mx * m_q2 + 2.0f * m_q0 * my * m_q1 + mz * q0q0 + 2.0f * m_q1 * mx * m_q3 - mz * q1q1 + 2.0f * m_q2 * my * m_q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient Descent Hata Adımı (Manyetik + İvme)
    s0 = -_2q2 * (2.0f * (m_q1 * m_q3 - m_q0 * m_q2) - ax) + _2q1 * (2.0f * (m_q0 * m_q1 + m_q2 * m_q3) - ay) - _2bz * m_q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (m_q1 * m_q3 - m_q0 * m_q2) - mx) + (-_2bx * m_q3 + _2bz * m_q1) * (_2bx * (m_q1 * m_q2 - m_q0 * m_q3) + _2bz * (m_q0 * m_q1 + m_q2 * m_q3) - my) + _2bx * m_q2 * (_2bx * (m_q0 * m_q2 + m_q1 * m_q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * (m_q1 * m_q3 - m_q0 * m_q2) - ax) + _2q0 * (2.0f * (m_q0 * m_q1 + m_q2 * m_q3) - ay) - 4.0f * m_q1 * (1.0f - 2.0f * (q1q1 + q2q2) - az) + _2bz * m_q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (m_q1 * m_q3 - m_q0 * m_q2) - mx) + (_2bx * m_q2 + _2bz * m_q0) * (_2bx * (m_q1 * m_q2 - m_q0 * m_q3) + _2bz * (m_q0 * m_q1 + m_q2 * m_q3) - my) + (_2bx * m_q3 - _4bz * m_q1) * (_2bx * (m_q0 * m_q2 + m_q1 * m_q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * (m_q1 * m_q3 - m_q0 * m_q2) - ax) + _2q3 * (2.0f * (m_q0 * m_q1 + m_q2 * m_q3) - ay) - 4.0f * m_q2 * (1.0f - 2.0f * (q1q1 + q2q2) - az) + (-_4bx * m_q2 - _2bz * m_q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (m_q1 * m_q3 - m_q0 * m_q2) - mx) + (_2bx * m_q1 + _2bz * m_q3) * (_2bx * (m_q1 * m_q2 - m_q0 * m_q3) + _2bz * (m_q0 * m_q1 + m_q2 * m_q3) - my) + (_2bx * m_q0 - _4bz * m_q2) * (_2bx * (m_q0 * m_q2 + m_q1 * m_q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * (m_q1 * m_q3 - m_q0 * m_q2) - ax) + _2q2 * (2.0f * (m_q0 * m_q1 + m_q2 * m_q3) - ay) + (-_4bx * m_q3 + _2bz * m_q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (m_q1 * m_q3 - m_q0 * m_q2) - mx) + (-_2bx * m_q0 + _2bz * m_q2) * (_2bx * (m_q1 * m_q2 - m_q0 * m_q3) + _2bz * (m_q0 * m_q1 + m_q2 * m_q3) - my) + _2bx * m_q1 * (_2bx * (m_q0 * m_q2 + m_q1 * m_q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Feedback uygula
    qDot1 -= m_bMadgwick * s0;
    qDot2 -= m_bMadgwick * s1;
    qDot3 -= m_bMadgwick * s2;
    qDot4 -= m_bMadgwick * s3;
  }

  // 4. Entegrasyon
  m_q0 += qDot1 * m_imu.timeDelta;
  m_q1 += qDot2 * m_imu.timeDelta;
  m_q2 += qDot3 * m_imu.timeDelta;
  m_q3 += qDot4 * m_imu.timeDelta;

  recipNorm = invSqrt(m_q0 * m_q0 + m_q1 * m_q1 + m_q2 * m_q2 + m_q3 * m_q3);
  m_q0 *= recipNorm;
  m_q1 *= recipNorm;
  m_q2 *= recipNorm;
  m_q3 *= recipNorm;

  // --- BURASI SENİN İSTEDİĞİN ÖZEL MANTIK KISMI ---

  // 5. Euler Açılarına Dönüşüm
  if (DemandProcessor::FlightState::PROP_HANG == *flightState) {
    m_imu.pitch = -fastAtan2(((m_q1 * m_q3) - (m_q0 * m_q2)), 0.5f - ((m_q1 * m_q1) + (m_q2 * m_q2))) * 57.29577951f;
    if constexpr (Config::REVERSE_PROP_HANG_PITCH_CORRECTIONS) {
      m_imu.pitch += 90.0f;
    } else {
      m_imu.pitch -= 90.0f;
    }
    const float rads = constrain(2.0f * ((m_q0 * m_q1) + (m_q2 * m_q3)), -0.999999f, 0.999999f);
    m_imu.yaw = asin(rads) * 57.29577951f;
    if constexpr (Config::REVERSE_PROP_HANG_YAW_CORRECTIONS) {
      m_imu.yaw = -m_imu.yaw;
    }
    m_imu.roll = 0.0f;
  } else {
    m_imu.roll = fastAtan2((m_q0 * m_q1) + (m_q2 * m_q3), 0.5f - (m_q1 * m_q1) - (m_q2 * m_q2)) * 57.29577951f;
    const float rads = constrain(-2.0f * ((m_q1 * m_q3) - (m_q0 * m_q2)), -0.999999f, 0.999999f);
    m_imu.pitch = asin(rads) * 57.29577951f;

    // Yaw artık pusula verisiyle hesaplandığı için 0.0f yerine gerçek değeri almalı
    m_imu.yaw = fastAtan2((m_q1 * m_q2 + m_q0 * m_q3), 0.5f - (m_q2 * m_q2 + m_q3 * m_q3)) * 57.29577951f;
  }

  // Config Tersleme Kontrolleri
  if constexpr (Config::REVERSE_ROLL_CORRECTIONS) { m_imu.roll = -m_imu.roll; }
  if constexpr (Config::REVERSE_PITCH_CORRECTIONS) { m_imu.pitch = -m_imu.pitch; }
  if constexpr (Config::REVERSE_YAW_CORRECTIONS && Config::USE_PROP_HANG_MODE) { m_imu.yaw = -m_imu.yaw; }

  // 6. Debug Çıktıları
  if constexpr (Config::DEBUG_MADGWICK) {
    const uint64_t nowTime = millis();
    if (m_updateTime <= nowTime) {
      Serial.print("Roll: ");
      Serial.print(m_imu.roll);
      Serial.print(", Pitch: ");
      Serial.print(m_imu.pitch);
      Serial.print(", Yaw: ");
      Serial.println(m_imu.yaw);  // Yaw her zaman yazılsın
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

bool IMU::calibrateMagnetometer() {
  // İşlem başlarken güvenliği devreye al (uçuşa izin verme)
  m_imu.magCalibrated = false;

  if constexpr (Config::DEBUG_MAG_CALIB) {
    Serial.println("--- MANYETOMETRE KALIBRASYONU BASLADI ---");
    Serial.println("Lutfen ucagi/sensoru tum eksenlerde (8 cizerek) dondurun...");
  }

  // Sınırları ters başlatıyoruz
  float minX = 10000.0f, maxX = -10000.0f;
  float minY = 10000.0f, maxY = -10000.0f;
  float minZ = 10000.0f, maxZ = -10000.0f;

  uint32_t startTime = millis();

  // Belirlenen süre boyunca sensörü oku ve Max/Min değerleri bul
  while (millis() - startTime < Config::MAG_CALIB_DURATION) {
    lsm303.readMagnetometer();
    LSM303AGR::SensorData* data = lsm303.getData();

    if (data->magX < minX) minX = data->magX;
    if (data->magX > maxX) maxX = data->magX;

    if (data->magY < minY) minY = data->magY;
    if (data->magY > maxY) maxY = data->magY;

    if (data->magZ < minZ) minZ = data->magZ;
    if (data->magZ > maxZ) maxZ = data->magZ;

    delay(10);  // 100Hz ODR için
  }

  // Yarıçapları (Chord) hesapla
  float chordX = (maxX - minX) / 2.0f;
  float chordY = (maxY - minY) / 2.0f;
  float chordZ = (maxZ - minZ) / 2.0f;

  // --- SAĞLAMLIK KONTROLÜ (VALIDATION CHECK) ---
  // Dünya manyetik alanına göre, sensör hareket ettirildiyse yarıçaplar belli bir değerden büyük olmalıdır.
  // 0.1f Gauss, hareket edilip edilmediğini anlamak için güvenli bir eşiktir.
  constexpr float MIN_VALID_CHORD = 0.1f;

  if (chordX > MIN_VALID_CHORD && chordY > MIN_VALID_CHORD && chordZ > MIN_VALID_CHORD) {

    // 1. Hard Iron Düzeltmesi (Merkez Kayması)
    m_magOffsetX = (maxX + minX) / 2.0f;
    m_magOffsetY = (maxY + minY) / 2.0f;
    m_magOffsetZ = (maxZ + minZ) / 2.0f;

    // 2. Soft Iron Düzeltmesi (Ölçeklendirme)
    float avgChord = (chordX + chordY + chordZ) / 3.0f;
    m_magScaleX = avgChord / chordX;
    m_magScaleY = avgChord / chordY;
    m_magScaleZ = avgChord / chordZ;

    // Matematiksel olarak güvenli, kalibrasyon başarılı!
    m_imu.magCalibrated = true;

    if constexpr (Config::DEBUG_MAG_CALIB) {
      Serial.println("--- KALIBRASYON BASARILI ---");
      Serial.print("Offset X: ");
      Serial.print(m_magOffsetX);
      Serial.print(" \tY: ");
      Serial.print(m_magOffsetY);
      Serial.print(" \tZ: ");
      Serial.println(m_magOffsetZ);

      Serial.print("Scale  X: ");
      Serial.print(m_magScaleX);
      Serial.print(" \tY: ");
      Serial.print(m_magScaleY);
      Serial.print(" \tZ: ");
      Serial.println(m_magScaleZ);
    }
  } else {
    // Sensör hareket ettirilmemiş veya değerler çok dar bir aralıkta kalmış
    m_imu.magCalibrated = false;

    if constexpr (Config::DEBUG_MAG_CALIB) {
      Serial.println("--- HATA: KALIBRASYON BASARISIZ ---");
      Serial.println("Sebep: Yeterli hareket saglanmadi. Ucak sabit kalmis olabilir.");
      Serial.print("Algilanan hareket - X: ");
      Serial.print(chordX);
      Serial.print(" Y: ");
      Serial.print(chordY);
      Serial.print(" Z: ");
      Serial.println(chordZ);
    }
  }

  return m_imu.magCalibrated;
}

bool IMU::calibrated() {
  return m_imu.calibrated;
}

bool IMU::isMagCalibrated() {
  return m_imu.magCalibrated;
}

void IMU::operate(const float tDelta, const DemandProcessor::FlightState* const flightState) {
  m_imu.timeDelta = tDelta;

  // 1. Jiroskop ve İvmeölçer HER DÖNGÜDE (1kHz) okunur
  m_i2cReadOk = lsm6dsl.readData(&m_imu.imuRaw);
  lsm303.readAccelerometer();

  // 2. Manyetometre SADECE 10ms'de bir (100Hz) okunur
  uint32_t nowTime = millis();
  if (nowTime - m_lastMagUpdate >= 10U) {
    lsm303.readMagnetometer();
    m_lastMagUpdate = nowTime;
  }

  // Verileri çek (Manyetometre güncellenmediyse, getData() eski veriyi verir)
  LSM303AGR::SensorData* data303 = lsm303.getData();

  // 3. Verileri IMU yapısına kaydet
  m_imu.magX = data303->magX;
  m_imu.magY = data303->magY;
  m_imu.magZ = data303->magZ;

  // 4. İvmeölçer Füzyonu (Sensor Fusion)
  // Eğer her iki sensör de sağlıklıysa ortalamasını al
  if (m_i2cReadOk && data303->accelReady) {
    m_imu.imuRaw.accel_X = (m_imu.imuRaw.accel_X + data303->accelX) / 2.0f;
    m_imu.imuRaw.accel_Y = (m_imu.imuRaw.accel_Y + data303->accelY) / 2.0f;
    m_imu.imuRaw.accel_Z = (m_imu.imuRaw.accel_Z + data303->accelZ) / 2.0f;
  }
  // Eğer LSM6DSL bozuksa veya veri okunamadıysa, yedek olarak LSM303 kullan
  else if (!m_i2cReadOk && data303->accelReady) {
    m_imu.imuRaw.accel_X = data303->accelX;
    m_imu.imuRaw.accel_Y = data303->accelY;
    m_imu.imuRaw.accel_Z = data303->accelZ;
  }  // Eğer LSM303AGR'den veri gelmiyorsa ama LSM6DSL sağlamsa hiçbir değişik yapılamadığından imuRaw yapısının verileri LSM6DSL kendi verileri olarak kalır.

  // 5. Filtre 1kHz hızında, eski (ama yeterince güncel) manyetometre verisiyle çalışmaya devam eder
  Madgwick9DOF(flightState);
}

IMU::ImuData* const IMU::getImuData() {
  return &m_imu;
}