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

#include "Lsm6dsl.hpp"

// LSM6DSL Register Adresleri (Burst Read için)
// OUT_TEMP_L (0x20) adresinden başlayarak 14 byte okuyacağız:
// Temp(2) + Gyro(6) + Accel(6) ardışık adreslerdedir.
static constexpr uint8_t LSM6DSL_OUT_TEMP_L = 0x20;

/**
* @brief    Constructor
*/
LSM6DSL::LSM6DSL() {
  // Constructor
}

/**
* @brief    Sensörü I2C hattında başlatır.
*/
void LSM6DSL::begin() {
  Wire.setPins(Config::I2C_SDA, Config::I2C_SCL);
  m_lsm.begin(LSM6DSL_ADD);

  // ÖNEMLİ: I2C Hızını 1MHz (Fast Mode) yapıyoruz.
  // begin fonksiyonundan sonra çağrılmalı.
  Wire.setClock(1000000);
}

/**
* @brief    IMU.cpp kontrolü için sensör ID'si.
*/
uint8_t
LSM6DSL::whoAmI() {
  uint8_t id = 0;
  m_lsm.readRegister(0x0F, &id);
  return id;
}

/**
* @brief    Sensör ayarlarını yapılandırır.
*/
void LSM6DSL::initialise() {
  // 1. I2C Pinlerini Config'den ayarla
  Wire.setPins(Config::I2C_SDA, Config::I2C_SCL);

  // 2. Kütüphane Ayar Yapısını Hazırla
  SensorSettings settings;

  // --- Gyro Ayarları ---
  settings.gyroEnabled = 1;

  if constexpr (Config::USE_250_DEGS_SECOND) {
    settings.gyroRange = 245;
  } else if constexpr (Config::USE_500_DEGS_SECOND) {
    settings.gyroRange = 500;
  } else {
    settings.gyroRange = 2000;
  }

  // Yüksek Performans için Hız Ayarları
  settings.gyroSampleRate = 833;  // 416Hz yerine 833Hz (veya 1.66kHz) deneyebiliriz, şimdilik 833 güvenli.
  settings.gyroBandWidth = 400;
  settings.gyroFifoEnabled = 1;
  settings.gyroFifoDecimation = 1;

  // --- İvmeölçer Ayarları ---
  settings.accelEnabled = 1;
  settings.accelRange = 16;
  settings.accelSampleRate = 833;  // Gyro ile senkron
  settings.accelBandWidth = 400;
  settings.accelFifoEnabled = 1;
  settings.accelFifoDecimation = 1;

  // --- Diğer Ayarlar ---
  settings.tempEnabled = 1;
  settings.commMode = 1;

  // 3. Sensörü Başlat
  m_lsm.begin(LSM6DSL_ADD, &settings);

  // Performans için hızı artırıyoruz
  Wire.setClock(1000000);

  delay(50);
}

/**
* @brief    Tek seferde tüm veriyi okur (Burst Read).
*/
bool LSM6DSL::readData(RawImuData* const data) {
  // Tampon bellek: Temp(2) + Gyro(6) + Accel(6) = 14 Byte
  uint8_t buffer[14];

  // Deneyap kütüphanesinin (LSM6DSMCore) sağladığı toplu okuma fonksiyonu.
  // Tek bir I2C işlemi ile 14 byte'ı birden çekiyoruz.
  // Bu işlem performansı 10 kat artıracaktır.
  m_lsm.readRegisterRegion(LSM6DSL_OUT_TEMP_L, buffer, 14);

  // Buffer'dan Ham Verileri Birleştirme (Little Endian: Low Byte önce gelir)
  int16_t rawTemp = (int16_t)((buffer[1] << 8) | buffer[0]);

  int16_t rawGx = (int16_t)((buffer[3] << 8) | buffer[2]);
  int16_t rawGy = (int16_t)((buffer[5] << 8) | buffer[4]);
  int16_t rawGz = (int16_t)((buffer[7] << 8) | buffer[6]);

  int16_t rawAx = (int16_t)((buffer[9] << 8) | buffer[8]);
  int16_t rawAy = (int16_t)((buffer[11] << 8) | buffer[10]);
  int16_t rawAz = (int16_t)((buffer[13] << 8) | buffer[12]);

  // --- Ölçekleme Çarpanları (Compile-Time) ---
  // Gyro: mdps/LSB -> dps'e çevirmek için 1000'e bölüyoruz.
  constexpr float GYRO_SCALE_250 = 0.00875f;
  constexpr float GYRO_SCALE_500 = 0.01750f;
  constexpr float GYRO_SCALE_2000 = 0.070f;

  // Accel: mg/LSB -> g'ye çevirmek için 1000'e bölüyoruz.
  // 16g modunda hassasiyet 0.488 mg/LSB'dir.
  constexpr float ACCEL_SCALE_16G = 0.000488f;

  float gyroScale;
  if constexpr (Config::USE_250_DEGS_SECOND) gyroScale = GYRO_SCALE_250;
  else if constexpr (Config::USE_500_DEGS_SECOND) gyroScale = GYRO_SCALE_500;
  else gyroScale = GYRO_SCALE_2000;

  // --- Verileri İşleme ve Yönlendirme ---

  float gx, gy, gz, ax, ay, az;

  // Ham verileri fiziksel büyüklüğe çevir
  gx = (float)rawGx * gyroScale;
  gy = (float)rawGy * gyroScale;
  gz = (float)rawGz * gyroScale;

  ax = (float)rawAx * ACCEL_SCALE_16G;
  ay = (float)rawAy * ACCEL_SCALE_16G;
  az = (float)rawAz * ACCEL_SCALE_16G;

  // Sıcaklık (LSM6DSL'de LSB/256 + 25 formülü genelde kullanılır ama
  // Deneyap kütüphanesi doğrudan scale ediyor olabilir. Basit dönüşüm:)
  data->temperature = (rawTemp / 256) + 25;

  // Config Oryantasyonuna Göre Atama
  if constexpr (Config::IMU_ROLLED_RIGHT_90) {
    data->accel_X = ay;
    data->accel_Y = -ax;
    data->accel_Z = az;
    data->gyro_X = gy;
    data->gyro_Y = -gx;
    data->gyro_Z = gz;
    data->rawGyro_X = rawGy;
    data->rawGyro_Y = -rawGx;
    data->rawGyro_Z = rawGz;
  } else if constexpr (Config::IMU_ROLLED_180) {
    data->accel_X = ax;
    data->accel_Y = -ay;
    data->accel_Z = -az;
    data->gyro_X = gx;
    data->gyro_Y = -gy;
    data->gyro_Z = -gz;
    data->rawGyro_X = rawGx;
    data->rawGyro_Y = -rawGy;
    data->rawGyro_Z = -rawGz;
  } else {
    data->accel_X = ax;
    data->accel_Y = ay;
    data->accel_Z = az;
    data->gyro_X = gx;
    data->gyro_Y = gy;
    data->gyro_Z = gz;
    data->rawGyro_X = rawGx;
    data->rawGyro_Y = rawGy;
    data->rawGyro_Z = rawGz;
  }

  // Kalibrasyon Ofsetlerini Çıkarma
  data->gyro_X -= (float)data->gyroOffset_X * gyroScale;
  data->gyro_Y -= (float)data->gyroOffset_Y * gyroScale;
  data->gyro_Z -= (float)data->gyroOffset_Z * gyroScale;

  // Debug Çıktısı (Throttle Edilmiş)
  if constexpr (Config::DEBUG_LSM6DSL) {
    static uint32_t lastPrintTime = 0;
    uint32_t now = millis();

    // Saniyede 10 kere (her 100ms'de bir) yazdır
    if (now - lastPrintTime >= 100) {
      Serial.print("\t gx:");
      Serial.print(data->gyro_X);
      Serial.print("\t gy:");
      Serial.print(data->gyro_Y);
      Serial.print("\t gz:");
      Serial.print(data->gyro_Z);
      Serial.print("\t ax:");
      Serial.print(data->accel_X);
      Serial.print("\t ay:");
      Serial.print(data->accel_Y);
      Serial.print("\t az:");
      Serial.println(data->accel_Z);
      lastPrintTime = now;
    }
  }

  return true;
}