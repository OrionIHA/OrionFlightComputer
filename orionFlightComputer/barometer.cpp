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

#include "barometer.hpp"

BARO::BARO() {
  // Empty
}

void BARO::begin() {
  bme.begin();
  gnss.begin();

  if constexpr (Config::DEBUG_BME280 || Config::DEBUG_GNSS) {
    Serial.println("Barometer & GNSS Initializing...");
  }

  uint32_t startTime = millis();
  bool gpsLocked = false;

  while ((millis() - startTime) < 5000) {  // It's 5s for tests, it go higher during flight
    if (gnss.readData(&gnssData)) {
      if (gnssData.satellites >= MIN_SATE_TO_OP && gnssData.vdop < MAX_VDOP_TO_OP && gnssData.fix) {
        gpsLocked = true;
        break;
      }
    }
    delay(10);
  }

  bme.readBurstData();

  if (gpsLocked) {
    x[0] = gnssData.altitude;
    float calibratedSeaLevel = bme.calculateSeaLevelPressure(gnssData.altitude, bme.getPressure());
    bme.setSeaLevelPressure(calibratedSeaLevel);
    if constexpr (Config::DEBUG_GNSS) { Serial.println("GPS Locked. Baro Calibrated."); }
  } else {
    x[0] = bme.readAltitude(bme.getSeaLevelPressure());
    if constexpr (Config::DEBUG_GNSS) { Serial.println("GPS NOT Locked."); }
  }

  x[1] = 0.0f;
  lastUpdateMicros = micros();
}

void BARO::operate() {
  // Zaman Farkı Hesapla
  uint32_t now = micros();
  float dt = (float)(now - lastUpdateMicros) / 1000000.0f;
  lastUpdateMicros = now;

  if (dt > 0.1f) dt = 0.01f;

  // Güncel veri kontrol bayrakları
  bool newGnssData = false;
  bool newBaroData = false;

  // Barometreden en veri okunma zamanı (ms)
  static uint32_t lastBaroRead = 0;

  // 1000ms / 100Hz = 10ms
  if ((millis() - lastBaroRead) >= (1000.f / Config::BME_REFRESH_RATE)) {
    bme.readBurstData();
    lastBaroRead = millis();
    newBaroData = true;

    // Ham veriyi güncellenir
    m_data.rawBaroAlt = bme.readAltitude(bme.getSeaLevelPressure());
  }

  if (newGnssData) {
    m_data.rawGnssAlt = gnssData.altitude;
    m_data.gnssFix = gnssData.fix;
  }
  

  // EKF Çalıştır (Yeni veri durumunu parametre olarak gönderiyoruz)
  ExtendedKalmanFilter(dt, newBaroData, newGnssData);

  // Debug Çıktıları
  if constexpr (Config::DEBUG_BME280) {
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint > 200) {  // 5Hz debug
      Serial.print("\tSicaklik (°C):\t");
      Serial.println(bme.getTemperature());
      Serial.print("\tBasinc (mbar):\t");
      Serial.println(bme.getPressure());
      Serial.print("\tIrtifa (m):\t");
      Serial.println(bme.readAltitude(bme.getSeaLevelPressure()));
      lastPrint = millis();
    }
  }

  if constexpr (Config::DEBUG_GNSS) {
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint > 200) {  // 5Hz debug
      Serial.print("\tlat:\t");
      Serial.print(gnssData.latitude, 6);
      Serial.print("\tlng:\t");
      Serial.print(gnssData.longitude, 6);
      Serial.print("\talt:\t");
      Serial.println(gnssData.altitude);

      Serial.print("\tsat:\t");
      Serial.print(gnssData.satellites);
      Serial.print("\tpdop:\t");
      Serial.print(gnssData.pdop);
      Serial.print("\thdop:\t");
      Serial.print(gnssData.hdop);
      Serial.print("\tvdop:\t");
      Serial.println(gnssData.vdop);

      Serial.print("\tdurum:\t");
      Serial.println(gnssData.fix ? "Fixed" : "Unfixed");

      lastPrint = millis();
    }
  }
}

/**
 * @brief Extended Kalman Filter implementation
 * @param dt Time delta
 * @param hasNewBaro True if fresh barometer data is available
 * @param hasNewGnss True if fresh GNSS data is available
 */
void BARO::ExtendedKalmanFilter(float dt, bool hasNewBaro, bool hasNewGnss) {

  // --- A. PREDICTION STEP (TAHMİN) --- HER DÖNGÜDE ÇALIŞIR (1kHz+)

  float x0_pred = x[0] + x[1] * dt;
  float x1_pred = x[1];

  float P00_pred = P[0][0] + dt * (P[1][0] + P[0][1]) + dt * dt * P[1][1] + Q[0][0];
  float P01_pred = P[0][1] + dt * P[1][1] + Q[0][1];
  float P10_pred = P[1][0] + dt * P[1][1] + Q[1][0];
  float P11_pred = P[1][1] + Q[1][1];

  x[0] = x0_pred;
  x[1] = x1_pred;
  P[0][0] = P00_pred;
  P[0][1] = P01_pred;
  P[1][0] = P10_pred;
  P[1][1] = P11_pred;

  // --- B. UPDATE STEP (BAROMETER) --- SADECE YENİ VERİ VARSA ÇALIŞIR (100Hz)
  if (hasNewBaro) {
    float z = m_data.rawBaroAlt;
    float y = z - x[0];

    float S = P[0][0] + R_baro;
    float K0 = P[0][0] / S;
    float K1 = P[1][0] / S;

    x[0] = x[0] + K0 * y;
    x[1] = x[1] + K1 * y;

    float P00_new = P[0][0] - K0 * P[0][0];
    float P01_new = P[0][1] - K0 * P[0][1];
    float P10_new = P[1][0] - K1 * P[0][0];
    float P11_new = P[1][1] - K1 * P[0][1];

    P[0][0] = P00_new;
    P[0][1] = P01_new;
    P[1][0] = P10_new;
    P[1][1] = P11_new;
  }

  // --- C. UPDATE STEP (GNSS) --- SADECE YENİ VERİ VARSA ÇALIŞIR (~5-10Hz)
  if (hasNewGnss && m_data.gnssFix) {
    float z = m_data.rawGnssAlt;
    float y = z - x[0];

    float S = P[0][0] + R_gnss;
    float K0 = P[0][0] / S;
    float K1 = P[1][0] / S;

    x[0] = x[0] + K0 * y;
    x[1] = x[1] + K1 * y;

    float P00_new = P[0][0] - K0 * P[0][0];
    float P01_new = P[0][1] - K0 * P[0][1];
    float P10_new = P[1][0] - K1 * P[0][0];
    float P11_new = P[1][1] - K1 * P[0][1];

    P[0][0] = P00_new;
    P[0][1] = P01_new;
    P[1][0] = P10_new;
    P[1][1] = P11_new;
  }

  m_data.altitude = x[0];
  m_data.velocity = x[1];
}