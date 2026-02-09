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

#include "GNSS.hpp"

// Constructor: Serial1 ve Özel NMEA Ayrıştırıcıları Başlatılıyor
GNSS::GNSS()
  : m_serialGps(1) {
  // GPGSA cümlesindeki 15. alan PDOP, 17. alan VDOP değeridir.
  m_pdop = new TinyGPSCustom(m_gps, "GNGSA", 15);
  m_vdop = new TinyGPSCustom(m_gps, "GNGSA", 17);
}

GNSS::~GNSS() {
  delete m_pdop;
  delete m_vdop;
}

void GNSS::begin() {
  // GPS Bağlantısını Başlat
  m_serialGps.begin(Config::GNSS_BAUD_RATE, SERIAL_8N1, Config::GNSS_RX_PIN, Config::GNSS_TX_PIN);
}

/**
 * @brief  GPS verisini okur ve struct'ı doldurur.
 * @return Yeni ve geçerli bir konum (FIX) alındıysa true döner.
 */
bool GNSS::readData(GnssData* data) {
  // bool newData = false;

  // 1. Seri Porttan Gelen Veriyi TinyGPS'e Yedir
  // Non-blocking: Döngüyü kilitlemeden tampon bellekteki veriyi okur
  while (m_serialGps.available() > 0) {
    m_gps.encode(m_serialGps.read());
  }

  // 2. Konum Güncellendi mi ve Geçerli mi?
  if (m_gps.location.isUpdated() && m_gps.location.isValid()) {
    // newData = true;

    // --- Temel Konum Verileri ---
    data->latitude = m_gps.location.lat();
    data->longitude = m_gps.location.lng();
    data->altitude = m_gps.altitude.meters();

    // --- Zaman Verileri ---
    if (m_gps.date.isValid()) {
      data->year = m_gps.date.year();
      data->month = m_gps.date.month();
      data->day = m_gps.date.day();
    }

    if (m_gps.time.isValid()) {
      data->hour = m_gps.time.hour();
      data->minute = m_gps.time.minute();
      data->second = m_gps.time.second();
    }

    // --- Uydu ve Hız ---
    data->satellites = (uint8_t)m_gps.satellites.value();
    data->speed = (float)m_gps.speed.kmph();

    // --- DOP Değerleri (GPGSA Cümlesinden) ---
    data->hdop = (float)m_gps.hdop.hdop();

    // PDOP: Custom nesneden string olarak gelir, float'a çevirip atarız.
    if (m_pdop->isValid()) {
      data->pdop = (float)atof(m_pdop->value());
    } else {
      data->pdop = 99;  // Veri yoksa yüksek değer (kötü hassasiyet)
    }

    // VDOP: Custom nesneden string olarak gelir.
    if (m_vdop->isValid()) {
      data->vdop = (float)atof(m_vdop->value());
    } else {
      data->vdop = 99;
    }

    data->fix = true;
  } else {
    // Uydu sayısı 0 ise fix yoktur
    if (m_gps.satellites.value() == 0) {
      data->fix = false;
    }
    return false;
  }

  return true;
}