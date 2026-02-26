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
#include "TinyGPSPlus.h"  // Harici kütüphane

class GNSS {
public:
  struct GnssData {
    float latitude;   // DD.DD
    float longitude;  // DD.DD
    float altitude;   // meter
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t satellites;  // number of satellites
    float pdop;          // Position Dilution of Precision
    float hdop;          // Horizontal Dilution of Percision 
    float vdop;          // Vertical Dilution of Percision
    float speed;         // Speed in kilometer per hour
    bool fix;            // Konumun geçerli olup olmadığı
  };

  GNSS();
  ~GNSS();

  void begin();

  // Veriyi okur ve struct'ı doldurur
  bool readData(GnssData *data);

private:
  // Serial nesnesi
  HardwareSerial m_serialGps;

  // TinyGPS nesnesi
  TinyGPSPlus m_gps;

  // TinyGPSCustom nesneleri (PDOP ve VDOP için)
  // GPGSA cümlesinden veri çekeceğiz.
  TinyGPSCustom *m_pdop;
  TinyGPSCustom *m_vdop;
};