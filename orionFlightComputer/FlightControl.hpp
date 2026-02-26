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

/**
* @file   FlightControl.hpp
* @brief  This class contains methods to operate the flight controller.
*/
#pragma once

#include <inttypes.h>
#include <Arduino.h>
#include "Utilities.hpp"
#include "DemandProcessor.hpp"
#include "BatteryMonitor.hpp"
#include "LED.hpp"
#include "PIDF.hpp"
#include "IMU.hpp"
#include "barometer.hpp"
#include "Config.hpp"
#include "SBus.hpp"
#include "Configurator.hpp"
#include "lsm303agr.hpp"
#include "ModelTypes.hpp"


/**
* @class FlightControl
*/

class FlightControl : public Utilities {
public:
  FlightControl(){};
  void operate();
  void begin();

protected:
  //Constants
  static constexpr int32_t ACRO_TRAINER_RECOVERY_RATE = static_cast<int32_t>(Config::ACRO_TRAINER_LEVEL_RATE * 100.0f);

  // Variables
  float m_targetAltitude = 0.0f; // Kilitlenen hedef yükseklik
  bool m_altHoldActive = false;  // Mod durumu

  DemandProcessor::FlightState m_flightState = DemandProcessor::FlightState::CALIBRATE;
  DemandProcessor::FlightState m_lastFlightState = DemandProcessor::FlightState::CALIBRATE;
  IMU::ImuData* imuData;
  ModelBase* myModel;
  BARO::BaroData baroData;
  LSM303AGR::SensorData *magData;

  //Methods
  void modelConfig();
  void doCalibrateState();
  void doDisarmedState();
  void doPassThroughState();
  void doRateState();
  void doLevelledState();
  void doFailSafeState();
  void doWifiApState();
  void doFaultedState();
  void doPropHangState();
  void doAcroTrainerState();
  void checkStateChange();
  void processPIDF(DemandProcessor::Demands* const demands);
  void altitudeHold();

  //Objects
  Led statusLed = Led(Config::LED_ONBOARD);
  LedNeopixel statusLedNeopixel = LedNeopixel(Config::LED_ONBOARD);
  Led externLed = Led(Config::EXT_LED_PIN);


  DemandProcessor rc = DemandProcessor();
  BatteryMonitor batteryMonitor = BatteryMonitor(Config::BATT_ADC_PIN);
  PIDF pitchPIDF = PIDF();
  PIDF yawPIDF = PIDF();
  PIDF rollPIDF = PIDF();
  PIDF altPosPIDF = PIDF();   // Dış Döngü (Altitude -> Velocity)
  PIDF altRatePIDF = PIDF();  // İç Döngü (Velocity -> Pitch)
  IMU imu = IMU();
  BARO baro = BARO();
  LSM303AGR mag = LSM303AGR();
  Configurator config;
};