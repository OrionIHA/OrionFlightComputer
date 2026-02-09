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
* @file   OrionFlightController.ino
* @brief  Contains Arduino setup and loop.
*/

#include "FlightControl.hpp"

// Create instance of FlightControl...
FlightControl orionFlightComputer = FlightControl();


void setup() {
  orionFlightComputer.begin();
}


void loop() {
  orionFlightComputer.operate();
}