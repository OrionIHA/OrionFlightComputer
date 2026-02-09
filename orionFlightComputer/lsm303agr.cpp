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

#include "lsm303agr.hpp"

LSM303AGR::LSM303AGR() {
  ;
}

void LSM303AGR::begin() {
  ;  // Nothing happens
}

float LSM303AGR::getMagnetometer() {
  return 0.f;
}

float LSM303AGR::getAccelerometer() {
  return 0.f;
}

uint8_t LSM303AGR::whoAmI() {
  return 0x33;
}