// Copyright 2023 michael. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#ifndef BRAKE_h
#define BRAKE_h

#include <SPI.h>  // using CAN_BUS_Shield from Seeed Studio
#include "pin.h"
#include "mcp2515_can.h"
#include "base_module.h"
#include "macros.h"
const unsigned long COMMAND_ID = 0xFF0000;  // Default command ID for CAN Actuator
const unsigned long REPORT_ID = 0xFF0001;   // Default report ID for CAN Actuator
const char CLUTCH_MOTOR_OFF[8] = { 0x0F, 0x4A, 0xC4, 0x09, 0, 0, 0, 0 };
const byte CLUTCH_ON[8] = { 0x0F, 0x4A, 0xC4, 0x89, 0, 0, 0, 0 };
const char POS_ZERO[8] = { 0x0F, 0x4A, 0xC0, 0x00, 0, 0, 0, 0 };

class BrakeActuator : public BaseModule
{
public:
  BrakeActuator();
  Status setup();
  Status loop();
  Status cleanup();
  void setSpeedError(float error);
  /**
   * 0 ~ 1
   */
  void writeToBrake(float val);

private:
  bool isCANConnected = false;
  float latestSpeedError = 0.0;
  bool hasEnteredCase1 = true;
  float brake_out = 2;
};

#endif  // BRAKE_h