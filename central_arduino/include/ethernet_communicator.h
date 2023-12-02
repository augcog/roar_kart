// Copyright 2023 michael. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#include "models.h"
#include "base_module.h"
#include <Dhcp.h>
#include <SPI.h>
#include <Ethernet2.h>
#include <EthernetUdp2.h> // UDP library from: bjoern@cs.stanford.edu 12/30/2008
#include <ArduinoJson.h>

class EthernetCommunicator : public BaseModule
{

public:
  struct ActuationModelFromEthernet
  {
    float targetSpeed = 0.0f;
    float steeringAngle = 0.0f;
    float brake = 0.0f;
    bool reverse = false;
  };

  EthernetCommunicator();
  Status setup();
  Status loop();
  Status cleanup();

  ActuationModelFromEthernet getAction();
  void setVehicleState(VehicleState vehicle_state);

private:
  void writeStateToUDP();
  ActuationModelFromEthernet readUDP();
  ActuationModelFromEthernet parseJSONData(DeserializationError &error);

  VehicleState latest_vehicle_state;
  ActuationModelFromEthernet actuation_received;

  void printActuationModel(const ActuationModelFromEthernet &act)
  {
    Serial.print("Target Speed: ");
    Serial.print(act.targetSpeed);
    Serial.print(", Steering Angle: ");
    Serial.print(act.steeringAngle);
    Serial.print(", Brake: ");
    Serial.print(act.brake);
    Serial.print(", Reverse: ");
    Serial.print(act.reverse);
  }
  // char receivedChars[32]; // make sure this is the same as numChars
  // boolean newData = false;
};
