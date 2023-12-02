// Copyright 2023 michael. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#include <brake.h>
#include <macros.h>

mcp2515_can CAN(SPI_CS_PIN);
int output_brake_max = 1;
int output_brake_min = 0;
int prev_brake = 0;
unsigned char stmp[8] = {0x0F, 0x0A, 0x00, 0xC4, 0xC9, 0x00, 0x00, 0x00};
unsigned char stmpa[8] = {0x0F, 0x4A, 0x00, 0xC0, 0xC9, 0x00, 0x00, 0x00};

void overwriteBuf(volatile byte *buf, int b0, int b1, int b2, int b3, int b4, int b5, int b6, int b7)
{
    buf[0] = b0;
    buf[1] = b1;
    buf[2] = b2;
    buf[3] = b3;
    buf[4] = b4;
    buf[5] = b5;
    buf[6] = b6;
    buf[7] = b7;
}

/*
 *   inputs a hex string (without prefix 0x)
 *   and return the int value
 */
int strHexToInt(char str[])
{
    // inputs a hex string (without prefix 0x)
    // and return the int value
    return (int)strtol(str, 0, 16);
}

/*
 *   Format Byte 3 as String given clutch, motor flags
 *   and significant byte of position
 */
String posCmdBite3Parser(int ce, int m, String dpos_hi)
{
    return String((int)(ce * pow(2, 7) + m * pow(2, 6) + strHexToInt(const_cast<char *>(dpos_hi.c_str()))), HEX);
}

/*
 *   Move the Actuator to designated position in inches.
 *   The Actuator will execute whatever
 *   the latest command is immediately.
 */
void setActuatorPosition(float inputDist)
{
    // Serial.print(" Final out: ");
    // Serial.print(inputDist);

    unsigned char data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    String bite2;
    String bite3;
    String dpos_lo;
    String dpos_hi;

    // Clipping input range
    inputDist = inputDist < MIN_BRAKE_DIST ? MIN_BRAKE_DIST : inputDist;
    inputDist = inputDist > MAX_BRAKE_DIST ? MAX_BRAKE_DIST : inputDist;

    // Convert input to hex
    int intDist = inputDist * 1000 + 500; // in 0.001” steps
    String hexDist = String(intDist, HEX);
    int hexDistLen = hexDist.length();

    // the least significant byte of position
    dpos_lo = hexDist.substring(hexDistLen - 2);
    bite2 = dpos_lo;
    // The most significant byte of position
    dpos_hi = hexDist.substring(0, hexDistLen - 2);

    // Clutch on, Motor on and move to the desired position
    bite3 = posCmdBite3Parser(1, 1, dpos_hi);
    overwriteBuf(data, 0x0F, 0x4A, strHexToInt(const_cast<char *>(bite2.c_str())),
                 strHexToInt(const_cast<char *>(bite3.c_str())), 0, 0, 0, 0);

    CAN.sendMsgBuf(COMMAND_ID, CAN_EXT_ID, CAN_RTR_BIT, data);
}

BrakeActuator::BrakeActuator()
{
    this->name = "BrakeActuator";
}

Status BrakeActuator::setup()
{
    if (CAN_OK == CAN.begin(CAN_250KBPS))
    { // init can bus : baudrate = 500k
        isCANConnected = true;
        Serial.println("CAN init ok!");
    }
    else
    {
        isCANConnected = false;
        Serial.println("CAN init failed!");
    }
    return Status::OK;
}

Status BrakeActuator::loop()
{
    if (!isCANConnected)
    {
        if (CAN_OK == CAN.begin(CAN_250KBPS))
        { // init can bus : baudrate = 500k
            isCANConnected = true;
            Serial.println("CAN init ok!");
        }
        else
        {
            isCANConnected = false;
        }
    }

    return Status::OK;
}

Status BrakeActuator::cleanup()
{
    return Status::OK;
}

void BrakeActuator::setSpeedError(float error)
{
    this->latestSpeedError = error;
}
void BrakeActuator::writeToBrake(float val)
{
    if (latestSpeedError <= 1)
    {
        brake_out = OPEN_BRAKE_DIST;
    }
    else if (latestSpeedError > 1)
    {
       // Serial.print(" OPEN BRAKE ");
        brake_out = PRIME_DIST;
    }

    float user_request_brake = float(constrain(val, output_brake_min, output_brake_max));
    if (user_request_brake > 0.1)
    {
       //Serial.print(" PRIME DIST ");
        brake_out = PRIME_DIST + (MAX_BRAKE_DIST - PRIME_DIST) * user_request_brake;
    }

    setActuatorPosition(brake_out);
    return;

    // Serial.print(" Verr: ");
    // Serial.print(latestSpeedError);
    // if (latestSpeedError > 2)
    // {
    //     Serial.print(" OPEN BRAKE ");
    //     brake_out = OPEN_BRAKE_DIST;
    // }
    // else if (latestSpeedError < 1)
    // {
    //     Serial.print(" PRIME DIST ");
    //     brake_out = PRIME_DIST;
    // }

    // float user_request_brake = float(constrain(val, output_brake_min, output_brake_max));
    // if (user_request_brake > 0.2)
    // {
    //     Serial.print(" PRIME DIST ");
    //     brake_out = max(PRIME_DIST, user_request_brake * MAX_BRAKE_DIST);
    // }

    // setActuatorPosition(brake_out);
    // return;

    // float scaleBreakOutput = map((brake_out*1000),0,1000,2000,2700);
    // if (brake_out < 0.05)
    // {
    //     scaleBreakOutput = 0;
    // }
    // setActuatorPosition(scaleBreakOutput/1000);
    // setActuatorPosition(2.0); // test if goes to 2.0

    // if (brake_out > 0.2)
    // {
    //     float output = brake_out * MAX_BRAKE_DIST;
    //     setActuatorPosition(output);
    //     return;
    // }

    // float scaleBrakeOutput = map((brake_out*10000),0,10000,20000,25600);
    // if (brake_out==0){
    //     scaleBrakeOutput = 0;
    // } else if (latestSpeedError < MIN_EFFECTIVE_SPEED_FOR_BRAKE)
    // {
    //     scaleBrakeOutput = 20000;
    // }
    // float brake_output = scaleBrakeOutput/10000;
    // setActuatorPosition(brake_output);
}
