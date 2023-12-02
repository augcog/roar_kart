// Copyright 2023 michael. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#include <Arduino.h>

#include "pin.h"
#include "pwm_voltage_converter.h"

PWMVoltageConverterModule::PWMVoltageConverterModule(int pin)
{
    this->pin = pin;
    this->name = "PWMVoltageConverterModule";
}
Status PWMVoltageConverterModule::setup()
{
    pinMode(this->pin, OUTPUT);
    digitalWrite(this->pin, HIGH);
    delay(1000);

    return Status::OK;
}
Status PWMVoltageConverterModule::loop()
{
    return Status::OK;
}
Status PWMVoltageConverterModule::cleanup()
{
    this->writeToThrottle(0.0);
    return Status::OK;
}

void PWMVoltageConverterModule::actuate(float throttle)
{
    throttle = constrain(throttle, 0, 1);
    int output = (throttle - 0) / (1.0 - 0.0) * (225.0 - 0.0) + 0.0;
    analogWrite(this->pin, output);
}

/**
 * @brief  write to throttle
 * @note Instead of executing the throttle directly, it will execute a smoothing function that gradually goes toward the desired throttle
 * @retval None
 */
void PWMVoltageConverterModule::writeToThrottle(float throttle)
{
    smoothWriteThrottle(throttle);
}

void PWMVoltageConverterModule::smoothWriteThrottle(float throttle)
{
    float prevTotal = getPrevTotal();
    float throttleTotal = prevTotal * PREV_THROTTLE_WEIGHT + throttle * CURR_THROTTLE_WEIGHT;
    float smoothedThrottle = throttleTotal / (buffer.size() + 1);
    actuate(smoothedThrottle);
    buffer.push(throttle);
}

float PWMVoltageConverterModule::getPrevTotal()
{
    float total = 0;
    for (size_t i = 0; i < buffer.size(); i++)
    {
        total += buffer[i];
    }
    return total;
}

float PWMVoltageConverterModule::getPrevAvg()
{
    return getPrevTotal() / buffer.size();
}
