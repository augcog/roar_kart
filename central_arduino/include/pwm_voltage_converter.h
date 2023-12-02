// Copyright 2023 michael. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.
#ifndef _PWM_VOLTAGE_CONVERTER_MODULE_h
#define _PWM_VOLTAGE_CONVERTER_MODULE_h
#include <Arduino.h>
#include <pin.h>
#include <CircularBuffer.h>
#include "base_module.h"
#include "macros.h"

class PWMVoltageConverterModule : public BaseModule
{
public:
    PWMVoltageConverterModule(int pin);
    Status setup();
    Status loop();
    Status cleanup();

    /**
     * @brief  write to throttle
     * @note
     * @retval None
     */
    void writeToThrottle(float throttle);

private:
    void actuate(float throttle);
    void smoothWriteThrottle(float throttle);
    float getPrevAvg();
    float getPrevTotal();

    int pin;

    CircularBuffer<float, THROTTLE_REQUEST_BUFFER_LENGTH> buffer;
};
#endif // _PWM_VOLTAGE_CONVERTER_MODULE_h
