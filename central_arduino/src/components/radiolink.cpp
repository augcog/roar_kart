// Copyright 2023 michael. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#include "radiolink.h"
#include "macros.h"

static uint32_t throttle_source_pin;
static uint32_t steering_source_pin;
static uint32_t brake_source_pin;
static uint32_t button_source_pin;

static volatile unsigned long throttle_timer_start;
static volatile unsigned long steering_timer_start;
static volatile unsigned long brake_timer_start;
static volatile unsigned long button_timer_start;

static volatile int throttle_last_interrupt_time;
static volatile int steering_last_interrupt_time;
static volatile int brake_last_interrupt_time;
static volatile int button_last_interrupt_time;

static volatile int throttle_pulse_time;
static volatile int steering_pulse_time;
static volatile int knob_pulse_time;
static volatile int button_pulse_time;

RadioLinkModule::RadioLinkModule(uint32_t throttle_pin, uint32_t steering_pin, uint32_t brake_pin, uint32_t button_pin)
{
    throttle_source_pin = throttle_pin;
    throttle_source_pin = throttle_pin;
    throttle_source_pin = throttle_pin;
    steering_source_pin = steering_pin;
    brake_source_pin = brake_pin;
    button_source_pin = button_pin;

    throttle_pulse_time = 1500;
    steering_pulse_time = 1500;
    knob_pulse_time = 1500;
    button_pulse_time = 1500;
    this->name = "RadioLinkModule";
}
Status RadioLinkModule::setup()
{
    throttle_timer_start = 0;
    steering_timer_start = 0;
    brake_timer_start = 0;
    button_timer_start = 0;
    attachInterrupt(throttle_source_pin, calcThrottleSignal, CHANGE);
    attachInterrupt(steering_source_pin, calcSteeringSignal, CHANGE);
    attachInterrupt(brake_source_pin, calcReverseSignal, CHANGE);
    attachInterrupt(button_source_pin, calcButtonSignal, CHANGE);
    return Status::OK;
}
Status RadioLinkModule::loop()
{
    this->p_processButton();
    this->checkForward();
    return Status::OK;
}
Status RadioLinkModule::cleanup()
{
    return Status::OK;
}
float RadioLinkModule::getSteeringDeg()
{
    int flagValue = 0;
    flagValue = steering_pulse_time;
    float steering_value = pulseTimeToFloat(flagValue);
    float converted =
        (steering_value + 1.0) / (1.0 + 1.0) * (MAX_STEERING_DEGREE - MIN_STEERING_DEGREE) + MIN_STEERING_DEGREE;
    return converted;
}

float RadioLinkModule::getTargetSpeed()
{
    int flagValue = 0;
    flagValue = throttle_pulse_time;                    // 1000 ~ 2000
    float throttle_val = pulseTimeToFloat(flagValue);   // -1 ~ 1
    float capped = throttle_val < 0 ? 0 : throttle_val; // 0 ~ 1
    float targetSpeed = capped * RADIO_LINK_MAX_SPEED_GAIN;
    // float converted = (steering_value + 1.0) / (1.0 + 1.0) * (MAX_STEERING_DEGREE - MIN_STEERING_DEGREE) +
    // MIN_STEERING_DEGREE;
    return targetSpeed;
}

float RadioLinkModule::getSteering()
{
    int flagValue = 0;
    flagValue = steering_pulse_time;
    float steering_value = pulseTimeToFloat(flagValue);
    return steering_value;
}

float RadioLinkModule::getThrottle()
{
    int flagValue = 0;
    flagValue = throttle_pulse_time;
    float throttle_value = max(0, pulseTimeToFloat(flagValue));
    return throttle_value;
}

float RadioLinkModule::getBrake()
{
    int flagValue = 0;
    flagValue = throttle_pulse_time;
    float brake_value = min(0, pulseTimeToFloat(flagValue)) * -1;
    return brake_value;
}

void RadioLinkModule::calcThrottleSignal()
{
    // record the interrupt time so that we can tell if the receiver has a signal from the transmitter
    throttle_last_interrupt_time = micros();
    // if the pin has gone HIGH, record the microseconds since the Arduino started up
    if (digitalRead(throttle_source_pin) == HIGH)
    {
        throttle_timer_start = micros();
    }
    // otherwise, the pin has gone LOW
    else
    {
        // only worry about this if the timer has actually started
        if (throttle_timer_start != 0)
        {
            // record the pulse time
            throttle_pulse_time = constrain(((volatile int)micros() - throttle_timer_start), 1000, 2000);
            // restart the timer
            throttle_timer_start = 0;
        }
    }
}

void RadioLinkModule::calcSteeringSignal()
{
    // record the interrupt time so that we can tell if the receiver has a signal from the transmitter
    steering_last_interrupt_time = micros();
    // if the pin has gone HIGH, record the microseconds since the Arduino started up
    if (digitalRead(steering_source_pin) == HIGH)
    {
        steering_timer_start = micros();
    }
    // otherwise, the pin has gone LOW
    else
    {
        // only worry about this if the timer has actually started
        if (steering_timer_start != 0)
        {
            // record the pulse time
            steering_pulse_time = constrain(((volatile int)micros() - steering_timer_start), 1000, 2000);
            // restart the timer
            steering_timer_start = 0;
        }
    }
}

void RadioLinkModule::calcReverseSignal()
{
    // record the interrupt time so that we can tell if the receiver has a signal from the transmitter
    brake_last_interrupt_time = micros();
    // if the pin has gone HIGH, record the microseconds since the Arduino started up
    if (digitalRead(brake_source_pin) == HIGH)
    {
        brake_timer_start = micros();
    }
    // otherwise, the pin has gone LOW
    else
    {
        // only worry about this if the timer has actually started
        if (brake_timer_start != 0)
        {
            // record the pulse time
            knob_pulse_time = constrain(((volatile int)micros() - brake_timer_start), 1000, 2000);
            // restart the timer
            brake_timer_start = 0;
        }
    }
}

void RadioLinkModule::calcButtonSignal()
{
    // record the interrupt time so that we can tell if the receiver has a signal from the transmitter
    button_last_interrupt_time = micros();
    // if the pin has gone HIGH, record the microseconds since the Arduino started up
    if (digitalRead(button_source_pin) == HIGH)
    {
        button_timer_start = micros();
    }
    // otherwise, the pin has gone LOW
    else
    {
        // only worry about this if the timer has actually started
        if (button_timer_start != 0)
        {
            // record the pulse time
            button_pulse_time = ((volatile int)micros() - button_timer_start);
            // restart the timer
            button_timer_start = 0;
        }
    }
}

bool RadioLinkModule::isAutoFromButton()
{
    return this->isAutoMode;
}

void RadioLinkModule::p_processButton()
{
    bool prevButtonPressed = this->isButtonPressed;
    bool prevButtonLifted = this->isButtonLifted;

    // update the new button state
    if (button_pulse_time >= 1600)
    {
        this->isButtonPressed = true;
    }
    else
    {
        this->isButtonPressed = false;
    }

    if (button_pulse_time <= 1400)
    {
        this->isButtonLifted = true;
    }
    else
    {
        this->isButtonLifted = false;
    }

    // process whether the button is pressed
    if (prevButtonPressed && this->isButtonLifted)
    {
        // Serial.println("Shifting mode");
        if (this->isAutoMode)
        {
            this->isAutoMode = false;
        }
        else
        {
            this->isAutoMode = true;
        }
    }
}

float RadioLinkModule::pulseTimeToFloat(uint32_t pulse_time)
{
    float val = constrain(pulse_time, 1000, 2000);
    val = (val - 1000.0) / (2000.0 - 1000.0) * (1 - -1) + -1;
    return val;
}

void RadioLinkModule::checkForward()
{
    if (knob_pulse_time < 1200)
    {
        isForward = false;
    }
    else
    {
        isForward = true;
    }
}
