#include "throttle_pid.h"
#include <Arduino.h>
#include "utilities.h"
#include "pin.h"
ThrottlePIDController::ThrottlePIDController(float kp, float kd, float ki, float min_output, float max_output)
    : kp(kp), kd(kd), ki(ki)
{
    this->kp = kp;
    this->kd = kd;
    this->ki = ki;
    this->min_output = min_output;
    this->max_output = max_output;
}

Status ThrottlePIDController::setup()
{
    return Status::OK;
}

Status ThrottlePIDController::loop()
{
    return Status::OK;
}

Status ThrottlePIDController::cleanup()
{
    return Status::OK;
}

float ThrottlePIDController::compute(float value, float target)
{
    analogSpeedLimit = analogRead(ANALOG_SPEED_LIMIT_INPUT_PIN); // Read the analog speed limit input
    analog_speed_buffer.push(analogSpeedLimit);

    float analog_limit = calcAvgAnalogLimit(); // TODO: take this out into a seperate module
    knobMaxSpeed = analog_limit / 10;
    if (target <= 0.00001)
    {
        // if requested speed is near 0, go to 0 immediately
        error_buffer.clear();
        prev_output = 0.0;
        total_err = 0;
        return 0.0;
    }

    target = (target > knobMaxSpeed) ? knobMaxSpeed : target; // limit target speed by max speed

    unsigned long cM = millis();

    float final_output = 0.0;
    if (cM - pM > interval)
    {
        float error = target - value;
        float dt = cM - pM;

         //Serial.print(" Max: ");
        // Serial.print(knobMaxSpeed);
        //Serial.print(" target: ");
        // Serial.print(target);
        // Serial.print(" Speed: ");
         //Serial.print(value);

        float pid_output = 0.0;
        if (dt == 0)
        {
            pid_output = 0; // safety, should never enter
         
        }
        else
        {
           // error_buffer.push(error);
          //  total_err = getTotalError();

           if ((error*kp<1)&&(error*kp>-1)) total_err = total_err + (error/300);
           else total_err = 0; 
           total_err = MAX(0, MIN(12, total_err));



            pid_output = error * this->kp + (error - prev_err) / dt * this->kd + this->total_err * this->ki;
            // Serial.print(" p:");
            // Serial.print(error * this->kp);
            // Serial.print(" d:");
            // Serial.print((error - prev_err) / dt * this->kd);
            // Serial.print(" i:");
            // Serial.print(total_err* this->ki);

             //Serial.print(" total_err ");
             //Serial.print(total_err);
             //Serial.print(" pid_output: ");
             //Serial.print(pid_output);
            prev_err = error;
        }
        //pM = cM; // update when PID is attempted

        final_output = MAX(this->min_output, MIN(this->max_output, pid_output));
        // Serial.print(" final: ");
        // Serial.print(final_output);
        // Serial.println();
    }
    else
    {
        final_output = prev_output;
    }
    prev_output = final_output;

    return final_output;
}

float ThrottlePIDController::getTotalError()
{
    float total = 0;
    for (size_t i = 0; i < error_buffer.size(); i++)
    {
        total += error_buffer[i];
    }
    return total;
}

float ThrottlePIDController::calcAvgAnalogLimit()
{
    float total = 0;
    for (size_t i = 0; i < analog_speed_buffer.size(); i++)
    {
        total += analog_speed_buffer[i];
    }
    return total / analog_speed_buffer.size();
}
