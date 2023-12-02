#ifndef THROTTLE_PID_h
#define THROTTLE_PID_h
#include "base_module.h"
#include "macros.h"

#include <CircularBuffer.h>
class ThrottlePIDController : public BaseModule
{
public:
    ThrottlePIDController(float kp,
                          float kd,
                          float ki,
                          float min_output,
                          float max_output);
    Status setup();
    Status loop();
    Status cleanup();

    float compute(float value, float target);
    float kp = 1;
    float kd = 0;
    float ki = 0;

private:
    float getTotalError();
    float calcAvgAnalogLimit();
    float min_output = -1;
    float max_output = 1;

    float curr_err = 0;
    float total_err = 0;
    float prev_err = 0;
    float time_last = 0;
    unsigned long pM = 0UL;
    unsigned long interval = 10UL;
    float prev_output = 0.0;

    float analogSpeedLimit;
    float knobMaxSpeed;

    CircularBuffer<float, THROTTLE_INTEGRAL_LENGTH> error_buffer;
    CircularBuffer<float, 10> analog_speed_buffer;
};

#endif // THROTTLE_PID_h