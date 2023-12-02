
#ifndef _SPEED_SENSOR_h
#define _SPEED_SENSOR_h
#include <Arduino.h>
#include "base_module.h"
#include <CircularBuffer.h>
#include "macros.h"

class SpeedSensor : public BaseModule
{
static void pulseCounter();

public:
    SpeedSensor(uint32_t speed_sensor_pin);
    Status setup();
    Status loop();
    Status cleanup();
    float getAvgSpeed();

private:
    uint32_t pin;
    unsigned long prevTime = 0; // Time of the previous pulse
    unsigned long prevDebounceTime = 0;
    float lastSensorReadingMph = 0;


    CircularBuffer<float, SPEED_QUEUE_LENGTH> raw_reading_buffer;
    void addSpeedReading(float speed);
    float getLatestReading();
    float getTotal();

    
};


#endif // _SPEED_SENSOR_h