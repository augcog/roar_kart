#include "speed_sensor.h"

static volatile unsigned long pulseCount = 0; // Number of pulses from the Hall effect sensor

SpeedSensor::SpeedSensor(uint32_t speed_sensor_pin)
{
    this->pin = speed_sensor_pin;
    this->name = "SpeedSensor";
}
Status SpeedSensor::setup()
{
    attachInterrupt(this->pin, pulseCounter, RISING);

    return Status::OK;
}

Status SpeedSensor::loop()
{
    // get raw data if possible
    float latest_sensor_reading = getLatestReading();

    // use weighted avg for all previous + current
    float total = getTotal() * PREVIOUS_READING_WEIGHT + latest_sensor_reading * NEW_READING_WEIGHT;
    float smoothed = total / (raw_reading_buffer.size() + 1);
    // add to the speed reading queue
    addSpeedReading(smoothed);
    return Status::OK;
}

float SpeedSensor::getLatestReading()
{
    unsigned long currentTime = millis(); // Current time in milliseconds
    unsigned long elapsedDebounceTime = currentTime - prevDebounceTime;

    // Read every DEBOUNCE_DELAY sec to allow sufficient time
    if (elapsedDebounceTime < DEBOUNCE_DELAY)
    {
        return lastSensorReadingMph;
    }

    prevDebounceTime = currentTime;

    // if reached debounce time
    // calculate how many times did the pulse came in within the elapsed time
    unsigned long elapsedTime = currentTime - prevTime;
    float distanceInches = pulseCount * WHEEL_CIRCUMFERENCE ;  // Calculate distance in inches
    float distanceMiles = distanceInches / INCHES_PER_MILE;           // Convert distance to miles
    lastSensorReadingMph = distanceMiles / (elapsedTime / 3600000.0); // Calculate speed in mph

    pulseCount = 0;
    prevTime = currentTime;
   // Serial.print(" lastSensorReadingMph: ");
  //  Serial.print(lastSensorReadingMph);    
    
    return lastSensorReadingMph;

}

Status SpeedSensor::cleanup()
{
    return Status::OK;
}

void SpeedSensor::pulseCounter()
{
    pulseCount++;
}
float SpeedSensor::getTotal()
{
    float total = 0;
    for (size_t i = 0; i < raw_reading_buffer.size(); i++)
    {
        total += raw_reading_buffer[i];
    }
    return total;
}
float SpeedSensor::getAvgSpeed()
{
    float avg = getTotal() / raw_reading_buffer.size();
    if (std::isnan(avg))
    {
        return 0.0;
    }

    return avg;
}

void SpeedSensor::addSpeedReading(float speed)
{
    raw_reading_buffer.push(speed);
}
