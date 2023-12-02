#include <Arduino.h>

#include "pin.h"
#include "angle_sensor.h"

SteeringAngleSensor::SteeringAngleSensor(int pin, int calPin)
{
    this->pin = pin;
    this->calPin = calPin;
    this->name = "SteeringAngleSensor";
}

Status SteeringAngleSensor::setup()
{
    if (this->pin)
    {
        return Status::OK;
    }
    return Status::FAILED;
}

Status SteeringAngleSensor::loop()
{
    if (!this->pin)
    {
        return Status::FAILED;
    }

    float steeringCalSensorVal = analogRead(this->calPin);
    float steering_cal_offset = (512 - steeringCalSensorVal)/STEERING_ANGLE_CAL_POT_FACTOR;
    float sensorValue = analogRead(this->pin) + steering_cal_offset;
    // Serial.print(" Sensor Value: ");
    // Serial.print(sensorValue);
    // Serial.println(); 
    // float angle = -1 * ((sensorValue - 0) / (1023 - 0) * (30 - -30) + -30);

    float angle = -1 * ((sensorValue - 0) / (1023 - 0) * (MAX_STEERING_DEGREE - MIN_STEERING_DEGREE) + MIN_STEERING_DEGREE);
    this->currentAngle = angle;

    // Serial.print(" base angle: ");
   //  Serial.print(angle);

    this->addReading(this->currentAngle);
    this->currentAngularVelocity = this->calcVelocity();

    return Status::OK;
}

void SteeringAngleSensor::addReading(float reading)
{
    reading_buffer.push(reading);
    timestamp_buffer.push(millis());
}

float SteeringAngleSensor::calcVelocity()
{
    if (reading_buffer.size() != STEERING_ANGLE_BUFFER_LEN || timestamp_buffer.size() != STEERING_ANGLE_BUFFER_LEN)
    {
        return 0.0;
    }
    // find the earliest timestamp
    uint32_t min_timestamp_index = -1;
    uint32_t min_timestamp = millis();
    for (uint32_t i = 0; i < timestamp_buffer.size(); i++)
    {
        if (timestamp_buffer[i] < min_timestamp)
        {
            min_timestamp_index = i;
        }
    }
    // find average velocity between each pair
    float total = 0;
    for (size_t i = 0; i < STEERING_ANGLE_BUFFER_LEN - 1; i++)
    {
        size_t curr_index = (min_timestamp_index + i) % STEERING_ANGLE_BUFFER_LEN;
        size_t next_index = (min_timestamp + i + 1) % STEERING_ANGLE_BUFFER_LEN;

        float curr_reading = reading_buffer[curr_index];
        float next_reading = reading_buffer[next_index];

        float velocity = next_reading - curr_reading;
        total += velocity;
    }

    float avg_velocity = total / STEERING_ANGLE_BUFFER_LEN;

    return avg_velocity;
}

Status SteeringAngleSensor::cleanup()
{
    return Status::OK;
}

float SteeringAngleSensor::getSteeringAngle()
{
    return this->currentAngle;
     Serial.print("current angle: ");
     Serial.print(currentAngle);

}
float SteeringAngleSensor::getAngularVelocity()
{
    return this->currentAngularVelocity;
}