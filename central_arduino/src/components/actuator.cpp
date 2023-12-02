#include "actuator.h"
#include "macros.h"

ActuationModule::ActuationModule(SteeringLimiter *limiter,
                                 PWMVoltageConverterModule *pwm_to_voltage_converter,
                                 SparkMaxModule *spark_max_module,
                                 BrakeActuator *brake_module)
{
    this->steering_limiter = limiter;
    this->pwm_to_voltage_converter = pwm_to_voltage_converter;
    this->spark_max_module = spark_max_module;
    this->brake_module = brake_module;
    this->name = "ActuationModule";
}

Status ActuationModule::setup()
{

    return Status::OK;
}

Status ActuationModule::loop()
{
    return Status::OK;
}

Status ActuationModule::cleanup()
{
    return Status::OK;
}

Actuation *ActuationModule::p_ensure_safety(Actuation *act)
{
    Actuation *output = new Actuation();
    output->brake = act->brake;
    output->reverse = act->reverse;
    output->throttle = act->throttle < 0 ? 0 : act->throttle;
    output->steering = act->steering;

    if (this->steering_limiter->isLeftLimiterON())
    {
        Serial.println("LEFT ON");
        output->steering = act->steering > 0 ? 0 : act->steering;
    }

    if (this->steering_limiter->isRightLimiterON())
    {
        Serial.println("RIGHT ON");
        output->steering = act->steering < 0 ? 0 : act->steering;
    }

    // if (last_known_vehicle_state->current_speed > 1.0) {
    //     act->reverse = false;  // Ensure reverse is false when speed is greater than 1
    // }
    // else{
    //     act->reverse = true;
    // }

    return output;
}

void ActuationModule::p_drive(VehicleState *vehicle_state)
{
    Actuation *act = this->p_ensure_safety(vehicle_state->current_actuation);
    // Serial.print(" steering: ");
    // Serial.print(act->steering);
    // Serial.print(" vehicle_state->current_angle: ");
    // Serial.println(vehicle_state->current_angle);
    // software steering limit
    if (vehicle_state->current_angle >= SOFTWARE_MAX_STEERING_ANGLE_LIMIT)
    {

        // Serial.println("MAX STEERING");

        act->steering = act->steering > 0 ? 0 : act->steering;
    }

    if (vehicle_state->current_angle <= SOFTWARE_MIN_STEERING_ANGLE_LIMIT)
    {
        // Serial.println("MIN STEERING");
        act->steering = act->steering < 0 ? 0 : act->steering;
    }

    spark_max_module->writeToSteering(act->steering);
    pwm_to_voltage_converter->writeToThrottle(act->throttle);
    brake_module->writeToBrake(act->brake);
    free(act); // MUST free allocated memory
}

void ActuationModule::actuate(VehicleState *vehicle_state)
{
    last_known_vehicle_state = vehicle_state;
    p_drive(vehicle_state);
}
