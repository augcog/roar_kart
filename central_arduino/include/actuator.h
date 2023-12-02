
#include "models.h"
#include "spark_max.h"
#include "base_module.h"
#include "steering_limiter.h"
#include "pwm_voltage_converter.h"
#include "brake.h"
#include <pidcontroller.h>

class ActuationModule : public BaseModule
{

public:
    ActuationModule(SteeringLimiter *limiter,
                    PWMVoltageConverterModule *pwm_to_voltage_converter,
                    SparkMaxModule *spark_max_module,
                    BrakeActuator *brake_module);
    Status setup();
    Status loop();
    Status cleanup();

    void actuate(VehicleState *vehicle_state);

private:
    Actuation *p_ensure_safety(Actuation *act);
    void p_drive(VehicleState *vehicle_state);
    SteeringLimiter *steering_limiter;
    PWMVoltageConverterModule *pwm_to_voltage_converter;
    SparkMaxModule *spark_max_module;
    BrakeActuator *brake_module;
    VehicleState* last_known_vehicle_state;
};