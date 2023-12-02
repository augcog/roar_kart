#include "macros.h"

// steering
const float MIN_STEERING_DEGREE = -30.0;
const float MAX_STEERING_DEGREE = 30.0;
const float SOFTWARE_MIN_STEERING_ANGLE_LIMIT = -27.0;
const float SOFTWARE_MAX_STEERING_ANGLE_LIMIT = 27.0;

// radio link
const float RADIO_LINK_MAX_SPEED_GAIN = 10;

// speed sensor
const float WHEEL_CIRCUMFERENCE = .0232962963; // Circumference of the wheel in inches 5.75 for bolts/hall
const float INCHES_PER_MILE = 63360.0;         // Number of inches in a mile
const unsigned long DEBOUNCE_DELAY = 10;       // Debounce delay in milliseconds
const float PREVIOUS_READING_WEIGHT = 1.0;     // TODO: tune this weight
const float NEW_READING_WEIGHT = 1.0;          // TODO: tune this weight

// throttle
const float PREV_THROTTLE_WEIGHT = 1.0; // TODO: tune this weight
const float CURR_THROTTLE_WEIGHT = 1.0; // TODO: tune this weight
const float MAX_SPEED = 10;             // TODO: tune this

// brake
#define CAN_EXT_ID 1
#define CAN_RTR_BIT 8
const float MIN_BRAKE_DIST = 0.0;  // Minimum distance for actuator
const float MAX_BRAKE_DIST = 3.0;  // Maximum safe distance that the actuator can move (inches)
const float PRIME_DIST = 1.8;      // brake on verge of actuating?
const float OPEN_BRAKE_DIST = 0.0; // Minimum distance for actuator
