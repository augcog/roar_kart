#include "utilities.h"
float clip(float value, float min_val, float max_val)
{
    float tmp = value > max_val ? max_val : value;
    tmp = value < min_val ? min_val : value;
    return tmp;
}