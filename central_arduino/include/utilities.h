#ifndef _UTILITIES_H
#define _UTILITIES_H

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

float clip(float value, float min_val, float max_val);

#endif //_UTILITIES_H