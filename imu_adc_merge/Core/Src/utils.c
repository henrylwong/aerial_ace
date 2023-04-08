#include "utils.h"

float max(float a, float b) {
    if (a >= b) {
        return a;
    }
    return b;
}

float clamp(float val, float min, float max) {
    if (val < min) {
        return min;
    }
    if (val > max) {
        return max;
    }
    return val;
}

float lerp(float a, float b, float t)
{
    return a + (b - a) * t;
}

float map(float value, float in_min, float in_max, float out_min, float out_max) {
    if (value >= in_max) {
        return out_max;
    }
    if (value <= in_min) {
        return out_min;
    }
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float convert_period_to_freq(float period) {
    return 1 / period;
}

float convert_freq_to_period(float freq) {
    return 1 / freq;
}