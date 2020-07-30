#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void set_throttle_calibration(uint16_t min, uint16_t max);
uint16_t get_throttle_adc();

uint8_t motor_disarm(); //returns armed state
void motor_arm();

inline void utils_step_towards(float *value, float goal, float step) {
    if (*value < goal) {
        if ((*value + step) < goal) {
            *value += step;
        } else {
            *value = goal;
        }
    } else if (*value > goal) {
        if ((*value - step) > goal) {
            *value -= step;
        } else {
            *value = goal;
        }
    }
}

static inline float utils_calc_ratio(float low, float high, float val) {
	return (val - low) / (high - low);
}

static inline float utils_map(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static inline int utils_map_int(int x, int in_min, int in_max, int out_min, int out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#define UTILS_LP_FAST(value, sample, filter_constant)	(value -= (filter_constant) * ((value) - (sample)))

static inline float clip(float val, float min, float max) {
	if(val<min) return min;
	if(val>max) return max;
	return val;
}

#ifndef MAX
#define MAX(a,b) (((a)>(b))?(a):(b))
#endif

#ifdef __cplusplus
}
#endif