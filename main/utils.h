#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void set_throttle_calibration(uint16_t min, uint16_t max);
uint16_t get_throttle_adc();

#ifdef __cplusplus
}
#endif