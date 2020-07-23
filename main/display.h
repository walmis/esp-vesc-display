#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void display_set_mot_current_minmax(float min, float max);
void display_set_mot_current(float current);
void display_set_curr_power(float power);
void display_set_trip(float dist);
void display_set_odo(float dist);
void display_set_duty(float duty);
void display_set_mos_temp(float temp);
void display_set_bat_info(float volts, float amps);
void display_set_energy(float ah_in, float ah_out);
void display_set_speed(float speed);
void display_set_bat_limits(float cut_start, float cut_end);
void display_set_avgspeed(float avgspeed);
void display_set_triptime(int trptime_ms);
void display_set_fet_temp(float temp);
void display_set_connected(bool connected);
void display_show_message(const char* message);
void display_set_power_level(int8_t power_level);

void display_run();
void display_setup();
#ifdef __cplusplus
}
#endif
