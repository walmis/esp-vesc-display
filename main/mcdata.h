/*
 * mcdata.h
 *
 *  Created on: Dec 23, 2019
 *      Author: walmis
 */

#ifndef MAIN_MCDATA_H_
#define MAIN_MCDATA_H_


struct mc_data {
    float v_in;
    float temp_mos;
    float temp_mos_1;
    float temp_mos_2;
    float temp_mos_3;
    float temp_motor;
    float current_motor;
    float current_in;
    float id;
    float iq;
    float rpm;
    float duty_now;
    float amp_hours;
    float amp_hours_charged;
    float watt_hours;
    float watt_hours_charged;
    int tachometer;
    int tachometer_abs;
    float position;
    mc_fault_code fault_code;
    int vesc_id;
    const char* fault_str;
    float vd;
    float vq;
};

extern struct mc_data mc_data;

#endif /* MAIN_MCDATA_H_ */
