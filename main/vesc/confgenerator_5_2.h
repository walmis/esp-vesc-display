// This file is autogenerated by VESC Tool

#ifndef CONFGENERATOR_H_
#define CONFGENERATOR_H_

#include "datatypes_5_2.h"
#include <stdint.h>
#include <stdbool.h>

//vesc 5.2
// Constants
#define MCCONF_SIGNATURE		3698540221
#define APPCONF_SIGNATURE		2460147246

// Functions
int32_t confgenerator_serialize_mcconf(uint8_t *buffer, const mc_configuration *conf);
int32_t confgenerator_serialize_appconf(uint8_t *buffer, const app_configuration *conf);

bool confgenerator_deserialize_mcconf(const uint8_t *buffer, mc_configuration *conf);
bool confgenerator_deserialize_appconf(const uint8_t *buffer, app_configuration *conf);

void confgenerator_set_defaults_mcconf(mc_configuration *conf);
void confgenerator_set_defaults_appconf(app_configuration *conf);

// CONFGENERATOR_H_
#endif