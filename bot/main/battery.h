/*
 * Battery level sensor for FooBot
 *
 * Steve Barnett November 2018
 *
 */

#pragma once

#include "driver/adc.h"

// Initialise ADC
void battery_init(adc1_channel_t channel);

// Read battery voltage
float battery_read_voltage(void);

