#ifndef VEHICLE_STRATEGY_H
#define VEHICLE_STRATEGY_H

#include <stdint.h>
#include "shared_types.h"

float pedal_sensing(uint16_t adc1, uint16_t adc2);
float calculate_torque(float pedal_percent, float speed, BatteryInformations battery);

#endif