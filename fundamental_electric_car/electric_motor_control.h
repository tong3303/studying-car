#ifndef ELECTIRC_MOTOR_CONTROL_H
#define ELECTIRC_MOTOR_CONTROL_H

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>

#include "shared_types.h"

void mtpa_current_mapping(float torque_cmd, float* id_ref, float* iq_ref);
void current_pi_control(float id_ref, float iq_ref, float id_actual, float iq_actual, float* vd_cmd, float* vq_cmd);
PWMOutputs inverse_dq_and_pwm(float vd, float vq, float theta);

#endif