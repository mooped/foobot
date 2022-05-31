#pragma once

#define MOTOR_MAX     1000
#define MOTOR_COAST   0

/// Initialise the PWM/motor control module
void motor_init(void);

/// Set desired motor states -100 - 100
/// 0 - coast
void motor_set_all(int m1, int m2);

