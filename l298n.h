#ifndef L298N_H
#define L298N_H

#include "../common/acc_types.h"

/*
 * L298N interface.
 *
 * This layer isolates the motor-driver hardware
 * from the Actuator process.
 */


/* Initialize L298N interface */
int l298n_init(void);

/* Shutdown L298N interface */
void l298n_shutdown(void);

/* Set motor direction */
int l298n_set_direction(motor_direction_t direction);

/* Set PWM duty cycle */
int l298n_set_pwm(double duty_percent);

/* Stop motor immediately */
int l298n_stop(void);

#endif
