#ifndef PWM_H
#define PWM_H


/*
 * Initialize PWM subsystem.
 */
int pwm_init(void);


/*
 * Shutdown PWM subsystem.
 */
void pwm_shutdown(void);


/*
 * Set PWM duty cycle.
 *
 * duty_percent: 0.0 to 100.0
 */
int pwm_set_duty(double duty_percent);


/*
 * Stop PWM output.
 */
int pwm_stop(void);

#endif
