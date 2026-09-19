/*
 * QNX Intelligent Adaptive Cruise Control
 *
 * Actuator Process Header
 *
 * Receives only Safety-approved commands and controls
 * the L298N motor driver.
 */

#ifndef ACC_ACTUATOR_H
#define ACC_ACTUATOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "../common/acc_types.h"


/* ============================================================
 * ACTUATOR CONTEXT
 * ============================================================ */

typedef struct
{
    /*
     * Controls execution of the Actuator thread.
     *
     * 1 = running
     * 0 = shutdown requested
     */
    volatile int running;


    /*
     * Actuator command sequence number.
     */
    uint32_t sequence;


    /*
     * Current motor status.
     */
    motor_status_t motor_status;


    /*
     * Last command received from Safety.
     */
    control_command_t last_command;


    /*
     * Timestamp of the last valid Safety command.
     */
    uint64_t last_command_timestamp_ns;

} actuator_context_t;


/* ============================================================
 * INITIALIZATION
 * ============================================================ */

/*
 * Initialize the Actuator process.
 *
 * Initializes:
 *
 *     - Safety IPC receiver
 *     - L298N motor driver
 *     - PWM/GPIO interface
 *     - Actuator state
 *
 * Returns:
 *
 *     ACC_SUCCESS
 *     ACC_FAILURE
 */
int actuator_init(
    actuator_context_t *context
);


/* ============================================================
 * ACTUATOR THREAD
 * ============================================================ */

/*
 * Main Actuator thread.
 *
 * Responsibilities:
 *
 *     1. Receive Safety-approved command
 *     2. Validate command
 *     3. Check command freshness
 *     4. Apply motor direction
 *     5. Apply PWM duty cycle
 *     6. Stop motor on invalid/stale command
 *     7. Measure actuator response timing
 */
void *actuator_thread(
    void *argument
);


/* ============================================================
 * COMMAND VALIDATION
 * ============================================================ */

/*
 * Validate a Safety-approved actuator command.
 *
 * Checks:
 *
 *     - Motor direction
 *     - PWM duty cycle
 *     - Target speed
 *     - Actuator enable
 *     - ACC state
 *     - Command timestamp
 *
 * Returns:
 *
 *     ACC_SUCCESS
 *     ACC_FAILURE
 */
int actuator_validate_command(
    const control_command_t *command
);


/* ============================================================
 * MOTOR CONTROL
 * ============================================================ */

/*
 * Apply a validated command to the physical motor.
 *
 * The command is passed to the L298N driver through
 * the GPIO/PWM interface.
 */
int actuator_apply_command(
    const control_command_t *command
);


/*
 * Immediately stop the motor.
 *
 * This is the deterministic safe state used for:
 *
 *     - IPC failure
 *     - stale command
 *     - invalid command
 *     - process fault
 *     - shutdown
 */
int actuator_safe_stop(void);


/* ============================================================
 * SHUTDOWN
 * ============================================================ */

/*
 * Stop the Actuator process and release motor/GPIO resources.
 */
int actuator_shutdown(
    actuator_context_t *context
);


#ifdef __cplusplus
}
#endif

#endif /* ACC_ACTUATOR_H */
