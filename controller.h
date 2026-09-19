#ifndef ACC_CONTROLLER_H
#define ACC_CONTROLLER_H

/*
 * QNX Intelligent Adaptive Cruise Control
 *
 * Controller Process Interface
 *
 * Process:
 *     acc_controller
 *
 * Responsibilities:
 *     - Receive Tracking data
 *     - Evaluate target distance
 *     - Calculate ACC control command
 *     - Detect invalid/stale tracking data
 *     - Measure controller execution time
 *     - Monitor controller deadline
 *     - Send control command to Safety
 *
 * Safety architecture:
 *
 *     Tracking -> Controller -> Safety -> Actuator
 *
 * The Controller never directly commands the actuator.
 */

#include <stdint.h>
#include <pthread.h>

#include "../common/acc_types.h"


/* ============================================================
 * CONTROLLER CONTEXT
 * ============================================================ */

typedef struct
{
    /*
     * Worker thread execution state.
     *
     * 1 = running
     * 0 = stop requested
     */
    volatile int running;

    /*
     * Controller command sequence number.
     */
    uint32_t sequence;

    /*
     * Most recent target distance.
     */
    double distance_cm;

    /*
     * Most recent relative target speed.
     */
    double relative_speed_cm_s;

    /*
     * Current controller state.
     */
    acc_state_t state;

    /*
     * Last received tracking timestamp.
     */
    uint64_t last_tracking_timestamp_ns;

    /*
     * Number of invalid/stale tracking inputs.
     */
    uint32_t fault_count;

} controller_context_t;


/* ============================================================
 * INITIALIZATION
 * ============================================================ */

/*
 * Initialize the Controller process.
 *
 * Opens the Tracking -> Controller IPC queue and the
 * Controller -> Safety IPC queue.
 *
 * Returns:
 *
 *     ACC_SUCCESS
 *     ACC_FAILURE
 */
int controller_init(
    controller_context_t *context
);


/* ============================================================
 * CONTROLLER THREAD
 * ============================================================ */

/*
 * Main Controller worker thread.
 *
 * Processing sequence:
 *
 *     1. Receive Tracking data.
 *     2. Measure IPC latency.
 *     3. Validate tracking information.
 *     4. Determine ACC state.
 *     5. Calculate motor command.
 *     6. Measure response/execution time.
 *     7. Check controller deadline.
 *     8. Send command to Safety.
 */
void *controller_thread(
    void *argument
);


/* ============================================================
 * CONTROL CALCULATION
 * ============================================================ */

/*
 * Calculate an ACC command from Tracking data.
 *
 * The command contains:
 *
 *     - motor direction
 *     - PWM duty cycle
 *     - target speed
 *     - ACC state
 *     - actuator enable
 *
 * Returns:
 *
 *     ACC_SUCCESS
 *     ACC_FAILURE
 */
int controller_calculate_command(
    controller_context_t *context,
    const tracking_data_t *tracking,
    control_command_t *command
);


/* ============================================================
 * TRACKING DATA VALIDATION
 * ============================================================ */

/*
 * Validate Tracking input before control calculation.
 *
 * Returns:
 *
 *     ACC_SUCCESS = valid
 *     ACC_FAILURE = invalid
 */
int controller_validate_tracking(
    const tracking_data_t *tracking
);


/* ============================================================
 * STATE DETERMINATION
 * ============================================================ */

/*
 * Determine the ACC state from target distance and sensor
 * condition.
 */
acc_state_t controller_determine_state(
    const tracking_data_t *tracking
);


/* ============================================================
 * DUTY-CYCLE CALCULATION
 * ============================================================ */

/*
 * Calculate requested motor PWM duty cycle.
 *
 * Returned range:
 *
 *     0.0 - 100.0 %
 */
double controller_calculate_duty(
    const tracking_data_t *tracking
);


/* ============================================================
 * COMMAND VALIDATION
 * ============================================================ */

/*
 * Validate a generated control command before sending it
 * to Safety.
 *
 * Returns:
 *
 *     ACC_SUCCESS = valid
 *     ACC_FAILURE = invalid
 */
int controller_validate_command(
    const control_command_t *command
);


/* ============================================================
 * SHUTDOWN
 * ============================================================ */

/*
 * Stop Controller and close its IPC resources.
 *
 * Returns:
 *
 *     ACC_SUCCESS
 *     ACC_FAILURE
 */
int controller_shutdown(
    controller_context_t *context
);


#endif /* ACC_CONTROLLER_H */
