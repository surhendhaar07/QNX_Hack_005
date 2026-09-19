/*
 * QNX Intelligent Adaptive Cruise Control
 *
 * Safety Process Header
 *
 * The Safety process is the final safety gate between the
 * Controller process and the Actuator process.
 */

#ifndef ACC_SAFETY_H
#define ACC_SAFETY_H
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "../common/acc_types.h"


/* ============================================================
 * SAFETY CONTEXT
 * ============================================================ */

typedef struct
{
    /*
     * Controls execution of the Safety thread.
     *
     * 1 = running
     * 0 = shutdown requested
     */
    volatile int running;


    /*
     * Safety message sequence number.
     */
    uint32_t sequence;


    /*
     * Current Safety state.
     *
     * Contains:
     *     - ACC state
     *     - sensor status
     *     - distance
     *     - deadline status
     *     - fault status
     *     - emergency stop status
     *     - timing information
     */
    safety_status_t status;

} safety_context_t;


/* ============================================================
 * INITIALIZATION
 * ============================================================ */

/*
 * Initialize the Safety process.
 *
 * Opens:
 *
 *     Controller -> Safety IPC queue
 *     Safety -> Actuator IPC queue
 *     Safety -> Supervisor IPC queue
 *
 * Initializes the Safety state and timing information.
 *
 * Returns:
 *     ACC_SUCCESS on success
 *     ACC_FAILURE on failure
 */
int safety_init(
    safety_context_t *context
);


/* ============================================================
 * SAFETY THREAD
 * ============================================================ */

/*
 * Main Safety thread.
 *
 * Responsibilities:
 *
 *     1. Receive Controller command
 *     2. Validate IPC message
 *     3. Check command freshness
 *     4. Check Controller deadline
 *     5. Apply Safety state machine
 *     6. Generate safe Actuator command
 *     7. Send command to Actuator
 *     8. Send Safety status to Supervisor
 *     9. Measure timing and IPC latency
 */
void *safety_thread(
    void *argument
);


/* ============================================================
 * COMMAND VALIDATION
 * ============================================================ */

/*
 * Validate a Controller command before allowing it
 * to reach the Actuator process.
 *
 * Checks:
 *
 *     - PWM duty range
 *     - target speed
 *     - timestamp
 *     - motor direction
 *     - actuator enable state
 *     - emergency/fault consistency
 *
 * Returns:
 *     ACC_SUCCESS if valid
 *     ACC_FAILURE if invalid
 */
int safety_validate_command(
    const control_command_t *command
);


/* ============================================================
 * SHUTDOWN
 * ============================================================ */

/*
 * Stop the Safety thread and close its IPC queues.
 */
int safety_shutdown(
    safety_context_t *context
);


#ifdef __cplusplus
}
#endif

#endif /* ACC_SAFETY_H */
