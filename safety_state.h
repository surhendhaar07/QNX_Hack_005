#ifndef ACC_SAFETY_STATE_H
#define ACC_SAFETY_STATE_H

/*
 * QNX Intelligent Adaptive Cruise Control
 *
 * Safety State Interface
 *
 * Safety state machine:
 *
 *     INIT
 *       |
 *       v
 *     ACTIVE
 *       |
 *       +--------> WARNING
 *       |             |
 *       |             v
 *       +--------> EMERGENCY
 *       |
 *       +--------> FAULT
 *                       |
 *                       v
 *                    STOPPED
 *
 * Safety is the gatekeeper between Controller and Actuator.
 */

#include "../common/acc_types.h"


/* ============================================================
 * SAFETY STATE INITIALIZATION
 * ============================================================ */

/*
 * Initialize a safety status structure.
 *
 * Initial state:
 *
 *     ACC_STATE_INIT
 *
 * Sensor status:
 *
 *     SENSOR_TIMEOUT
 *
 * No actuator operation is permitted during initialization.
 */
void safety_state_init(
    safety_status_t *status
);


/* ============================================================
 * SAFETY STATE UPDATE
 * ============================================================ */

/*
 * Update the safety state using:
 *
 *     - Controller command
 *     - Sensor status
 *     - Controller deadline condition
 *
 * Returns the resulting ACC safety state.
 */
acc_state_t safety_state_update(
    safety_status_t *status,
    const control_command_t *command,
    sensor_status_t sensor_status,
    int controller_deadline_missed
);


/* ============================================================
 * SENSOR FAULT
 * ============================================================ */

/*
 * Apply a sensor fault to the safety state.
 *
 * The resulting state is:
 *
 *     ACC_STATE_FAULT
 *
 * and actuator operation is expected to be stopped.
 */
void safety_state_sensor_fault(
    safety_status_t *status,
    sensor_status_t sensor_status
);


/* ============================================================
 * SENSOR STALE
 * ============================================================ */

/*
 * Apply stale sensor data to the safety state.
 *
 * The resulting state is:
 *
 *     ACC_STATE_FAULT
 */
void safety_state_sensor_stale(
    safety_status_t *status
);


/* ============================================================
 * CONTROLLER DEADLINE FAULT
 * ============================================================ */

/*
 * Apply a controller deadline miss.
 *
 * A missed controller deadline results in a safety fault
 * and disables normal actuator operation.
 */
void safety_state_controller_deadline_fault(
    safety_status_t *status
);


/* ============================================================
 * EMERGENCY CONDITION
 * ============================================================ */

/*
 * Apply an emergency-distance condition.
 *
 * The resulting state is:
 *
 *     ACC_STATE_EMERGENCY
 *
 * The actuator must not receive a propulsion command.
 */
void safety_state_emergency(
    safety_status_t *status,
    double distance_cm
);


/* ============================================================
 * SAFE STOP
 * ============================================================ */

/*
 * Force the safety state to STOPPED.
 *
 * This is used when the system must enter a deterministic
 * safe state.
 */
void safety_state_force_stop(
    safety_status_t *status
);


/* ============================================================
 * STATE QUERIES
 * ============================================================ */

/*
 * Determine whether the current state permits actuator
 * operation.
 *
 * Returns:
 *
 *     1 = actuator operation permitted
 *     0 = actuator must remain stopped
 */
int safety_state_actuator_allowed(
    const safety_status_t *status
);


/*
 * Determine whether the current state represents an
 * emergency condition.
 *
 * Returns:
 *
 *     1 = emergency
 *     0 = not emergency
 */
int safety_state_is_emergency(
    const safety_status_t *status
);


/*
 * Determine whether the current state represents a fault.
 *
 * Returns:
 *
 *     1 = fault
 *     0 = no fault
 */
int safety_state_is_fault(
    const safety_status_t *status
);


#endif /* ACC_SAFETY_STATE_H */
