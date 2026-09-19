/*
 * QNX Intelligent Adaptive Cruise Control
 *
 * Safety State Machine
 *
 * States:
 *
 *     INIT
 *       |
 *       v
 *     ACTIVE
 *       |
 *       +----> WARNING
 *       |
 *       +----> EMERGENCY
 *       |
 *       +----> FAULT
 *                 |
 *                 v
 *              STOPPED
 *
 * Safety is the gatekeeper between Controller and Actuator.
 *
 * Any invalid sensor condition, stale data condition, or
 * Controller deadline violation results in a safe state.
 */

#include "safety_state.h"

#include "../common/acc_config.h"
#include "../common/timing.h"

#include <string.h>


/* ============================================================
 * INTERNAL SAFE-STOP HELPER
 * ============================================================ */

static void safety_state_clear_actuator_request(
    safety_status_t *status
)
{
    if (status == NULL)
    {
        return;
    }

    /*
     * A safety state by itself does not contain a motor command.
     *
     * The safety_status_t fields indicate that actuator
     * operation is not permitted.
     */
    status->emergency_stop = 1;
}


/* ============================================================
 * INITIALIZE SAFETY STATE
 * ============================================================ */

void safety_state_init(
    safety_status_t *status
)
{
    if (status == NULL)
    {
        return;
    }


    memset(
        status,
        0,
        sizeof(*status)
    );


    /*
     * System begins in INIT.
     */
    status->state =
        ACC_STATE_INIT;


    /*
     * No valid sensor measurement is available during
     * initialization.
     */
    status->sensor_status =
        SENSOR_TIMEOUT;


    status->distance_cm =
        0.0;


    status->controller_deadline_missed =
        0;


    status->fault_active =
        0;


    status->emergency_stop =
        1;


    status->timestamp_ns =
        timing_now_ns();
}


/* ============================================================
 * SENSOR FAULT
 * ============================================================ */

void safety_state_sensor_fault(
    safety_status_t *status,
    sensor_status_t sensor_status
)
{
    if (status == NULL)
    {
        return;
    }


    status->state =
        ACC_STATE_FAULT;


    status->sensor_status =
        sensor_status;


    status->fault_active =
        1;


    status->emergency_stop =
        1;


    status->controller_deadline_missed =
        0;


    safety_state_clear_actuator_request(
        status
    );


    status->timestamp_ns =
        timing_now_ns();
}


/* ============================================================
 * SENSOR STALE
 * ============================================================ */

void safety_state_sensor_stale(
    safety_status_t *status
)
{
    if (status == NULL)
    {
        return;
    }


    status->state =
        ACC_STATE_FAULT;


    status->sensor_status =
        SENSOR_STALE;


    status->fault_active =
        1;


    status->emergency_stop =
        1;


    safety_state_clear_actuator_request(
        status
    );


    status->timestamp_ns =
        timing_now_ns();
}


/* ============================================================
 * CONTROLLER DEADLINE FAULT
 * ============================================================ */

void safety_state_controller_deadline_fault(
    safety_status_t *status
)
{
    if (status == NULL)
    {
        return;
    }


    status->state =
        ACC_STATE_FAULT;


    status->controller_deadline_missed =
        1;


    status->fault_active =
        1;


    status->emergency_stop =
        1;


    safety_state_clear_actuator_request(
        status
    );


    status->timestamp_ns =
        timing_now_ns();
}


/* ============================================================
 * EMERGENCY CONDITION
 * ============================================================ */

void safety_state_emergency(
    safety_status_t *status,
    double distance_cm
)
{
    if (status == NULL)
    {
        return;
    }


    status->state =
        ACC_STATE_EMERGENCY;


    status->sensor_status =
        SENSOR_OK;


    status->distance_cm =
        distance_cm;


    status->controller_deadline_missed =
        0;

    status->fault_active = 0;
    status->emergency_stop = 0;


    status->fault_active =
        0;


    status->emergency_stop =
        1;


    safety_state_clear_actuator_request(
        status
    );


    status->timestamp_ns =
        timing_now_ns();
}


/* ============================================================
 * FORCE SAFE STOP
 * ============================================================ */

void safety_state_force_stop(
    safety_status_t *status
)
{
    if (status == NULL)
    {
        return;
    }


    status->state =
        ACC_STATE_STOPPED;


    status->fault_active =
        1;


    status->emergency_stop =
        1;


    safety_state_clear_actuator_request(
        status
    );


    status->timestamp_ns =
        timing_now_ns();
}


/* ============================================================
 * SAFETY STATE UPDATE
 * ============================================================ */

acc_state_t safety_state_update(
    safety_status_t *status,
    const control_command_t *command,
    sensor_status_t sensor_status,
    int controller_deadline_missed
)
{
    uint64_t now_ns;
    uint64_t command_age_ns;


    if (status == NULL)
    {
        return ACC_STATE_FAULT;
    }


    /*
     * --------------------------------------------------------
     * Sensor fault has highest priority.
     * --------------------------------------------------------
     */

    if (sensor_status == SENSOR_TIMEOUT ||
        sensor_status == SENSOR_INVALID)
    {
        safety_state_sensor_fault(
            status,
            sensor_status
        );

        return status->state;
    }


    /*
     * --------------------------------------------------------
     * Stale sensor data.
     * --------------------------------------------------------
     */

    if (sensor_status == SENSOR_STALE)
    {
        safety_state_sensor_stale(
            status
        );

        return status->state;
    }


    /*
     * --------------------------------------------------------
     * Controller deadline.
     * --------------------------------------------------------
     */

    if (controller_deadline_missed)
    {
        safety_state_controller_deadline_fault(
            status
        );

        return status->state;
    }


    /*
     * A current valid command has passed the sensor/deadline
     * checks.  Clear status fields that may have been left over
     * from a previous fault indication.
     */
    status->sensor_status =
        SENSOR_OK;

    status->controller_deadline_missed =
        0;

    status->fault_active =
        0;

    status->emergency_stop =
        0;
    /*
     * --------------------------------------------------------
     * Command must exist.
     * --------------------------------------------------------
     */

    if (command == NULL)
    {
        status->state =
            ACC_STATE_FAULT;

        status->fault_active =
            1;

        status->emergency_stop =
            1;

        status->timestamp_ns =
            timing_now_ns();

        return status->state;
    }


    /*
     * --------------------------------------------------------
     * Validate command timestamp.
     * --------------------------------------------------------
     */

    if (command->timestamp_ns == 0)
    {
        status->state =
            ACC_STATE_FAULT;

        status->fault_active =
            1;

        status->emergency_stop =
            1;

        status->timestamp_ns =
            timing_now_ns();

        return status->state;
    }


    /*
     * --------------------------------------------------------
     * Check command age.
     * --------------------------------------------------------
     */

    now_ns =
        timing_now_ns();

    if (now_ns == 0)
    {
        status->state =
            ACC_STATE_FAULT;

        status->fault_active =
            1;

        status->emergency_stop =
            1;

        status->timestamp_ns =
            timing_now_ns();

        return status->state;
    }


    if (now_ns < command->timestamp_ns)
    {
        status->state =
            ACC_STATE_FAULT;

        status->fault_active =
            1;

        status->emergency_stop =
            1;

        status->timestamp_ns =
            now_ns;

        return status->state;
    }


    command_age_ns =
        now_ns -
        command->timestamp_ns;


    if (command_age_ns >
        timing_ms_to_ns(
            SENSOR_STALE_TIMEOUT_MS
        ))
    {
        status->state =
            ACC_STATE_FAULT;

        status->fault_active =
            1;

        status->emergency_stop =
            1;

        status->timestamp_ns =
            now_ns;

        return status->state;
    }


    /*
     * --------------------------------------------------------
     * Check distance.
     * --------------------------------------------------------
     *
     * The Controller's command state is not accepted blindly.
     * Safety independently checks the distance contained in
     * the command's associated ACC state.
     *
     * The actual measured distance is maintained separately
     * by the Safety process.
     *
     * At this layer, an EMERGENCY command is always treated
     * as an emergency stop.
     */

    /*
     * --------------------------------------------------------
     * Validate PWM request.
     * --------------------------------------------------------
     */
    if (command->duty_percent < 0.0 ||
        command->duty_percent > 100.0)
    {
        status->state = ACC_STATE_FAULT;
        status->fault_active = 1;
        status->emergency_stop = 1;
        status->timestamp_ns = now_ns;
        return status->state;
    }

    /*
     * --------------------------------------------------------
     * Emergency/fault/stop commands are always stopped.
     * --------------------------------------------------------
     */
    if (command->state == ACC_STATE_EMERGENCY)
    {
        if (command->direction != MOTOR_STOP ||
            command->duty_percent != 0.0 ||
            command->actuator_enable != 0)
        {
            status->state = ACC_STATE_FAULT;
            status->fault_active = 1;
            status->emergency_stop = 1;
            status->timestamp_ns = now_ns;
            return status->state;
        }

        status->state = ACC_STATE_EMERGENCY;
        status->fault_active = 0;
        status->emergency_stop = 1;
        status->timestamp_ns = now_ns;
        return status->state;
    }

    if (command->state == ACC_STATE_FAULT)
    {
        status->state = ACC_STATE_FAULT;
        status->fault_active = 1;
        status->emergency_stop = 1;
        status->timestamp_ns = now_ns;
        return status->state;
    }

    if (command->state == ACC_STATE_STOPPED)
    {
        status->state = ACC_STATE_STOPPED;
        status->fault_active = 0;
        status->emergency_stop = 1;
        status->timestamp_ns = now_ns;
        return status->state;
    }

    /*
     * --------------------------------------------------------
     * Enabled propulsion commands.
     * --------------------------------------------------------
     */
    if (command->actuator_enable != 1 ||
        command->direction != MOTOR_FORWARD)
    {
        status->state = ACC_STATE_FAULT;
        status->fault_active = 1;
        status->emergency_stop = 1;
        status->timestamp_ns = now_ns;
        return status->state;
    }

    if (command->state == ACC_STATE_ACTIVE)
    {
        if (command->duty_percent != ACC_NORMAL_DUTY_PERCENT)
        {
            status->state = ACC_STATE_FAULT;
            status->fault_active = 1;
            status->emergency_stop = 1;
            status->timestamp_ns = now_ns;
            return status->state;
        }

        status->state = ACC_STATE_ACTIVE;
        status->fault_active = 0;
        status->emergency_stop = 0;
        status->timestamp_ns = now_ns;
        return status->state;
    }

    if (command->state == ACC_STATE_WARNING)
    {
        if (command->duty_percent != ACC_WARNING_DUTY_PERCENT)
        {
            status->state = ACC_STATE_FAULT;
            status->fault_active = 1;
            status->emergency_stop = 1;
            status->timestamp_ns = now_ns;
            return status->state;
        }

        status->state = ACC_STATE_WARNING;
        status->fault_active = 0;
        status->emergency_stop = 0;
        status->timestamp_ns = now_ns;
        return status->state;
    }

    /*
     * --------------------------------------------------------
     * Unknown state.
     * --------------------------------------------------------
     */

    status->state =
        ACC_STATE_FAULT;

    status->fault_active =
        1;

    status->emergency_stop =
        1;

    status->timestamp_ns =
        now_ns;


    return status->state;
}


/* ============================================================
 * ACTUATOR PERMISSION
 * ============================================================ */

int safety_state_actuator_allowed(
    const safety_status_t *status
)
{
    if (status == NULL)
    {
        return 0;
    }


    /*
     * Any fault or emergency condition blocks propulsion.
     */
    if (status->fault_active)
    {
        return 0;
    }


    if (status->emergency_stop)
    {
        return 0;
    }


    /*
     * Only ACTIVE and WARNING permit normal actuator
     * operation.
     */
    if (status->state == ACC_STATE_ACTIVE ||
        status->state == ACC_STATE_WARNING)
    {
        return 1;
    }


    return 0;
}


/* ============================================================
 * EMERGENCY QUERY
 * ============================================================ */

int safety_state_is_emergency(
    const safety_status_t *status
)
{
    if (status == NULL)
    {
        return 0;
    }


    if (status->state ==
        ACC_STATE_EMERGENCY)
    {
        return 1;
    }


    return 0;
}


/* ============================================================
 * FAULT QUERY
 * ============================================================ */

int safety_state_is_fault(
    const safety_status_t *status
)
{
    if (status == NULL)
    {
        return 1;
    }


    if (status->state ==
            ACC_STATE_FAULT ||
        status->fault_active)
    {
        return 1;
    }


    return 0;
}
