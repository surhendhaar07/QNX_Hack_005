/*
 * QNX Intelligent Adaptive Cruise Control
 *
 * Controller Process
 *
 * Data path:
 *
 *     Tracking
 *         |
 *         v
 *     Controller
 *         |
 *         v
 *       Safety
 *         |
 *         v
 *      Actuator
 *
 * The Controller never directly controls the motor.
 *
 * Responsibilities:
 *     - Receive Tracking data
 *     - Validate sensor/tracking information
 *     - Determine ACC state
 *     - Calculate PWM command
 *     - Monitor stale data
 *     - Measure IPC latency
 *     - Measure execution/response time
 *     - Check controller deadline
 *     - Send command to Safety
 */

#include "controller.h"

#include "../common/acc_config.h"
#include "../common/ipc.h"
#include "../common/ipc_types.h"
#include "../common/timing.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>


/* ============================================================
 * IPC QUEUES
 * ============================================================ */

/*
 * Tracking -> Controller
 */
static ipc_queue_t tracking_queue;


/*
 * Controller -> Safety
 */
static ipc_queue_t safety_queue;


/*
 * IPC initialization status.
 */
static int controller_ipc_ready = 0;


/* ============================================================
 * INTERNAL HELPERS
 * ============================================================ */

/*
 * Limit a PWM duty-cycle request to the valid range.
 */
static double controller_limit_duty(
    double duty
)
{
    if (duty < 0.0)
    {
        return 0.0;
    }

    if (duty > 100.0)
    {
        return 100.0;
    }

    return duty;
}


/* ============================================================
 * INITIALIZATION
 * ============================================================ */

int controller_init(
    controller_context_t *context
)
{
    int result;


    if (context == NULL)
    {
        fprintf(
            stderr,
            "CONTROLLER ERROR: NULL context\n"
        );

        return ACC_FAILURE;
    }


    /* --------------------------------------------------------
     * Initialize context
     * -------------------------------------------------------- */

    memset(
        context,
        0,
        sizeof(*context)
    );

    context->running = 1;

    context->sequence = 0;

    context->distance_cm = 0.0;

    context->relative_speed_cm_s = 0.0;

    context->state =
        ACC_STATE_INIT;

    context->last_tracking_timestamp_ns = 0;

    context->fault_count = 0;


    /* --------------------------------------------------------
     * Validate IPC message size
     * -------------------------------------------------------- */

    result =
        ipc_validate_message_size();

    if (result != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "CONTROLLER ERROR: IPC message size validation failed\n"
        );

        return ACC_FAILURE;
    }


    /* --------------------------------------------------------
     * Open Tracking -> Controller queue
     * -------------------------------------------------------- */

    result =
        ipc_open_receiver(
            &tracking_queue,
            ACC_TRACKING_QUEUE_NAME
        );

    if (result != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "CONTROLLER ERROR: unable to open Tracking queue\n"
        );

        return ACC_FAILURE;
    }


    /* --------------------------------------------------------
     * Open Controller -> Safety queue
     * -------------------------------------------------------- */

    result =
        ipc_open_sender(
            &safety_queue,
            ACC_CONTROLLER_QUEUE_NAME
        );

    if (result != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "CONTROLLER ERROR: unable to open Safety queue\n"
        );

        ipc_close(
            &tracking_queue
        );

        return ACC_FAILURE;
    }


    controller_ipc_ready = 1;


    printf(
        "CONTROLLER: initialization complete\n"
    );

    return ACC_SUCCESS;
}


/* ============================================================
 * VALIDATE TRACKING DATA
 * ============================================================ */

int controller_validate_tracking(
    const tracking_data_t *tracking
)
{
    if (tracking == NULL)
    {
        return ACC_FAILURE;
    }


    /* --------------------------------------------------------
     * Sensor status
     * -------------------------------------------------------- */

    if (tracking->status != SENSOR_OK)
    {
        return ACC_FAILURE;
    }


    /* --------------------------------------------------------
     * Timestamp
     * -------------------------------------------------------- */

    if (tracking->timestamp_ns == 0)
    {
        return ACC_FAILURE;
    }


    /* --------------------------------------------------------
     * Distance
     * -------------------------------------------------------- */

    if (tracking->distance_cm <
        ACC_MIN_VALID_DISTANCE_CM)
    {
        return ACC_FAILURE;
    }

    if (tracking->distance_cm >
        ACC_MAX_VALID_DISTANCE_CM)
    {
        return ACC_FAILURE;
    }


    /* --------------------------------------------------------
     * Relative speed
     * -------------------------------------------------------- */

    if (isnan(
            tracking->relative_speed_cm_s))
    {
        return ACC_FAILURE;
    }

    if (isinf(
            tracking->relative_speed_cm_s))
    {
        return ACC_FAILURE;
    }


    return ACC_SUCCESS;
}


/* ============================================================
 * DETERMINE ACC STATE
 * ============================================================ */

acc_state_t controller_determine_state(
    const tracking_data_t *tracking
)
{
    uint64_t now_ns;
    uint64_t age_ns;


    if (tracking == NULL)
    {
        return ACC_STATE_FAULT;
    }


    /* --------------------------------------------------------
     * Sensor fault
     * -------------------------------------------------------- */

    if (tracking->status != SENSOR_OK)
    {
        return ACC_STATE_FAULT;
    }


    /* --------------------------------------------------------
     * Check timestamp
     * -------------------------------------------------------- */

    now_ns =
        timing_now_ns();

    if (now_ns == 0 ||
        tracking->timestamp_ns == 0)
    {
        return ACC_STATE_FAULT;
    }


    if (now_ns >= tracking->timestamp_ns)
    {
        age_ns =
            now_ns -
            tracking->timestamp_ns;
    }
    else
    {
        return ACC_STATE_FAULT;
    }


    /*
     * Tracking data older than the configured stale timeout
     * must not be used for normal ACC control.
     */
    if (age_ns >
        timing_ms_to_ns(
            SENSOR_STALE_TIMEOUT_MS
        ))
    {
        return ACC_STATE_FAULT;
    }


    /* --------------------------------------------------------
     * Emergency distance
     * -------------------------------------------------------- */

    if (tracking->distance_cm <=
        ACC_EMERGENCY_DISTANCE_CM)
    {
        return ACC_STATE_EMERGENCY;
    }


    /* --------------------------------------------------------
     * Warning distance
     * -------------------------------------------------------- */

    if (tracking->distance_cm <
        ACC_WARNING_DISTANCE_CM)
    {
        return ACC_STATE_WARNING;
    }


    /* --------------------------------------------------------
     * Safe following distance
     * -------------------------------------------------------- */

    if (tracking->distance_cm <=
        ACC_SAFE_DISTANCE_CM)
    {
        return ACC_STATE_ACTIVE;
    }


    /* --------------------------------------------------------
     * No close target
     * -------------------------------------------------------- */

    return ACC_STATE_ACTIVE;
}


/* ============================================================
 * CALCULATE PWM DUTY
 * ============================================================ */

double controller_calculate_duty(
    const tracking_data_t *tracking
)
{
    double duty;

    if (tracking == NULL)
    {
        return 0.0;
    }

    /*
     * Never generate propulsion from invalid sensor data.
     */
    if (tracking->status != SENSOR_OK)
    {
        return 0.0;
    }

    /*
     * Emergency zone:
     * distance <= 15 cm -> propulsion OFF.
     */
    if (tracking->distance_cm <=
        ACC_EMERGENCY_DISTANCE_CM)
    {
        return 0.0;
    }

    /*
     * Slow zone:
     * 15 cm < distance < 30 cm -> reduced propulsion.
     */
    if (tracking->distance_cm <
            ACC_WARNING_DISTANCE_CM)
    {
        return ACC_WARNING_DUTY_PERCENT;
    }

    /*
     * Normal zone:
     * distance >= 30 cm -> exactly 100% propulsion.
     */
    duty = ACC_NORMAL_DUTY_PERCENT;

    /*
     * User-defined ACC policy:
     *   distance >= 30 cm -> exactly 100% speed.
     *
     * Do not reduce this command based on relative speed;
     * the requested distance zones determine the motor speed.
     */
    return controller_limit_duty(duty);
}


/* ============================================================
 * CALCULATE CONTROL COMMAND
 * ============================================================ */

int controller_calculate_command(
    controller_context_t *context,
    const tracking_data_t *tracking,
    control_command_t *command
)
{
    acc_state_t state;
    double duty;


    if (context == NULL ||
        tracking == NULL ||
        command == NULL)
    {
        return ACC_FAILURE;
    }


    /* --------------------------------------------------------
     * Clear command
     * -------------------------------------------------------- */

    memset(
        command,
        0,
        sizeof(*command)
    );


    /* --------------------------------------------------------
     * Validate tracking input
     * -------------------------------------------------------- */

    if (controller_validate_tracking(
            tracking
        ) != ACC_SUCCESS)
    {
        context->fault_count++;

        context->state =
            ACC_STATE_FAULT;

        command->direction =
            MOTOR_STOP;

        command->duty_percent =
            0.0;

        command->target_speed_kmh =
            0.0;

        command->state =
            ACC_STATE_FAULT;

        command->actuator_enable =
            0;

        command->timestamp_ns =
            timing_now_ns();

        return ACC_FAILURE;
    }


    /* --------------------------------------------------------
     * Determine ACC state
     * -------------------------------------------------------- */

    state =
        controller_determine_state(
            tracking
        );


    context->state =
        state;

    context->distance_cm =
        tracking->distance_cm;

    context->relative_speed_cm_s =
        tracking->relative_speed_cm_s;

    context->last_tracking_timestamp_ns =
        tracking->timestamp_ns;


    /* --------------------------------------------------------
     * Default command
     * -------------------------------------------------------- */

    command->direction =
        MOTOR_STOP;

    command->duty_percent =
        0.0;

    command->target_speed_kmh =
        0.0;

    command->state =
        state;

    command->actuator_enable =
        0;


    /* --------------------------------------------------------
     * Generate command based on state
     * -------------------------------------------------------- */

    switch (state)
    {
        case ACC_STATE_ACTIVE:

            duty =
                controller_calculate_duty(
                    tracking
                );

            command->direction =
                MOTOR_FORWARD;

            command->duty_percent =
                duty;

            command->target_speed_kmh =
                ACC_TARGET_SPEED_KMH;

            command->actuator_enable =
                1;

            break;


        case ACC_STATE_WARNING:

            /*
             * Warning state reduces propulsion.
             */
            command->direction =
                MOTOR_FORWARD;

            command->duty_percent =
                ACC_WARNING_DUTY_PERCENT;

            command->target_speed_kmh =
                ACC_TARGET_SPEED_KMH * 0.5;

            command->actuator_enable =
                1;

            break;


        case ACC_STATE_EMERGENCY:

            /*
             * Emergency state immediately disables propulsion.
             *
             * Safety will independently verify this condition.
             */
            command->direction =
                MOTOR_STOP;

            command->duty_percent =
                0.0;

            command->target_speed_kmh =
                0.0;

            command->actuator_enable =
                0;

            break;


        case ACC_STATE_FAULT:

            /*
             * Sensor/tracking fault results in safe stop.
             */
            command->direction =
                MOTOR_STOP;

            command->duty_percent =
                0.0;

            command->target_speed_kmh =
                0.0;

            command->actuator_enable =
                0;

            break;


        case ACC_STATE_STOPPED:

        case ACC_STATE_INIT:

        default:

            command->direction =
                MOTOR_STOP;

            command->duty_percent =
                0.0;

            command->target_speed_kmh =
                0.0;

            command->actuator_enable =
                0;

            break;
    }


    command->timestamp_ns =
        timing_now_ns();


    return ACC_SUCCESS;
}


/* ============================================================
 * VALIDATE CONTROL COMMAND
 * ============================================================ */

int controller_validate_command(
    const control_command_t *command
)
{
    if (command == NULL)
    {
        return ACC_FAILURE;
    }


    /* --------------------------------------------------------
     * PWM range
     * -------------------------------------------------------- */

    if (command->duty_percent < 0.0 ||
        command->duty_percent > 100.0)
    {
        return ACC_FAILURE;
    }


    /* --------------------------------------------------------
     * Timestamp
     * -------------------------------------------------------- */

    if (command->timestamp_ns == 0)
    {
        return ACC_FAILURE;
    }


    /* --------------------------------------------------------
     * Safety consistency checks
     * -------------------------------------------------------- */

    if (command->actuator_enable == 0)
    {
        /*
         * Disabled actuator must not request propulsion.
         */
        if (command->direction != MOTOR_STOP)
        {
            return ACC_FAILURE;
        }

        if (command->duty_percent != 0.0)
        {
            return ACC_FAILURE;
        }

        if (command->target_speed_kmh != 0.0)
        {
            return ACC_FAILURE;
        }
    }


    if (command->state ==
            ACC_STATE_EMERGENCY ||
        command->state ==
            ACC_STATE_FAULT ||
        command->state ==
            ACC_STATE_STOPPED)
    {
        /*
         * These states must always result in propulsion stop.
         */
        if (command->direction != MOTOR_STOP ||
            command->duty_percent != 0.0 ||
            command->actuator_enable != 0)
        {
            return ACC_FAILURE;
        }
    }


    return ACC_SUCCESS;
}


/* ============================================================
 * SEND CONTROL COMMAND
 * ============================================================ */

static int controller_send_command(
    const control_command_t *command,
    uint32_t sequence
)
{
    ipc_message_t message;

    int result;


    if (command == NULL)
    {
        return ACC_FAILURE;
    }


    if (!controller_ipc_ready)
    {
        return ACC_FAILURE;
    }


    /* --------------------------------------------------------
     * Clear message
     * -------------------------------------------------------- */

    memset(
        &message,
        0,
        sizeof(message)
    );


    /* --------------------------------------------------------
     * Initialize IPC header
     * -------------------------------------------------------- */

    result =
        ipc_initialize_header(
            &message.header,
            IPC_MSG_CONTROL_COMMAND,
            sequence
        );

    if (result != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "CONTROLLER IPC ERROR: header initialization failed\n"
        );

        return ACC_FAILURE;
    }


    message.header.sender_pid =
        (int32_t)getpid();


    /* --------------------------------------------------------
     * Store command in IPC payload
     * -------------------------------------------------------- */

    message.payload.control =
        *command;


    /* --------------------------------------------------------
     * Send command to Safety
     * -------------------------------------------------------- */

    result =
        ipc_send(
            &safety_queue,
            &message
        );

    if (result != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "CONTROLLER IPC ERROR: command send failed\n"
        );

        return ACC_FAILURE;
    }


    return ACC_SUCCESS;
}


/* ============================================================
 * CONTROLLER THREAD
 * ============================================================ */

void *controller_thread(
    void *argument
)
{
    controller_context_t *context;

    ipc_message_t message;

    control_command_t command;

    task_timing_t timing;

    uint64_t next_release_ns;

    uint64_t receive_timestamp_ns;

    uint64_t ipc_latency_ns;

    unsigned int receive_priority;

    int receive_result;
    int command_result;
    int send_result;


    context =
        (controller_context_t *)argument;


    if (context == NULL)
    {
        fprintf(
            stderr,
            "CONTROLLER THREAD ERROR: NULL context\n"
        );

        return NULL;
    }


    /* --------------------------------------------------------
     * Initialize timing
     * -------------------------------------------------------- */

    timing_init(
        &timing
    );


    timing_set_deadline(
        &timing,
        CONTROLLER_DEADLINE_MS
    );


    timing_period_init(
        &next_release_ns,
        CONTROLLER_PERIOD_MS
    );


    printf(
        "CONTROLLER THREAD: started\n"
    );


    /* ========================================================
     * CONTROLLER LOOP
     * ======================================================== */

    while (context->running)
    {
        /* ----------------------------------------------------
         * Start timing
         * ---------------------------------------------------- */


        /* ----------------------------------------------------
         * Receive Tracking data
         * ---------------------------------------------------- */

        memset(
            &message,
            0,
            sizeof(message)
        );

        receive_priority = 0;


        receive_result =
            ipc_receive(
                &tracking_queue,
                &message,
                &receive_priority
            );


        receive_timestamp_ns =
            timing_now_ns();


        if (receive_result != ACC_SUCCESS)
        {
            fprintf(
                stderr,
                "CONTROLLER IPC ERROR: receive failed\n"
            );


            /*
             * No valid tracking input.
             *
             * Generate an explicit safe command.
             */
            memset(
                &command,
                0,
                sizeof(command)
            );


            command.direction =
                MOTOR_STOP;

            command.duty_percent =
                0.0;

            command.target_speed_kmh =
                0.0;

            command.state =
                ACC_STATE_FAULT;

            command.actuator_enable =
                0;

            command.timestamp_ns =
                timing_now_ns();


            context->state =
                ACC_STATE_FAULT;

            context->fault_count++;


            timing_stop(
                &timing
            );


            command.timing =
                timing;


            send_result =
                controller_send_command(
                    &command,
                    ++context->sequence
                );


            if (send_result != ACC_SUCCESS)
            {
                fprintf(
                    stderr,
                    "CONTROLLER WARNING: failed to send "
                    "safe command\n"
                );
            }


            if (timing_period_wait(
                    &next_release_ns,
                    CONTROLLER_PERIOD_MS
                ) != ACC_SUCCESS)
            {
                fprintf(
                    stderr,
                    "CONTROLLER TIMING ERROR: periodic wait failed\n"
                );
            }


            continue;
        }

        /* Start execution timing only after a successful IPC receive. */
        if (timing_start(&timing) != ACC_SUCCESS)
        {
            fprintf(
                stderr,
                "CONTROLLER TIMING ERROR: timing_start failed\n"
            );
        }


        /* ----------------------------------------------------
         * Verify message type
         * ---------------------------------------------------- */

        if (message.header.type !=
            IPC_MSG_TRACKING_DATA)
        {
            fprintf(
                stderr,
                "CONTROLLER IPC ERROR: unexpected message type "
                "%d\n",
                message.header.type
            );


            timing_stop(
                &timing
            );


            timing_period_wait(
                &next_release_ns,
                CONTROLLER_PERIOD_MS
            );


            continue;
        }


        /* ----------------------------------------------------
         * Measure IPC latency
         * ---------------------------------------------------- */

        ipc_latency_ns =
            ipc_calculate_latency_ns(
                message.header.timestamp_ns,
                receive_timestamp_ns
            );


        /* ----------------------------------------------------
         * Calculate control command
         * ---------------------------------------------------- */

        command_result =
            controller_calculate_command(
                context,
                &message.payload.tracking,
                &command
            );


        /*
         * If calculation failed, enforce safe stop.
         */
        if (command_result != ACC_SUCCESS)
        {
            memset(
                &command,
                0,
                sizeof(command)
            );


            command.direction =
                MOTOR_STOP;

            command.duty_percent =
                0.0;

            command.target_speed_kmh =
                0.0;

            command.state =
                ACC_STATE_FAULT;

            command.actuator_enable =
                0;

            command.timestamp_ns =
                timing_now_ns();
        }


        /* ----------------------------------------------------
         * Validate command
         * ---------------------------------------------------- */

        if (controller_validate_command(
                &command
            ) != ACC_SUCCESS)
        {
            fprintf(
                stderr,
                "CONTROLLER ERROR: invalid command generated\n"
            );


            /*
             * Fail safe.
             */
            memset(
                &command,
                0,
                sizeof(command)
            );


            command.direction =
                MOTOR_STOP;

            command.duty_percent =
                0.0;

            command.target_speed_kmh =
                0.0;

            command.state =
                ACC_STATE_FAULT;

            command.actuator_enable =
                0;

            command.timestamp_ns =
                timing_now_ns();


            context->state =
                ACC_STATE_FAULT;

            context->fault_count++;
        }


        /* ----------------------------------------------------
         * Complete controller timing
         * ---------------------------------------------------- */

        if (timing_stop(
                &timing
            ) != ACC_SUCCESS)
        {
            fprintf(
                stderr,
                "CONTROLLER TIMING ERROR: timing_stop failed\n"
            );
        }


        command.timing =
            timing;


        /* ----------------------------------------------------
         * Deadline monitoring
         * ---------------------------------------------------- */

        if (timing_deadline_missed(
                &timing
            ))
        {
            fprintf(
                stderr,
                "CONTROLLER DEADLINE MISSED: "
                "execution=%.3f ms "
                "deadline=%d ms\n",
                timing_execution_time_ms(
                    &timing
                ),
                CONTROLLER_DEADLINE_MS
            );


            /*
             * A missed controller deadline must not allow a
             * stale propulsion command to continue.
             *
             * Replace it with a safe stop.
             */
            command.direction =
                MOTOR_STOP;

            command.duty_percent =
                0.0;

            command.target_speed_kmh =
                0.0;

            command.state =
                ACC_STATE_FAULT;

            command.actuator_enable =
                0;

            command.timestamp_ns =
                timing_now_ns();
        }


        /* ----------------------------------------------------
         * Send command to Safety
         * ---------------------------------------------------- */

        send_result =
            controller_send_command(
                &command,
                ++context->sequence
            );


        if (send_result != ACC_SUCCESS)
        {
            fprintf(
                stderr,
                "CONTROLLER WARNING: IPC send failure\n"
            );
        }


        /* ----------------------------------------------------
         * Diagnostic output
         * ---------------------------------------------------- */

        printf(
            "[CONTROLLER] | #%04u | DIST %7.2f cm | REL SPD %8.2f cm/s | STATE %d | DUTY %6.1f%% | %s | IPC %6.3f ms | EXEC %6.3f ms\n",
            context->sequence,
            message.payload.tracking.distance_cm,
            message.payload.tracking.relative_speed_cm_s,
            command.state,
            command.duty_percent,
            command.actuator_enable ? "ENABLED" : "DISABLED",
            timing_ns_to_ms(
                ipc_latency_ns
            ),
            timing_execution_time_ms(
                &timing
            )
        );


        /* ----------------------------------------------------
         * Periodic release
         * ---------------------------------------------------- */

        if (timing_period_wait(
                &next_release_ns,
                CONTROLLER_PERIOD_MS
            ) != ACC_SUCCESS)
        {
            fprintf(
                stderr,
                "CONTROLLER TIMING ERROR: periodic wait failed\n"
            );
        }
    }


    printf(
        "CONTROLLER THREAD: stopped\n"
    );


    return NULL;
}


/* ============================================================
 * SHUTDOWN
 * ============================================================ */

int controller_shutdown(
    controller_context_t *context
)
{
    int result;

    int shutdown_result =
        ACC_SUCCESS;


    if (context == NULL)
    {
        return ACC_FAILURE;
    }


    /* --------------------------------------------------------
     * Stop worker
     * -------------------------------------------------------- */

    context->running = 0;


    /* --------------------------------------------------------
     * Close IPC queues
     * -------------------------------------------------------- */

    if (controller_ipc_ready)
    {
        result =
            ipc_close(
                &tracking_queue
            );

        if (result != ACC_SUCCESS)
        {
            fprintf(
                stderr,
                "CONTROLLER WARNING: Tracking queue close failed\n"
            );

            shutdown_result =
                ACC_FAILURE;
        }


        result =
            ipc_close(
                &safety_queue
            );

        if (result != ACC_SUCCESS)
        {
            fprintf(
                stderr,
                "CONTROLLER WARNING: Safety queue close failed\n"
            );

            shutdown_result =
                ACC_FAILURE;
        }


        controller_ipc_ready = 0;
    }


    printf(
        "CONTROLLER: shutdown complete\n"
    );


    return shutdown_result;
}
