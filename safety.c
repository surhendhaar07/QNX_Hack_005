/*
 * QNX Intelligent Adaptive Cruise Control
 *
 * Safety Process
 *
 * Data path:
 *
 *     Controller
 *          |
 *          v
 *       Safety
 *          |
 *          v
 *       Actuator
 *
 * Safety is the final gatekeeper before motor control.
 *
 * Responsibilities:
 *     - Receive Controller commands
 *     - Validate commands
 *     - Detect stale Controller data
 *     - Detect sensor faults
 *     - Detect Controller deadline faults
 *     - Apply emergency-stop logic
 *     - Measure IPC latency
 *     - Measure Safety response/execution time
 *     - Send only safe commands to Actuator
 *
 * No simulated sensor values are generated.
 */

#include "safety.h"

#include "safety_state.h"

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
 * Controller -> Safety
 */
static ipc_queue_t controller_queue;

static uint64_t g_last_supervisor_send_ns = 0;
#define SUPERVISOR_REPORT_PERIOD_MS 100U
/*
 * Safety -> Actuator
 */
static ipc_queue_t actuator_queue;


/*
 * Safety -> Supervisor
 *
 * Used for safety status reporting.
 */
static ipc_queue_t supervisor_queue;


/*
 * IPC initialization status.
 */
static int safety_ipc_ready = 0;


/* ============================================================
 * INTERNAL COMMAND HELPER
 * ============================================================ */

static void safety_create_stop_command(
    control_command_t *command,
    acc_state_t state
)
{
    if (command == NULL)
    {
        return;
    }


    memset(
        command,
        0,
        sizeof(*command)
    );


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

    command->timestamp_ns =
        timing_now_ns();
}


/* ============================================================
 * INITIALIZATION
 * ============================================================ */

int safety_init(
    safety_context_t *context
)
{
    int result;


    if (context == NULL)
    {
        fprintf(
            stderr,
            "SAFETY ERROR: NULL context\n"
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


    context->running =
        1;


    safety_state_init(
        &context->status
    );


    context->sequence =
        0;


    /* --------------------------------------------------------
     * Validate IPC configuration
     * -------------------------------------------------------- */

    result =
        ipc_validate_message_size();

    if (result != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "SAFETY ERROR: IPC message size validation failed\n"
        );

        return ACC_FAILURE;
    }


    /* --------------------------------------------------------
     * Open Controller -> Safety queue
     * -------------------------------------------------------- */

    result =
        ipc_open_receiver(
            &controller_queue,
            ACC_CONTROLLER_QUEUE_NAME
        );

    if (result != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "SAFETY ERROR: unable to open Controller queue\n"
        );

        return ACC_FAILURE;
    }


    /* --------------------------------------------------------
     * Open Safety -> Actuator queue
     * -------------------------------------------------------- */

    result =
        ipc_open_sender(
            &actuator_queue,
            ACC_SAFETY_QUEUE_NAME
        );

    if (result != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "SAFETY ERROR: unable to open Actuator queue\n"
        );

        ipc_close(
            &controller_queue
        );

        return ACC_FAILURE;
    }


    /* --------------------------------------------------------
     * Open Safety -> Supervisor queue
     * -------------------------------------------------------- */

    result =
        ipc_open_sender(
            &supervisor_queue,
            ACC_SUPERVISOR_QUEUE_NAME
        );

    if (result != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "SAFETY ERROR: unable to open Supervisor queue\n"
        );

        ipc_close(
            &controller_queue
        );

        ipc_close(
            &actuator_queue
        );

        return ACC_FAILURE;
    }


    safety_ipc_ready =
        1;


    printf(
        "SAFETY: initialization complete\n"
    );

    return ACC_SUCCESS;
}


/* ============================================================
 * VALIDATE CONTROL COMMAND
 * ============================================================ */

int safety_validate_command(
    const control_command_t *command
)
{
    if (command == NULL)
    {
        return ACC_FAILURE;
    }


    /* --------------------------------------------------------
     * Validate PWM duty
     * -------------------------------------------------------- */

    if (isnan(command->duty_percent) ||
        isinf(command->duty_percent))
    {
        return ACC_FAILURE;
    }


    if (command->duty_percent < 0.0 ||
        command->duty_percent > 100.0)
    {
        return ACC_FAILURE;
    }


    /* --------------------------------------------------------
     * Validate target speed
     * -------------------------------------------------------- */

    if (isnan(command->target_speed_kmh) ||
        isinf(command->target_speed_kmh))
    {
        return ACC_FAILURE;
    }


    if (command->target_speed_kmh < 0.0)
    {
        return ACC_FAILURE;
    }


    /* --------------------------------------------------------
     * Validate timestamp
     * -------------------------------------------------------- */

    if (command->timestamp_ns == 0)
    {
        return ACC_FAILURE;
    }


    /* --------------------------------------------------------
     * Disabled actuator consistency
     * -------------------------------------------------------- */

    if (command->actuator_enable == 0)
    {
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


    /* --------------------------------------------------------
     * Emergency/fault consistency
     * -------------------------------------------------------- */

    if (command->state == ACC_STATE_EMERGENCY ||
        command->state == ACC_STATE_FAULT ||
        command->state == ACC_STATE_STOPPED)
    {
        if (command->direction != MOTOR_STOP)
        {
            return ACC_FAILURE;
        }

        if (command->duty_percent != 0.0)
        {
            return ACC_FAILURE;
        }

        if (command->actuator_enable != 0)
        {
            return ACC_FAILURE;
        }
    }


    /* --------------------------------------------------------
     * Enabled command must request forward motion
     * -------------------------------------------------------- */

    if (command->actuator_enable != 0)
    {
        if (command->direction != MOTOR_FORWARD)
        {
            return ACC_FAILURE;
        }
    }

    /*
     * Independently enforce the configured ACC speed policy.
     * Safety must never pass a command that does not match its
     * state.
     */
    if (command->state == ACC_STATE_ACTIVE)
    {
        if (command->actuator_enable != 1 ||
            command->direction != MOTOR_FORWARD ||
            command->duty_percent != ACC_NORMAL_DUTY_PERCENT)
        {
            return ACC_FAILURE;
        }
    }
    else if (command->state == ACC_STATE_WARNING)
    {
        if (command->actuator_enable != 1 ||
            command->direction != MOTOR_FORWARD ||
            command->duty_percent != ACC_WARNING_DUTY_PERCENT)
        {
            return ACC_FAILURE;
        }
    }


    return ACC_SUCCESS;
}


/* ============================================================
 * CONTROLLER DATA STALENESS
 * ============================================================ */

static int safety_command_is_stale(
    const control_command_t *command
)
{
    uint64_t now_ns;
    uint64_t age_ns;


    if (command == NULL)
    {
        return 1;
    }


    if (command->timestamp_ns == 0)
    {
        return 1;
    }


    now_ns =
        timing_now_ns();

    if (now_ns == 0)
    {
        return 1;
    }


    if (now_ns < command->timestamp_ns)
    {
        return 1;
    }


    age_ns =
        now_ns -
        command->timestamp_ns;


    if (age_ns >
        timing_ms_to_ns(
            SENSOR_STALE_TIMEOUT_MS
        ))
    {
        return 1;
    }


    return 0;
}


/* ============================================================
 * SEND ACTUATOR COMMAND
 * ============================================================ */

static int safety_send_actuator_command(
    const control_command_t *command
)
{

    ipc_message_t message;

    int result;


    if (command == NULL)
    {
        return ACC_FAILURE;
    }


    if (!safety_ipc_ready)
    {
        return ACC_FAILURE;
    }


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
            IPC_MSG_ACTUATOR_COMMAND,
            (unsigned int)timing_execution_time_ns(
                &command->timing
            )
        );

    /*
     * The command sequence is maintained separately by Safety.
     *
     * The value passed above is only a sequence placeholder.
     * It is replaced by the caller through the message header
     * before transmission.
     */

    if (result != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "SAFETY IPC ERROR: header initialization failed\n"
        );

        return ACC_FAILURE;
    }


    message.header.sender_pid =
        (int32_t)getpid();


    message.payload.control =
        *command;


    result =
        ipc_send(
            &actuator_queue,
            &message
        );

    if (result != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "SAFETY IPC ERROR: Actuator command send failed\n"
        );

        return ACC_FAILURE;
    }


    return ACC_SUCCESS;
}


/* ============================================================
 * SEND SAFETY STATUS
 * ============================================================ */

static int safety_send_status(
    safety_context_t *context
)
{
    ipc_message_t message;

    int result;


    if (context == NULL)
    {
        return ACC_FAILURE;
    }


    if (!safety_ipc_ready)
    {
        return ACC_FAILURE;
    }


    memset(
        &message,
        0,
        sizeof(message)
    );


    result =
        ipc_initialize_header(
            &message.header,
            IPC_MSG_SAFETY_STATUS,
            context->sequence
        );

    if (result != ACC_SUCCESS)
    {
        return ACC_FAILURE;
    }


    message.header.sender_pid =
        (int32_t)getpid();


    message.payload.safety =
        context->status;


    result =
        ipc_send(
            &supervisor_queue,
            &message
        );

    if (result != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "SAFETY IPC WARNING: status send failed\n"
        );

        return ACC_FAILURE;
    }


    return ACC_SUCCESS;
}


/* ============================================================
 * SAFETY THREAD
 * ============================================================ */

void *safety_thread(
    void *argument
)
{
    safety_context_t *context;

    ipc_message_t message;

    control_command_t safe_command;

    task_timing_t timing;

    uint64_t next_release_ns;

    uint64_t receive_timestamp_ns;

    uint64_t ipc_latency_ns;

    unsigned int receive_priority;

    sensor_status_t sensor_status;

    int controller_deadline_missed;

    int receive_result;

    int command_valid;

    int state_result;

    int send_result;


    context =
        (safety_context_t *)argument;


    if (context == NULL)
    {
        fprintf(
            stderr,
            "SAFETY THREAD ERROR: NULL context\n"
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
        SAFETY_PERIOD_MS
    );


    timing_period_init(
        &next_release_ns,
        SAFETY_PERIOD_MS
    );


    printf(
        "SAFETY THREAD: started\n"
    );


    /* ========================================================
     * SAFETY LOOP
     * ======================================================== */

    while (context->running)
    {
        /* ----------------------------------------------------
         * Start timing
         * ---------------------------------------------------- */


        context->sequence++;


        /* ----------------------------------------------------
         * Receive Controller command
         * ---------------------------------------------------- */

        memset(
            &message,
            0,
            sizeof(message)
        );


        receive_priority = 0;


        receive_result =
            ipc_receive(
                &controller_queue,
                &message,
                &receive_priority
            );


        receive_timestamp_ns =
            timing_now_ns();


        /* ----------------------------------------------------
         * Handle IPC receive failure
         * ---------------------------------------------------- */

        if (receive_result != ACC_SUCCESS)
        {
            fprintf(
                stderr,
                "SAFETY IPC ERROR: Controller receive failed\n"
            );


            safety_state_force_stop(
                &context->status
            );


            safety_create_stop_command(
                &safe_command,
                ACC_STATE_STOPPED
            );


            safe_command.timing =
                timing;


            safety_send_actuator_command(
                &safe_command
            );


            timing_stop(
                &timing
            );


            timing_period_wait(
                &next_release_ns,
                SAFETY_PERIOD_MS
            );


            continue;
        }

        /* Start execution timing only after a successful IPC receive. */
        if (timing_start(&timing) != ACC_SUCCESS)
        {
            fprintf(
                stderr,
                "SAFETY TIMING ERROR: timing_start failed\n"
            );
        }


        /* ----------------------------------------------------
         * Validate message type
         * ---------------------------------------------------- */

        if (message.header.type !=
            IPC_MSG_CONTROL_COMMAND)
        {
            fprintf(
                stderr,
                "SAFETY IPC ERROR: unexpected message type "
                "%d\n",
                message.header.type
            );


            safety_state_force_stop(
                &context->status
            );


            safety_create_stop_command(
                &safe_command,
                ACC_STATE_FAULT
            );


            timing_stop(
                &timing
            );


            safe_command.timing =
                timing;


            safety_send_actuator_command(
                &safe_command
            );


            timing_period_wait(
                &next_release_ns,
                SAFETY_PERIOD_MS
            );


            continue;
        }


        /* ----------------------------------------------------
         * Calculate IPC latency
         * ---------------------------------------------------- */

        ipc_latency_ns =
            ipc_calculate_latency_ns(
                message.header.timestamp_ns,
                receive_timestamp_ns
            );


        /* ----------------------------------------------------
         * Get Controller command
         * ---------------------------------------------------- */

        control_command_t *controller_command =
            &message.payload.control;

        /*
         * Use the IPC header timestamp as the authoritative
         * freshness timestamp.  The header is created immediately
         * before the command is sent to Safety, so this measures
         * the actual age of the queued Controller command.
         */
        controller_command->timestamp_ns =
            message.header.timestamp_ns;


        /* ----------------------------------------------------
         * Validate Controller command
         * ---------------------------------------------------- */

        command_valid =
            safety_validate_command(
                controller_command
            );


        /* ----------------------------------------------------
         * Detect stale Controller command
         * ---------------------------------------------------- */

        {
            int command_stale =
                safety_command_is_stale(controller_command);

            if (command_stale)
            {
                command_valid = 0;
                sensor_status = SENSOR_STALE;
            }
            else if (!command_valid)
            {
                sensor_status = SENSOR_INVALID;
            }
            else
            {
                sensor_status = SENSOR_OK;
            }
        }


        /* ----------------------------------------------------
         * Controller deadline condition
         * ---------------------------------------------------- */

        controller_deadline_missed =
            0;


        /*
         * The Controller embeds its task timing in the command.
         *
         * Check whether that execution exceeded the configured
         * Controller deadline.
         */
        if (timing_execution_time_ns(
                &controller_command->timing
            ) >
            timing_ms_to_ns(
                CONTROLLER_DEADLINE_MS
            ))
        {
            controller_deadline_missed =
                1;
        }


        /* ----------------------------------------------------
         * Update safety state
         * ---------------------------------------------------- */

        if (!command_valid)
        {
            if (sensor_status == SENSOR_STALE)
                safety_state_sensor_stale(&context->status);
            else
                safety_state_sensor_fault(&context->status, SENSOR_INVALID);

            state_result = ACC_STATE_FAULT;
        }
        else
        {
            state_result =
                safety_state_update(
                    &context->status,
                    controller_command,
                    sensor_status,
                    controller_deadline_missed
                );
        }


        /* ----------------------------------------------------
         * Generate final Safety command
         * ---------------------------------------------------- */

        if (state_result == ACC_STATE_EMERGENCY)
        {
            /*
             * Emergency:
             *
             *     propulsion disabled
             *     motor stop
             */
            safety_create_stop_command(
                &safe_command,
                ACC_STATE_EMERGENCY
            );
        }
        else if (state_result == ACC_STATE_FAULT)
        {
            /*
             * Fault:
             *
             *     deterministic safe stop
             */
            safety_create_stop_command(
                &safe_command,
                ACC_STATE_FAULT
            );
        }
        else if (state_result == ACC_STATE_STOPPED)
        {
            safety_create_stop_command(
                &safe_command,
                ACC_STATE_STOPPED
            );
        }
        else if (safety_state_actuator_allowed(
                     &context->status
                 ))
        {
            /*
             * Controller command has passed Safety validation.
             */
            safe_command =
                *controller_command;
        }
        else
        {
            /*
             * Anything not explicitly permitted is stopped.
             */
            safety_create_stop_command(
                &safe_command,
                ACC_STATE_FAULT
            );
        }


        /* ----------------------------------------------------
         * Complete Safety timing
         * ---------------------------------------------------- */

        if (timing_stop(
                &timing
            ) != ACC_SUCCESS)
        {
            fprintf(
                stderr,
                "SAFETY TIMING ERROR: timing_stop failed\n"
            );
        }


        context->status.timing =
            timing;


        /* ----------------------------------------------------
         * Safety deadline monitoring
         * ---------------------------------------------------- */

        if (timing_deadline_missed(
                &timing
            ))
        {
            fprintf(
                stderr,
                "SAFETY WARNING: Safety task deadline missed "
                "(%.3f ms)\n",
                timing_execution_time_ms(
                    &timing
                )
            );

            safety_state_force_stop(
                &context->status
            );

            safety_create_stop_command(
                &safe_command,
                ACC_STATE_STOPPED
            );

            safe_command.timing =
                timing;
        }


        /* ----------------------------------------------------
         * Send final command to Actuator
         * ---------------------------------------------------- */

        safe_command.timing =
            timing;


        send_result =
            safety_send_actuator_command(
                &safe_command
            );

        printf(
            "[SAFETY->ACTUATOR] "
            "state=%d duty=%.1f%% enable=%d direction=%d\n",
            safe_command.state,
            safe_command.duty_percent,
            safe_command.actuator_enable,
            safe_command.direction
        );

        if (send_result != ACC_SUCCESS)
        {
            fprintf(
                stderr,
                "SAFETY WARNING: Actuator IPC send failure\n"
            );
        }


        /* ----------------------------------------------------
         * Send Safety status to Supervisor
         * ---------------------------------------------------- */

        /*
         * Supervisor status is diagnostic/health information.
         * Do not enqueue it on every Safety cycle because the
         * Supervisor consumes it at a slower 100 ms period.
         */
        {
            uint64_t now_ns =
                timing_now_ns();

            if (g_last_supervisor_send_ns == 0 ||
                (now_ns >= g_last_supervisor_send_ns &&
                 (now_ns - g_last_supervisor_send_ns) >=
                     timing_ms_to_ns(
                         SUPERVISOR_REPORT_PERIOD_MS)))
            {
                if (safety_send_status(
                        context
                    ) != ACC_SUCCESS)
                {
                    fprintf(
                        stderr,
                        "SAFETY WARNING: Supervisor status send failed\n"
                    );
                }
                else
                {
                    g_last_supervisor_send_ns =
                        now_ns;
                }
            }
        }


        /* ----------------------------------------------------
         * Diagnostic output
         * ---------------------------------------------------- */

        printf(
            "[SAFETY]     | #%04u | STATE %d | SENSOR %d | FAULT %s | E-STOP %s | DEADLINE %s | IPC %6.3f ms | EXEC %6.3f ms\n",
            context->sequence,
            context->status.state,
            context->status.sensor_status,
            context->status.fault_active ? "YES" : "NO",
            context->status.emergency_stop ? "YES" : "NO",
            context->status.controller_deadline_missed ? "MISS" : "OK",
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
                SAFETY_PERIOD_MS
            ) != ACC_SUCCESS)
        {
            fprintf(
                stderr,
                "SAFETY TIMING ERROR: periodic wait failed\n"
            );
        }
    }


    printf(
        "SAFETY THREAD: stopped\n"
    );


    return NULL;
}


/* ============================================================
 * SHUTDOWN
 * ============================================================ */

int safety_shutdown(
    safety_context_t *context
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
     * Close Controller queue
     * -------------------------------------------------------- */

    if (safety_ipc_ready)
    {
        result =
            ipc_close(
                &controller_queue
            );

        if (result != ACC_SUCCESS)
        {
            fprintf(
                stderr,
                "SAFETY WARNING: Controller queue close failed\n"
            );

            shutdown_result =
                ACC_FAILURE;
        }


        /* ----------------------------------------------------
         * Close Actuator queue
         * ---------------------------------------------------- */

        result =
            ipc_close(
                &actuator_queue
            );

        if (result != ACC_SUCCESS)
        {
            fprintf(
                stderr,
                "SAFETY WARNING: Actuator queue close failed\n"
            );

            shutdown_result =
                ACC_FAILURE;
        }


        /* ----------------------------------------------------
         * Close Supervisor queue
         * ---------------------------------------------------- */

        result =
            ipc_close(
                &supervisor_queue
            );

        if (result != ACC_SUCCESS)
        {
            fprintf(
                stderr,
                "SAFETY WARNING: Supervisor queue close failed\n"
            );

            shutdown_result =
                ACC_FAILURE;
        }


        safety_ipc_ready =
            0;
    }


    printf(
        "SAFETY: shutdown complete\n"
    );


    return shutdown_result;
}
