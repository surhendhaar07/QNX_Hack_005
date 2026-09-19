/*
 * QNX Intelligent Adaptive Cruise Control
 *
 * Actuator Process
 *
 * Safety -> Actuator -> L298N -> DC Motor
 *
 * The Actuator process accepts commands only from Safety.
 *
 * No simulated motor output is generated.
 */

#include "actuator.h"

#include "l298n.h"
#include "pwm.h"

#include "../common/acc_config.h"
#include "../common/ipc.h"
#include "../common/ipc_types.h"
#include "../common/timing.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>


/* ============================================================
 * IPC
 * ============================================================ */

static ipc_queue_t safety_queue;

static ipc_queue_t supervisor_queue;

static int actuator_ipc_ready = 0;


/* ============================================================
 * COMMAND FRESHNESS
 * ============================================================ */

static int actuator_command_is_stale(
    const control_command_t *command)
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

    now_ns = timing_now_ns();

    if (now_ns < command->timestamp_ns)
    {
        return 1;
    }

    age_ns =
        now_ns - command->timestamp_ns;

    if (age_ns >
        timing_ms_to_ns(
            SENSOR_STALE_TIMEOUT_MS))
    {
        return 1;
    }

    return 0;
}


/* ============================================================
 * COMMAND VALIDATION
 * ============================================================ */

int actuator_validate_command(
    const control_command_t *command)
{
    if (command == NULL)
    {
        return ACC_FAILURE;
    }

    /*
     * Validate PWM duty.
     */
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

    /*
     * Validate target speed.
     */
    if (isnan(command->target_speed_kmh) ||
        isinf(command->target_speed_kmh))
    {
        return ACC_FAILURE;
    }

    if (command->target_speed_kmh < 0.0)
    {
        return ACC_FAILURE;
    }

    /*
     * Timestamp must be present.
     */
    if (command->timestamp_ns == 0)
    {
        return ACC_FAILURE;
    }

    /*
     * Disabled actuator must be stopped.
     */
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
    }

    /*
     * Emergency, fault and stopped states
     * must never request motor motion.
     */
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

    /*
     * Enabled ACC operation is forward only.
     */
    if (command->actuator_enable != 0)
    {
        if (command->direction != MOTOR_FORWARD)
        {
            return ACC_FAILURE;
        }
    }

    /*
     * Validate ACC state.
     */
    if (command->state != ACC_STATE_ACTIVE &&
        command->state != ACC_STATE_WARNING &&
        command->state != ACC_STATE_EMERGENCY &&
        command->state != ACC_STATE_FAULT &&
        command->state != ACC_STATE_STOPPED)
    {
        return ACC_FAILURE;
    }

    return ACC_SUCCESS;
}


/* ============================================================
 * SAFE STOP
 * ============================================================ */

int actuator_safe_stop(void)
{
    int result;
    int status = ACC_SUCCESS;

    /*
     * Stop PWM output.
     *
     * pwm_set_duty() returns int.
     */
    result =
        pwm_set_duty(0.0);

    if (result != ACC_SUCCESS)
    {
        status = ACC_FAILURE;
    }

    /*
     * Stop the L298N.
     *
     * l298n_stop() returns int.
     */
    result =
        l298n_stop();

    if (result != ACC_SUCCESS)
    {
        status = ACC_FAILURE;
    }

    return status;
}


/* ============================================================
 * APPLY COMMAND
 * ============================================================ */

int actuator_apply_command(
    const control_command_t *command)
{
    int result;

    if (command == NULL)
    {
        return ACC_FAILURE;
    }

    /*
     * Validate before accessing hardware.
     */
    if (actuator_validate_command(command)
        != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "ACTUATOR ERROR: invalid command\n");

        actuator_safe_stop();

        return ACC_FAILURE;
    }

    /*
     * Check freshness.
     */
    if (actuator_command_is_stale(command))
    {
        fprintf(
            stderr,
            "ACTUATOR ERROR: stale Safety command\n");

        actuator_safe_stop();

        return ACC_FAILURE;
    }

    /*
     * Any explicit stop state results in a safe stop.
     */
    if (command->state == ACC_STATE_EMERGENCY ||
        command->state == ACC_STATE_FAULT ||
        command->state == ACC_STATE_STOPPED ||
        command->actuator_enable == 0 ||
        command->direction == MOTOR_STOP)
    {
        return actuator_safe_stop();
    }

    /*
     * ACC permits forward operation only.
     */
    if (command->direction != MOTOR_FORWARD)
    {
        actuator_safe_stop();

        return ACC_FAILURE;
    }

    /*
     * Set L298N direction.
     */
    result =
        l298n_set_direction(
            MOTOR_FORWARD);

    if (result != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "ACTUATOR ERROR: unable to set "
            "motor direction\n");

        actuator_safe_stop();

        return ACC_FAILURE;
    }

    /*
     * Apply PWM duty cycle.
     */
    result =
        pwm_set_duty(
            command->duty_percent);

    if (result != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "ACTUATOR ERROR: PWM update failed\n");

        actuator_safe_stop();

        return ACC_FAILURE;
    }

    return ACC_SUCCESS;
}


/* ============================================================
 * INITIALIZATION
 * ============================================================ */

int actuator_init(
    actuator_context_t *context)
{
    int result;

    if (context == NULL)
    {
        fprintf(
            stderr,
            "ACTUATOR ERROR: NULL context\n");

        return ACC_FAILURE;
    }

    memset(
        context,
        0,
        sizeof(*context));

    context->running = 1;

    context->sequence = 0;

    context->motor_status =
        MOTOR_DISABLED;

    context->last_command_timestamp_ns =
        0;

    /*
     * Validate IPC message configuration.
     */
    result =
        ipc_validate_message_size();

    if (result != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "ACTUATOR ERROR: IPC message size "
            "validation failed\n");

        return ACC_FAILURE;
    }

    /*
     * Open Safety -> Actuator queue.
     */
    result =
        ipc_open_receiver(
            &safety_queue,
            ACC_SAFETY_QUEUE_NAME);

    if (result != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "ACTUATOR ERROR: unable to open "
            "Safety queue\n");

        return ACC_FAILURE;
    }

    /*
     * Open Actuator -> Supervisor queue.
     */
    result =
        ipc_open_sender(
            &supervisor_queue,
            ACC_SUPERVISOR_QUEUE_NAME);

    if (result != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "ACTUATOR ERROR: unable to open "
            "Supervisor queue\n");

        ipc_close(&safety_queue);

        return ACC_FAILURE;
    }

    /*
     * Initialize PWM subsystem.
     */
    result =
        pwm_init();

    if (result != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "ACTUATOR ERROR: PWM initialization failed\n");

        ipc_close(&safety_queue);
        ipc_close(&supervisor_queue);

        return ACC_FAILURE;
    }

    /*
     * Initialize L298N.
     */
    result =
        l298n_init();

    if (result != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "ACTUATOR ERROR: L298N initialization failed\n");

        pwm_shutdown();

        ipc_close(&safety_queue);
        ipc_close(&supervisor_queue);

        return ACC_FAILURE;
    }

    /*
     * Always begin in the safe state.
     *
     * The actuator starts in a deterministic stopped state.
     * All subsequent output is sent to real QNX GPIO.
     */
    actuator_safe_stop();

    actuator_ipc_ready = 1;

    printf(
        "ACTUATOR: initialization complete\n");

    return ACC_SUCCESS;
}


/* ============================================================
 * ACTUATOR THREAD
 * ============================================================ */

void *actuator_thread(
    void *argument)
{
    actuator_context_t *context;

    ipc_message_t message;

    unsigned int receive_priority;

    task_timing_t timing;

    uint64_t next_release_ns;

    uint64_t receive_timestamp_ns;

    uint64_t ipc_latency_ns;

    int receive_result;

    int apply_result;

    control_command_t *command;


    context =
        (actuator_context_t *)argument;

    if (context == NULL)
    {
        fprintf(
            stderr,
            "ACTUATOR THREAD ERROR: "
            "NULL context\n");

        return NULL;
    }

    /*
     * Initialize task timing.
     */
    timing_init(&timing);

    timing_set_deadline(
        &timing,
        ACTUATOR_PERIOD_MS);

    timing_period_init(
        &next_release_ns,
        ACTUATOR_PERIOD_MS);

    printf(
        "ACTUATOR THREAD: started\n");


    while (context->running)
    {
        context->sequence++;

        memset(
            &message,
            0,
            sizeof(message));

        receive_priority = 0;

        /*
         * Receive only from Safety.
         */
        receive_result =
            ipc_receive(
                &safety_queue,
                &message,
                &receive_priority);

        receive_timestamp_ns =
            timing_now_ns();

        /*
         * IPC failure.
         */
        if (receive_result != ACC_SUCCESS)
        {
            fprintf(
                stderr,
                "ACTUATOR IPC ERROR: "
                "Safety receive failed\n");

            actuator_safe_stop();

            context->motor_status =
                MOTOR_FAULT;

            timing_stop(&timing);

            timing_period_wait(
                &next_release_ns,
                ACTUATOR_PERIOD_MS);

            continue;
        }

        /* Start execution timing only after a successful IPC receive. */
        if (timing_start(&timing) != ACC_SUCCESS)
        {
            fprintf(
                stderr,
                "ACTUATOR TIMING ERROR: timing_start failed\n"
            );
        }

        /*
         * Verify message type.
         */
        if (message.header.type !=
            IPC_MSG_ACTUATOR_COMMAND)
        {
            fprintf(
                stderr,
                "ACTUATOR IPC ERROR: "
                "unexpected message type %d\n",
                message.header.type);

            actuator_safe_stop();

            context->motor_status =
                MOTOR_FAULT;

            timing_stop(&timing);

            timing_period_wait(
                &next_release_ns,
                ACTUATOR_PERIOD_MS);

            continue;
        }

        /*
         * Measure IPC latency.
         */
        ipc_latency_ns =
            ipc_calculate_latency_ns(
                message.header.timestamp_ns,
                receive_timestamp_ns);

        /*
         * Actual payload field from ipc_types.h.
         */
        command =
            &message.payload.control;

        /*
         * Apply Safety-approved command.
         */
        if (command->state == ACC_STATE_EMERGENCY)
        {
            printf("[ACTUATOR]  >>> EMERGENCY BRAKE | PROPULSION DISABLED <<<\n");
        }
        else if (command->state == ACC_STATE_WARNING)
        {
            printf("[ACTUATOR]  >>> SLOW ZONE | REDUCED PROPULSION <<<\n");
        }
        else if (command->state == ACC_STATE_ACTIVE)
        {
            printf("[ACTUATOR]  >>> NORMAL ZONE | PROPULSION PERMITTED <<<\n");
        }

        apply_result =
            actuator_apply_command(command);

        if (apply_result == ACC_SUCCESS)
        {
            context->last_command =
                *command;

            context->last_command_timestamp_ns =
                command->timestamp_ns;

            if (command->actuator_enable != 0 &&
                command->direction ==
                    MOTOR_FORWARD &&
                command->duty_percent > 0.0)
            {
                context->motor_status =
                    MOTOR_ENABLED;
            }
            else
            {
                context->motor_status =
                    MOTOR_DISABLED;
            }
        }
        else
        {
            /*
             * Any failure leaves the motor stopped.
             */
            actuator_safe_stop();

            context->motor_status =
                MOTOR_FAULT;
        }

        /*
         * Stop timing measurement.
         */
        timing_stop(&timing);

        /*
         * Check Actuator deadline.
         */
        if (timing_deadline_missed(&timing))
        {
            fprintf(
                stderr,
                "ACTUATOR WARNING: deadline missed "
                "(%.3f ms)\n",
                timing_execution_time_ms(
                    &timing));

            actuator_safe_stop();

            context->motor_status =
                MOTOR_FAULT;
        }

        /*
         * Diagnostics.
         */
        printf(
            "[ACTUATOR]   | #%04d | MOTOR %-5s | DIR %-7s | DUTY %6.1f%% | IPC %6.3f ms | EXEC %6.3f ms\n",
            context->sequence,
            (context->motor_status == MOTOR_ENABLED) ? "RUN" :
            (context->motor_status == MOTOR_FAULT) ? "FAULT" : "STOP",
            (command->direction == MOTOR_FORWARD) ? "FORWARD" :
            (command->direction == MOTOR_REVERSE) ? "REVERSE" : "STOP",
            command->duty_percent,
            timing_ns_to_ms(ipc_latency_ns),
            timing_execution_time_ms(&timing)
        );
        /*
         * Maintain periodic execution.
         */
        timing_period_wait(
            &next_release_ns,
            ACTUATOR_PERIOD_MS);
    }

    /*
     * Final deterministic safe state.
     */
    actuator_safe_stop();

    context->motor_status =
        MOTOR_DISABLED;

    printf(
        "ACTUATOR THREAD: stopped\n");

    return NULL;
}


/* ============================================================
 * SHUTDOWN
 * ============================================================ */

int actuator_shutdown(
    actuator_context_t *context)
{
    if (context == NULL)
    {
        return ACC_FAILURE;
    }

    /*
     * Stop the worker thread.
     */
    context->running = 0;

    /*
     * Put motor into safe state.
     */
    actuator_safe_stop();

    /*
     * IMPORTANT:
     *
     * Both functions are declared void in the actual
     * uploaded headers.
     */
    l298n_shutdown();

    pwm_shutdown();

    /*
     * Close IPC queues.
     */
    if (actuator_ipc_ready)
    {
        ipc_close(
            &safety_queue);

        ipc_close(
            &supervisor_queue);

        actuator_ipc_ready = 0;
    }

    context->motor_status =
        MOTOR_DISABLED;

    printf(
        "ACTUATOR: shutdown complete\n");

    return ACC_SUCCESS;
}
