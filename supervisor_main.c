#include "supervisor.h"
#include "../common/acc_config.h"
#include "../common/ipc_types.h"
#include "../common/ipc.h"
#include "../common/timing.h"
#include "../common/qnx_affinity.h"
#include "../common/qnx_priority.h"

#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <stdint.h>

/*
 * Supervisor only monitors.  It never generates synthetic process
 * health data and never commands the actuator directly.
 */

int supervisor_init(supervisor_context_t *context)
{
    if (context == NULL)
        return ACC_FAILURE;

    memset(context, 0, sizeof(*context));
    context->running = 1;
    context->sequence = 0;
    timing_init(&context->timing);

    if (ipc_open_receiver(&context->supervisor_queue,
                          ACC_SUPERVISOR_QUEUE_NAME) != ACC_SUCCESS)
    {
        fprintf(stderr, "[SUPERVISOR] ERROR: unable to open Supervisor queue\n");
        return ACC_FAILURE;
    }

    context->status.system_fault = 0;
    context->status.safe_stop_requested = 0;
    context->status.timestamp_ns = timing_now_ns();
    context->last_safety_status_ns = 0;

    printf("[SUPERVISOR] Process initialized\n");
    printf("[SUPERVISOR] Priority: %d\n", SUPERVISOR_PRIORITY);
    printf("[SUPERVISOR] Period: %d ms\n", SUPERVISOR_PERIOD_MS);

    return ACC_SUCCESS;
}

void supervisor_shutdown(supervisor_context_t *context)
{
    if (context == NULL)
        return;

    context->running = 0;
    ipc_close(&context->supervisor_queue);
    printf("[SUPERVISOR] Process shutdown\n");
}

/*
 * Current project does not have a separate process-heartbeat IPC source.
 * Therefore do not invent health values.  Returning success means only
 * that this local supervisor health check has no detected fault.
 */
static int supervisor_check_process_health(supervisor_context_t *context)
{
    if (context == NULL)
        return ACC_FAILURE;

    return ACC_SUCCESS;
}

static void supervisor_receive_status(supervisor_context_t *context)
{
    ipc_message_t message;
    unsigned int priority;

    if (context == NULL)
        return;

    while (ipc_receive_nonblocking(&context->supervisor_queue,
                                   &message, &priority) == ACC_SUCCESS)
    {
        if (message.header.type != IPC_MSG_SAFETY_STATUS)
            continue;

        context->status.timestamp_ns =
            message.payload.safety.timestamp_ns;
        context->last_safety_status_ns = timing_now_ns();

        context->status.system_fault =
            (message.payload.safety.fault_active ||
             message.payload.safety.state == ACC_STATE_FAULT) ? 1 : 0;

        context->status.safe_stop_requested =
            (message.payload.safety.emergency_stop ||
             message.payload.safety.state == ACC_STATE_EMERGENCY ||
             message.payload.safety.state == ACC_STATE_FAULT) ? 1 : 0;

        printf("[SUPERVISOR] | SAFETY STATE %d | FAULT %s | E-STOP %s | DEADLINE %s | SYSTEM %s | SAFE STOP %s\n",
               message.payload.safety.state,
               message.payload.safety.fault_active ? "YES" : "NO",
               message.payload.safety.emergency_stop ? "YES" : "NO",
               message.payload.safety.controller_deadline_missed ? "MISS" : "OK",
               context->status.system_fault ? "FAULT" : "OK",
               context->status.safe_stop_requested ? "YES" : "NO");
    }
}

int supervisor_execute_cycle(supervisor_context_t *context)
{
    int health_result;

    if (context == NULL)
        return ACC_FAILURE;

    if (timing_start(&context->timing) != ACC_SUCCESS)
        return ACC_FAILURE;

    supervisor_receive_status(context);

    health_result = supervisor_check_process_health(context);

    if (health_result != ACC_SUCCESS)
    {
        context->status.system_fault = 1;
        context->status.safe_stop_requested = 1;
        printf("[SUPERVISOR] Process health fault detected\n");
    }

    context->status.timestamp_ns = timing_now_ns();

    if (context->last_safety_status_ns != 0 &&
        context->status.timestamp_ns >= context->last_safety_status_ns &&
        (context->status.timestamp_ns - context->last_safety_status_ns) >
        timing_ms_to_ns(SUPERVISOR_SAFETY_STATUS_TIMEOUT_MS))
    {
        context->status.system_fault = 1;
        context->status.safe_stop_requested = 1;
        printf("[SUPERVISOR] Safety status timeout - SAFE STOP requested\n");
    }

    context->sequence++;

    if (timing_stop(&context->timing) != ACC_SUCCESS)
        return ACC_FAILURE;

    printf("[SUPERVISOR] | SYSTEM %s | SAFE STOP %s | RESPONSE %6.3f ms\n",
           context->status.system_fault ? "FAULT" : "OK",
           context->status.safe_stop_requested ? "YES" : "NO",
           timing_response_time_ms(&context->timing));

    return ACC_SUCCESS;
}

void *supervisor_thread(void *arg)
{
    supervisor_context_t *context = (supervisor_context_t *)arg;
    uint64_t next_release_ns;

    if (context == NULL)
        return NULL;

    if (qnx_affinity_set_current(ACC_DEFAULT_RUNMASK) != 0)
        fprintf(stderr, "[SUPERVISOR] CPU affinity not applied\n");

    timing_period_init(&next_release_ns, SUPERVISOR_PERIOD_MS);

    printf("[SUPERVISOR] Worker thread started\n");

    while (context->running)
    {
        if (supervisor_execute_cycle(context) != ACC_SUCCESS)
        {
            fprintf(stderr, "[SUPERVISOR] Supervisor cycle failed\n");
            context->status.system_fault = 1;
            context->status.safe_stop_requested = 1;
        }

        timing_period_wait(&next_release_ns, SUPERVISOR_PERIOD_MS);
    }

    printf("[SUPERVISOR] Worker thread stopped\n");
    return NULL;
}

int main(void)
{
    supervisor_context_t context;
    pthread_t thread_id;

    if (supervisor_init(&context) != ACC_SUCCESS)
    {
        fprintf(stderr, "[SUPERVISOR_MAIN] Supervisor initialization failed\n");
        return 1;
    }

    if (pthread_create(&thread_id, NULL, supervisor_thread, &context) != 0)
    {
        fprintf(stderr, "[SUPERVISOR_MAIN] Failed to create supervisor thread\n");
        supervisor_shutdown(&context);
        return 1;
    }

    if (qnx_priority_configure_supervisor(thread_id) != 0)
        fprintf(stderr, "[SUPERVISOR_MAIN] Failed to configure supervisor priority\n");

    printf("[SUPERVISOR_MAIN] Supervisor process running\n");

    pthread_join(thread_id, NULL);

    supervisor_shutdown(&context);

    printf("[SUPERVISOR_MAIN] Supervisor process terminated\n");
    return 0;
}
