#ifndef SUPERVISOR_H
#define SUPERVISOR_H

#include "../common/acc_types.h"
#include "../common/ipc.h"
#include "../common/timing_types.h"

typedef struct
{
    /* IPC receiver for Safety and Actuator */
    ipc_queue_t supervisor_queue;

    /* Supervisor system status */
    supervisor_status_t status;

    /* Timing information */
    task_timing_t timing;

    /* Message/cycle sequence */
    uint32_t sequence;

    /* Last Safety status received through IPC */
    uint64_t last_safety_status_ns;

    /* Thread/process control */
    volatile int running;

} supervisor_context_t;


/* Initialization */
int supervisor_init(supervisor_context_t *context);

/* Worker thread */
void *supervisor_thread(void *arg);

/* Execute one monitoring cycle */
int supervisor_execute_cycle(supervisor_context_t *context);

/* Shutdown */
void supervisor_shutdown(supervisor_context_t *context);

#endif
