#ifndef QNX_PRIORITY_H
#define QNX_PRIORITY_H

/*
 * QNX Intelligent Adaptive Cruise Control
 *
 * QNX/POSIX Real-Time Priority Interface
 *
 * This module provides:
 *
 *     - SCHED_FIFO configuration
 *     - Thread priority configuration
 *     - Priority verification
 *     - Priority reporting
 *
 * The actual ACC priority values are defined in:
 *
 *     common/acc_config.h
 */

#include <pthread.h>


/* ============================================================
 * SCHEDULING POLICY
 * ============================================================ */

/*
 * Configure a thread for fixed-priority FIFO scheduling.
 *
 * Returns:
 *
 *      0 = success
 *     -1 = failure
 */
int qnx_priority_set_fifo(
    pthread_t thread,
    int priority
);


/* ============================================================
 * PRIORITY CONFIGURATION
 * ============================================================ */

/*
 * Configure a thread using the supplied scheduling policy
 * and priority.
 *
 * policy:
 *     SCHED_FIFO
 *     SCHED_RR
 *     or another POSIX-supported policy
 *
 * priority:
 *     QNX thread priority
 */
int qnx_priority_set(
    pthread_t thread,
    int policy,
    int priority
);


/* ============================================================
 * PRIORITY QUERY
 * ============================================================ */

/*
 * Obtain the current scheduling policy and priority
 * of a thread.
 */
int qnx_priority_get(
    pthread_t thread,
    int *policy,
    int *priority
);


/* ============================================================
 * PRIORITY VALIDATION
 * ============================================================ */

/*
 * Check whether a requested priority is valid for the
 * application.
 *
 * Returns:
 *
 *      0 = valid
 *     -1 = invalid
 */
int qnx_priority_validate(
    int priority
);


/*
 * Check whether the scheduling policy is supported
 * by the application.
 *
 * Returns:
 *
 *      0 = supported
 *     -1 = unsupported
 */
int qnx_priority_validate_policy(
    int policy
);


/* ============================================================
 * PRIORITY REPORTING
 * ============================================================ */

/*
 * Print the current scheduling policy and priority
 * of a thread.
 */
void qnx_priority_print(
    const char *task_name,
    pthread_t thread
);


/*
 * Convert a scheduling policy into readable text.
 */
const char *qnx_priority_policy_name(
    int policy
);


/* ============================================================
 * ACC TASK PRIORITIES
 * ============================================================ */

/*
 * Configure the standard ACC real-time priority hierarchy.
 *
 * These functions will be used by the individual process
 * main functions.
 */

int qnx_priority_configure_radar(
    pthread_t thread
);

int qnx_priority_configure_tracking(
    pthread_t thread
);

int qnx_priority_configure_controller(
    pthread_t thread
);

int qnx_priority_configure_actuator(
    pthread_t thread
);

int qnx_priority_configure_safety(
    pthread_t thread
);

int qnx_priority_configure_supervisor(
    pthread_t thread
);


#endif /* QNX_PRIORITY_H */
