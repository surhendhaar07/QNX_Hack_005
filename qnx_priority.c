/*
 * QNX Intelligent Adaptive Cruise Control
 *
 * QNX/POSIX Real-Time Priority Implementation
 *
 * Scheduling model:
 *
 *     SCHED_FIFO
 *
 * Priority hierarchy:
 *
 *     Supervisor  = 45
 *     Safety      = 40
 *     Actuator    = 35
 *     Controller  = 30
 *     Tracking    = 25
 *     Radar       = 20
 *
 * Higher numerical priority is used for the more
 * safety-critical application tasks.
 */

#include "qnx_priority.h"

#include "acc_config.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sched.h>


/* ============================================================
 * INTERNAL ERROR REPORTING
 * ============================================================ */

static void qnx_priority_print_error(
    const char *operation,
    int error_code
)
{
    if (operation == NULL)
    {
        return;
    }

    fprintf(
        stderr,
        "PRIORITY ERROR: %s: %s\n",
        operation,
        strerror(error_code)
    );
}


/* ============================================================
 * POLICY NAME
 * ============================================================ */

const char *qnx_priority_policy_name(
    int policy
)
{
    switch (policy)
    {
        case SCHED_FIFO:
            return "SCHED_FIFO";

        case SCHED_RR:
            return "SCHED_RR";

#ifdef SCHED_SPORADIC
        case SCHED_SPORADIC:
            return "SCHED_SPORADIC";
#endif

        default:
            return "UNKNOWN";
    }
}


/* ============================================================
 * POLICY VALIDATION
 * ============================================================ */

int qnx_priority_validate_policy(
    int policy
)
{
    /*
     * The ACC real-time task design uses FIFO scheduling.
     *
     * SCHED_RR is recognized because it is a POSIX scheduling
     * policy, but the ACC configuration itself will use FIFO.
     */
    if (policy == SCHED_FIFO)
    {
        return ACC_SUCCESS;
    }

    if (policy == SCHED_RR)
    {
        return ACC_SUCCESS;
    }

#ifdef SCHED_SPORADIC

    if (policy == SCHED_SPORADIC)
    {
        return ACC_SUCCESS;
    }

#endif

    fprintf(
        stderr,
        "PRIORITY ERROR: unsupported scheduling policy: %d\n",
        policy
    );

    return ACC_FAILURE;
}


/* ============================================================
 * PRIORITY VALIDATION
 * ============================================================ */

int qnx_priority_validate(
    int priority
)
{
    int minimum;
    int maximum;

    /*
     * Obtain the valid priority range for SCHED_FIFO
     * from the target system.
     *
     * This avoids hard-coding a host-specific range.
     */
    minimum = sched_get_priority_min(SCHED_FIFO);

    if (minimum == -1)
    {
        fprintf(
            stderr,
            "PRIORITY ERROR: sched_get_priority_min() failed: %s\n",
            strerror(errno)
        );

        return ACC_FAILURE;
    }

    maximum = sched_get_priority_max(SCHED_FIFO);

    if (maximum == -1)
    {
        fprintf(
            stderr,
            "PRIORITY ERROR: sched_get_priority_max() failed: %s\n",
            strerror(errno)
        );

        return ACC_FAILURE;
    }

    if (priority < minimum ||
        priority > maximum)
    {
        fprintf(
            stderr,
            "PRIORITY ERROR: priority %d outside "
            "SCHED_FIFO range %d-%d\n",
            priority,
            minimum,
            maximum
        );

        return ACC_FAILURE;
    }

    return ACC_SUCCESS;
}


/* ============================================================
 * SET PRIORITY
 * ============================================================ */

int qnx_priority_set(
    pthread_t thread,
    int policy,
    int priority
)
{
    struct sched_param param;
    int result;

    /*
     * Validate scheduling policy first.
     */
    if (qnx_priority_validate_policy(
            policy) != ACC_SUCCESS)
    {
        return ACC_FAILURE;
    }

    /*
     * Validate requested priority.
     */
    if (qnx_priority_validate(
            priority) != ACC_SUCCESS)
    {
        return ACC_FAILURE;
    }

    memset(
        &param,
        0,
        sizeof(param)
    );

    param.sched_priority = priority;

    /*
     * pthread_setschedparam() is a POSIX API supported by QNX.
     *
     * The calling process must have the required scheduling
     * privileges on the target system.
     */
    result = pthread_setschedparam(
        thread,
        policy,
        &param
    );

    if (result != 0)
    {
        qnx_priority_print_error(
            "pthread_setschedparam()",
            result
        );

        return ACC_FAILURE;
    }

    return ACC_SUCCESS;
}


/* ============================================================
 * SET FIFO PRIORITY
 * ============================================================ */

int qnx_priority_set_fifo(
    pthread_t thread,
    int priority
)
{
    return qnx_priority_set(
        thread,
        SCHED_FIFO,
        priority
    );
}


/* ============================================================
 * GET THREAD PRIORITY
 * ============================================================ */

int qnx_priority_get(
    pthread_t thread,
    int *policy,
    int *priority
)
{
    struct sched_param param;
    int result;
    int current_policy;

    if (policy == NULL ||
        priority == NULL)
    {
        fprintf(
            stderr,
            "PRIORITY ERROR: invalid output arguments\n"
        );

        return ACC_FAILURE;
    }

    memset(
        &param,
        0,
        sizeof(param)
    );

    result = pthread_getschedparam(
        thread,
        &current_policy,
        &param
    );

    if (result != 0)
    {
        qnx_priority_print_error(
            "pthread_getschedparam()",
            result
        );

        return ACC_FAILURE;
    }

    *policy = current_policy;

    *priority = param.sched_priority;

    return ACC_SUCCESS;
}


/* ============================================================
 * PRINT THREAD PRIORITY
 * ============================================================ */

void qnx_priority_print(
    const char *task_name,
    pthread_t thread
)
{
    int policy;
    int priority;

    if (task_name == NULL)
    {
        task_name = "UNKNOWN";
    }

    if (qnx_priority_get(
            thread,
            &policy,
            &priority) != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "PRIORITY: unable to query %s\n",
            task_name
        );

        return;
    }

    printf(
        "PRIORITY: %-12s policy=%s priority=%d\n",
        task_name,
        qnx_priority_policy_name(policy),
        priority
    );
}


/* ============================================================
 * RADAR PRIORITY
 * ============================================================ */

int qnx_priority_configure_radar(
    pthread_t thread
)
{
    return qnx_priority_set_fifo(
        thread,
        RADAR_PRIORITY
    );
}


/* ============================================================
 * TRACKING PRIORITY
 * ============================================================ */

int qnx_priority_configure_tracking(
    pthread_t thread
)
{
    return qnx_priority_set_fifo(
        thread,
        TRACKING_PRIORITY
    );
}


/* ============================================================
 * CONTROLLER PRIORITY
 * ============================================================ */

int qnx_priority_configure_controller(
    pthread_t thread
)
{
    return qnx_priority_set_fifo(
        thread,
        CONTROLLER_PRIORITY
    );
}


/* ============================================================
 * ACTUATOR PRIORITY
 * ============================================================ */

int qnx_priority_configure_actuator(
    pthread_t thread
)
{
    return qnx_priority_set_fifo(
        thread,
        ACTUATOR_PRIORITY
    );
}


/* ============================================================
 * SAFETY PRIORITY
 * ============================================================ */

int qnx_priority_configure_safety(
    pthread_t thread
)
{
    return qnx_priority_set_fifo(
        thread,
        SAFETY_PRIORITY
    );
}


/* ============================================================
 * SUPERVISOR PRIORITY
 * ============================================================ */

int qnx_priority_configure_supervisor(
    pthread_t thread
)
{
    return qnx_priority_set_fifo(
        thread,
        SUPERVISOR_PRIORITY
    );
}
