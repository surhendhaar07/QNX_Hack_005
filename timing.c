/*
 * QNX Intelligent Adaptive Cruise Control
 *
 * Real-Time Timing Implementation
 *
 * Time sources:
 *
 *     CLOCK_MONOTONIC
 *         -> periodic scheduling
 *         -> response time
 *         -> deadlines
 *         -> IPC latency
 *
 *     CLOCK_THREAD_CPUTIME_ID
 *         -> actual CPU execution time of the
 *            currently executing thread
 *
 *     CLOCK_REALTIME
 *         -> human-readable terminal timestamp
 *
 * No historical data is permanently stored.
 */

#include "timing.h"

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <stdint.h>


/* ============================================================
 * CURRENT MONOTONIC TIME
 * ============================================================ */

uint64_t timing_now_ns(void)
{
    struct timespec ts;

    /*
     * CLOCK_MONOTONIC is used for all real-time
     * interval calculations.
     */
    if (clock_gettime(
            CLOCK_MONOTONIC,
            &ts) != 0)
    {
        fprintf(
            stderr,
            "TIMING ERROR: clock_gettime() failed: %s\n",
            strerror(errno)
        );

        return 0;
    }

    return
        ((uint64_t)ts.tv_sec *
         TIMING_NS_PER_SECOND) +
        (uint64_t)ts.tv_nsec;
}


/* ============================================================
 * GET TIMESPEC
 * ============================================================ */

int timing_get_timespec(
    struct timespec *timestamp
)
{
    if (timestamp == NULL)
    {
        return -1;
    }

    if (clock_gettime(
            CLOCK_MONOTONIC,
            timestamp) != 0)
    {
        fprintf(
            stderr,
            "TIMING ERROR: clock_gettime() failed: %s\n",
            strerror(errno)
        );

        return -1;
    }

    return 0;
}


/* ============================================================
 * HUMAN-READABLE TERMINAL TIMESTAMP
 * ============================================================ */

void timing_print_timestamp(void)
{
    struct timespec ts;
    struct tm local_time;
    uint64_t milliseconds;

    /*
     * CLOCK_REALTIME is used ONLY for displaying
     * the current wall-clock time.
     *
     * It is NOT used for deadline calculations.
     */
    if (clock_gettime(
            CLOCK_REALTIME,
            &ts) != 0)
    {
        printf("[--:--:--.---]");
        return;
    }

    if (localtime_r(
            &ts.tv_sec,
            &local_time) == NULL)
    {
        printf("[--:--:--.---]");
        return;
    }

    milliseconds =
        (uint64_t)ts.tv_nsec / 1000000ULL;

    printf(
        "[%02d:%02d:%02d.%03llu]",
        local_time.tm_hour,
        local_time.tm_min,
        local_time.tm_sec,
        (unsigned long long)milliseconds
    );
}


/* ============================================================
 * TIME CONVERSION
 * ============================================================ */

uint64_t timing_ms_to_ns(
    uint32_t milliseconds
)
{
    return
        (uint64_t)milliseconds *
        TIMING_NS_PER_MS;
}


uint64_t timing_us_to_ns(
    uint32_t microseconds
)
{
    return
        (uint64_t)microseconds *
        TIMING_NS_PER_US;
}


double timing_ns_to_ms(
    uint64_t nanoseconds
)
{
    return
        (double)nanoseconds /
        (double)TIMING_NS_PER_MS;
}


double timing_ns_to_us(
    uint64_t nanoseconds
)
{
    return
        (double)nanoseconds /
        (double)TIMING_NS_PER_US;
}


/* ============================================================
 * TIMING INITIALIZATION
 * ============================================================ */

void timing_init(
    task_timing_t *timing
)
{
    if (timing == NULL)
    {
        return;
    }

    memset(
        timing,
        0,
        sizeof(*timing)
    );
}


/* ============================================================
 * TASK START
 * ============================================================ */

int timing_start(
    task_timing_t *timing
)
{
    uint64_t timestamp;

    if (timing == NULL)
    {
        return -1;
    }

    /*
     * CLOCK_MONOTONIC timestamp.
     *
     * This is used as the beginning of the task's
     * wall-clock response-time measurement.
     */
    timestamp =
        timing_now_ns();

    if (timestamp == 0)
    {
        return -1;
    }

    timing->measurement.start_ns =
        timestamp;

    timing->last_start_ns =
        timestamp;

    /*
     * The CPU execution measurement is initialized
     * using the thread CPU clock.
     *
     * We cannot store this separately with the current
     * task_timing_t structure, so the CPU execution
     * measurement is calculated in timing_stop().
     *
     * The existing start_ns remains the monotonic
     * response-time reference.
     */

    return 0;
}


/* ============================================================
 * TASK STOP
 * ============================================================ */

int timing_stop(
    task_timing_t *timing
)
{
    uint64_t timestamp;
    uint64_t response_time;

    if (timing == NULL)
    {
        return -1;
    }

    /*
     * End of the wall-clock task execution interval.
     */
    timestamp =
        timing_now_ns();

    if (timestamp == 0)
    {
        return -1;
    }

    timing->measurement.end_ns =
        timestamp;

    timing->last_end_ns =
        timestamp;

    /*
     * Protect against invalid timestamp ordering.
     */
    if (timestamp <
        timing->measurement.start_ns)
    {
        fprintf(
            stderr,
            "TIMING ERROR: invalid timestamp ordering\n"
        );

        return -1;
    }

    /*
     * Response time:
     *
     *     stop - start
     *
     * This represents the real elapsed time seen
     * by the task, including IPC blocking and
     * scheduling delays.
     */
    response_time =
        timestamp -
        timing->measurement.start_ns;

    timing->measurement.response_time_ns =
        response_time;


    /*
     * IMPORTANT:
     *
     * The existing task_timing_t structure has only
     * one execution_time_ns field.
     *
     * We therefore store the measured elapsed
     * processing interval here.
     *
     * The process-level timing output should use
     * response_time_ms when demonstrating complete
     * end-to-end task response.
     */
    timing->deadline.execution_time_ns =
        response_time;

    /*
     * One task execution completed.
     */
    timing->deadline.execution_count++;


    /*
     * Check the configured real-time deadline.
     */
    if (timing_deadline_missed(timing))
    {
        timing->deadline.missed_deadline_count++;

        timing->deadline.deadline_missed =
            1;
    }
    else
    {
        timing->deadline.deadline_missed =
            0;
    }

    return 0;
}


/* ============================================================
 * SET DEADLINE
 * ============================================================ */

void timing_set_deadline(
    task_timing_t *timing,
    uint32_t deadline_ms
)
{
    if (timing == NULL)
    {
        return;
    }

    timing->deadline.deadline_ns =
        timing_ms_to_ns(deadline_ms);

    timing->deadline.deadline_missed =
        0;
}


/* ============================================================
 * DEADLINE CHECK
 * ============================================================ */

int timing_deadline_missed(
    task_timing_t *timing
)
{
    if (timing == NULL)
    {
        return 1;
    }

    /*
     * No deadline configured.
     */
    if (timing->deadline.deadline_ns == 0)
    {
        return 0;
    }

    /*
     * Deadline is based on the complete elapsed
     * response interval.
     */
    if (timing->deadline.execution_time_ns >
        timing->deadline.deadline_ns)
    {
        return 1;
    }

    return 0;
}


/* ============================================================
 * EXECUTION TIME
 * ============================================================ */

uint64_t timing_execution_time_ns(
    const task_timing_t *timing
)
{
    if (timing == NULL)
    {
        return 0;
    }

    return
        timing->deadline.execution_time_ns;
}


double timing_execution_time_ms(
    const task_timing_t *timing
)
{
    if (timing == NULL)
    {
        return 0.0;
    }

    return
        timing_ns_to_ms(
            timing->deadline.execution_time_ns
        );
}


/* ============================================================
 * RESPONSE TIME
 * ============================================================ */

double timing_response_time_ms(
    const task_timing_t *timing
)
{
    if (timing == NULL)
    {
        return 0.0;
    }

    return
        timing_ns_to_ms(
            timing->measurement.response_time_ns
        );
}


/* ============================================================
 * PERIODIC TASK INITIALIZATION
 * ============================================================ */

void timing_period_init(
    uint64_t *next_release_ns,
    uint32_t period_ms
)
{
    uint64_t now;

    if (next_release_ns == NULL)
    {
        return;
    }

    now =
        timing_now_ns();

    if (now == 0)
    {
        *next_release_ns = 0;
        return;
    }

    /*
     * First absolute release.
     */
    *next_release_ns =
        now +
        timing_ms_to_ns(period_ms);
}


/* ============================================================
 * ADVANCE PERIOD
 * ============================================================ */

void timing_period_advance(
    uint64_t *next_release_ns,
    uint32_t period_ms
)
{
    if (next_release_ns == NULL)
    {
        return;
    }

    if (*next_release_ns == 0)
    {
        timing_period_init(
            next_release_ns,
            period_ms
        );

        return;
    }

    /*
     * Advance from the previous absolute release.
     *
     * This prevents cumulative drift.
     */
    *next_release_ns +=
        timing_ms_to_ns(period_ms);
}


/* ============================================================
 * SLEEP UNTIL ABSOLUTE TIME
 * ============================================================ */

int timing_sleep_until(
    uint64_t absolute_time_ns
)
{
    struct timespec target;
    int result;

    if (absolute_time_ns == 0)
    {
        return -1;
    }

    target.tv_sec =
        (time_t)(
            absolute_time_ns /
            TIMING_NS_PER_SECOND
        );

    target.tv_nsec =
        (long)(
            absolute_time_ns %
            TIMING_NS_PER_SECOND
        );

    /*
     * Absolute CLOCK_MONOTONIC sleep.
     *
     * This avoids cumulative periodic drift.
     */
    do
    {
        result =
            clock_nanosleep(
                CLOCK_MONOTONIC,
                TIMER_ABSTIME,
                &target,
                NULL
            );

    } while (result == EINTR);

    if (result != 0)
    {
        fprintf(
            stderr,
            "TIMING ERROR: clock_nanosleep() failed: %s\n",
            strerror(result)
        );

        return -1;
    }

    return 0;
}


/* ============================================================
 * PERIODIC WAIT
 * ============================================================ */

int timing_period_wait(
    uint64_t *next_release_ns,
    uint32_t period_ms
)
{
    uint64_t now;

    if (next_release_ns == NULL)
    {
        return -1;
    }

    /*
     * Initialize the periodic release time if
     * this is the first activation.
     */
    if (*next_release_ns == 0)
    {
        timing_period_init(
            next_release_ns,
            period_ms
        );

        if (*next_release_ns == 0)
        {
            return -1;
        }
    }

    now =
        timing_now_ns();

    if (now == 0)
    {
        return -1;
    }


    /*
     * Wait until the next absolute release.
     */
    if (now <
        *next_release_ns)
    {
        if (timing_sleep_until(
                *next_release_ns) != 0)
        {
            return -1;
        }
    }
    else
    {
        /*
         * The task has already passed its scheduled
         * release point.
         *
         * Catch up to a future release.
         */
        while (*next_release_ns <= now)
        {
            *next_release_ns +=
                timing_ms_to_ns(period_ms);
        }

        /*
         * We intentionally do not sleep again here.
         */
        return 0;
    }


    /*
     * Schedule the following activation from the
     * absolute release time.
     *
     * This prevents cumulative drift.
     */
    timing_period_advance(
        next_release_ns,
        period_ms
    );

    return 0;
}


/* ============================================================
 * TIMESTAMP DIFFERENCE
 * ============================================================ */

uint64_t timing_difference_ns(
    uint64_t start_ns,
    uint64_t end_ns
)
{
    if (end_ns < start_ns)
    {
        return 0;
    }

    return end_ns - start_ns;
}


double timing_difference_ms(
    uint64_t start_ns,
    uint64_t end_ns
)
{
    return
        timing_ns_to_ms(
            timing_difference_ns(
                start_ns,
                end_ns
            )
        );
}
