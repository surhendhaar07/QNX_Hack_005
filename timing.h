#ifndef ACC_TIMING_H
#define ACC_TIMING_H

/*
 * QNX Intelligent Adaptive Cruise Control
 *
 * Real-Time Timing Interface
 *
 * Timing source:
 *     CLOCK_MONOTONIC
 *
 * Used for:
 *     - task response time
 *     - execution time
 *     - deadline monitoring
 *     - IPC latency support
 *     - periodic task scheduling
 */

#include <stdint.h>
#include <time.h>

#include "timing_types.h"


/* ============================================================
 * TIME CONVERSION
 * ============================================================ */

/*
 * Nanoseconds per unit.
 */
#define TIMING_NS_PER_US       1000ULL
#define TIMING_NS_PER_MS       1000000ULL
#define TIMING_NS_PER_SECOND   1000000000ULL


/* ============================================================
 * CURRENT TIME
 * ============================================================ */

/*
 * Return current CLOCK_MONOTONIC time in nanoseconds.
 *
 * Returns:
 *     timestamp in ns
 *
 * Returns 0 if clock_gettime() fails.
 */
uint64_t timing_now_ns(void);


/*
 * Get CLOCK_MONOTONIC as a timespec.
 *
 * Returns:
 *     0  = success
 *    -1  = failure
 */
int timing_get_timespec(
    struct timespec *timestamp
);


/* ============================================================
 * TIME CONVERSION
 * ============================================================ */

/*
 * Convert milliseconds to nanoseconds.
 */
uint64_t timing_ms_to_ns(
    uint32_t milliseconds
);


/*
 * Convert microseconds to nanoseconds.
 */
uint64_t timing_us_to_ns(
    uint32_t microseconds
);


/*
 * Convert nanoseconds to milliseconds.
 */
double timing_ns_to_ms(
    uint64_t nanoseconds
);


/*
 * Convert nanoseconds to microseconds.
 */
double timing_ns_to_us(
    uint64_t nanoseconds
);


/* ============================================================
 * TASK TIMING
 * ============================================================ */

/*
 * Initialize a task timing structure.
 */
void timing_init(
    task_timing_t *timing
);


/*
 * Record task start time.
 */
int timing_start(
    task_timing_t *timing
);


/*
 * Record task end time.
 *
 * Calculates:
 *
 *     response_time_ns
 *
 * and
 *
 *     execution_time_ns
 */
int timing_stop(
    task_timing_t *timing
);


/* ============================================================
 * DEADLINE MONITORING
 * ============================================================ */

/*
 * Set task deadline in milliseconds.
 */
void timing_set_deadline(
    task_timing_t *timing,
    uint32_t deadline_ms
);


/*
 * Check whether the most recent execution exceeded
 * the configured deadline.
 *
 * Returns:
 *
 *     0 = deadline met
 *     1 = deadline missed
 */
int timing_deadline_missed(
    task_timing_t *timing
);


/*
 * Return the execution time of the most recent task
 * execution in nanoseconds.
 */
uint64_t timing_execution_time_ns(
    const task_timing_t *timing
);


/*
 * Return the execution time of the most recent task
 * execution in milliseconds.
 */
double timing_execution_time_ms(
    const task_timing_t *timing
);


/*
 * Return the response time of the most recent task
 * execution in milliseconds.
 */
double timing_response_time_ms(
    const task_timing_t *timing
);


/* ============================================================
 * PERIODIC EXECUTION
 * ============================================================ */

/*
 * Initialize the next absolute release time.
 *
 * start_time_ns:
 *     initial reference time
 *
 * period_ms:
 *     task period
 */
void timing_period_init(
    uint64_t *next_release_ns,
    uint32_t period_ms
);


/*
 * Advance the next absolute release time by one period.
 */
void timing_period_advance(
    uint64_t *next_release_ns,
    uint32_t period_ms
);


/*
 * Sleep until the specified absolute CLOCK_MONOTONIC time.
 *
 * This avoids repeatedly sleeping for a relative period and
 * accumulating timing drift.
 */
int timing_sleep_until(
    uint64_t absolute_time_ns
);


/*
 * Perform one periodic wait and advance the release time.
 *
 * Returns:
 *
 *     0  = normal
 *    -1  = timing error
 */
int timing_period_wait(
    uint64_t *next_release_ns,
    uint32_t period_ms
);


/* ============================================================
 * TIMESTAMP DIFFERENCE
 * ============================================================ */

/*
 * Calculate difference between two timestamps.
 *
 * Returns:
 *
 *     end - start
 *
 * If end < start, returns 0.
 */
uint64_t timing_difference_ns(
    uint64_t start_ns,
    uint64_t end_ns
);


/*
 * Calculate difference in milliseconds.
 */
double timing_difference_ms(
    uint64_t start_ns,
    uint64_t end_ns
);


#endif /* ACC_TIMING_H */
