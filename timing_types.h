#ifndef TIMING_TYPES_H
#define TIMING_TYPES_H

#include <stdint.h>

/*
 * Timestamp information.
 *
 * All timestamps are based on CLOCK_MONOTONIC.
 */
typedef struct
{
    uint64_t start_ns;
    uint64_t end_ns;
    uint64_t response_time_ns;

} timing_measurement_t;


/*
 * IPC timing information.
 *
 * sender_timestamp_ns:
 *     timestamp when sender generated the message
 *
 * receiver_timestamp_ns:
 *     timestamp when receiver received the message
 *
 * ipc_latency_ns:
 *     receiver_timestamp - sender_timestamp
 */
typedef struct
{
    uint64_t sender_timestamp_ns;
    uint64_t receiver_timestamp_ns;
    uint64_t ipc_latency_ns;

} ipc_timing_t;


/*
 * Deadline monitoring information.
 */
typedef struct
{
    uint64_t execution_time_ns;
    uint64_t deadline_ns;

    int deadline_missed;

    uint32_t execution_count;
    uint32_t missed_deadline_count;

} deadline_status_t;


/*
 * Complete timing information for one real-time task.
 */
typedef struct
{
    timing_measurement_t measurement;

    deadline_status_t deadline;

    uint64_t last_start_ns;
    uint64_t last_end_ns;

} task_timing_t;


#endif /* TIMING_TYPES_H */
