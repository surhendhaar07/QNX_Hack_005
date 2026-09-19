/*
 * QNX Intelligent Adaptive Cruise Control
 *
 * Tracking Process
 *
 * Responsibilities:
 *
 *     - Receive Radar data through POSIX IPC
 *     - Validate sensor status
 *     - Calculate relative target speed
 *     - Detect stale sensor data
 *     - Measure IPC latency
 *     - Measure Tracking execution time
 *     - Send tracking data to Controller
 *
 * No simulated or random sensor values are generated.
 */

#include "tracking.h"

#include "../common/acc_config.h"
#include "../common/ipc.h"
#include "../common/ipc_types.h"
#include "../common/timing.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>


/* ============================================================
 * IPC QUEUES
 * ============================================================ */

/*
 * Radar -> Tracking
 */
static ipc_queue_t radar_queue;


/*
 * Tracking -> Controller
 */
static ipc_queue_t controller_queue;


/*
 * IPC initialization status.
 */
static int tracking_ipc_ready = 0;


/* ============================================================
 * TRACKING INITIALIZATION
 * ============================================================ */

int tracking_init(
    tracking_context_t *context
)
{
    int result;

    if (context == NULL)
    {
        fprintf(
            stderr,
            "TRACKING ERROR: NULL context\n"
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

    context->previous_distance_cm = 0.0;

    context->previous_timestamp_ns = 0;

    context->sensor_status = SENSOR_TIMEOUT;

    context->last_radar_timestamp_ns = 0;


    /* --------------------------------------------------------
     * Validate IPC message size
     * -------------------------------------------------------- */

    result =
        ipc_validate_message_size();

    if (result != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "TRACKING ERROR: IPC message size validation failed\n"
        );

        return ACC_FAILURE;
    }


    /* --------------------------------------------------------
     * Open Radar receive queue
     * -------------------------------------------------------- */

    result =
        ipc_open_receiver(
            &radar_queue,
            ACC_RADAR_QUEUE_NAME
        );

    if (result != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "TRACKING ERROR: unable to open Radar queue\n"
        );

        return ACC_FAILURE;
    }


    /* --------------------------------------------------------
     * Open Controller send queue
     * -------------------------------------------------------- */

    result =
        ipc_open_sender(
            &controller_queue,
            ACC_TRACKING_QUEUE_NAME
        );

    if (result != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "TRACKING ERROR: unable to open Controller queue\n"
        );

        ipc_close(
            &radar_queue
        );

        return ACC_FAILURE;
    }


    tracking_ipc_ready = 1;


    printf(
        "TRACKING: initialization complete\n"
    );

    return ACC_SUCCESS;
}


/* ============================================================
 * TRACKING DATA CALCULATION
 * ============================================================ */

int tracking_process_radar(
    tracking_context_t *context,
    const radar_data_t *radar,
    tracking_data_t *tracking
)
{
    uint64_t current_timestamp_ns;

    double dt_seconds;
    double relative_speed_cm_s;


    if (context == NULL ||
        radar == NULL ||
        tracking == NULL)
    {
        return ACC_FAILURE;
    }


    /* --------------------------------------------------------
     * Clear output
     * -------------------------------------------------------- */

    memset(
        tracking,
        0,
        sizeof(*tracking)
    );


    tracking->sequence =
        radar->sequence;


    /* --------------------------------------------------------
     * Capture current timestamp
     * -------------------------------------------------------- */

    current_timestamp_ns =
        timing_now_ns();

    if (current_timestamp_ns == 0)
    {
        tracking->status =
            SENSOR_INVALID;

        return ACC_FAILURE;
    }


    /* --------------------------------------------------------
     * Check sensor status
     * -------------------------------------------------------- */

    if (radar->status != SENSOR_OK)
    {
        /*
         * Do not calculate relative speed from invalid
         * sensor information.
         */
        tracking->distance_cm =
            0.0;

        tracking->relative_speed_cm_s =
            0.0;

        tracking->timestamp_ns =
            current_timestamp_ns;

        tracking->status =
            radar->status;

        return ACC_FAILURE;
    }


    /* --------------------------------------------------------
     * Validate distance
     * -------------------------------------------------------- */

    if (radar->distance_cm <
        ACC_MIN_VALID_DISTANCE_CM ||
        radar->distance_cm >
        ACC_MAX_VALID_DISTANCE_CM)
    {
        tracking->distance_cm =
            0.0;

        tracking->relative_speed_cm_s =
            0.0;

        tracking->timestamp_ns =
            current_timestamp_ns;

        tracking->status =
            SENSOR_INVALID;

        return ACC_FAILURE;
    }


    /* --------------------------------------------------------
     * Check for stale Radar data
     * -------------------------------------------------------- */

    if (context->last_radar_timestamp_ns != 0)
    {
        uint64_t age_ns;

        if (current_timestamp_ns >=
            radar->timestamp_ns)
        {
            age_ns =
                current_timestamp_ns -
                radar->timestamp_ns;
        }
        else
        {
            age_ns = 0;
        }

        if (age_ns >
            timing_ms_to_ns(
                SENSOR_STALE_TIMEOUT_MS
            ))
        {
            tracking->distance_cm =
                radar->distance_cm;

            tracking->relative_speed_cm_s =
                0.0;

            tracking->timestamp_ns =
                current_timestamp_ns;

            tracking->status =
                SENSOR_STALE;

            return ACC_FAILURE;
        }
    }


    /* --------------------------------------------------------
     * Calculate relative speed
     * -------------------------------------------------------- */

    relative_speed_cm_s = 0.0;


    /*
     * A relative-speed calculation requires two valid
     * measurements.
     */
    if (context->previous_timestamp_ns != 0)
    {
        uint64_t delta_time_ns;

        delta_time_ns =
            timing_difference_ns(
                context->previous_timestamp_ns,
                radar->timestamp_ns
            );

        if (delta_time_ns > 0)
        {
            dt_seconds =
                ((double)delta_time_ns) /
                1000000000.0;

            /*
             * Positive closing speed means the distance
             * is decreasing.
             *
             *     closing speed =
             *     previous distance - current distance
             *     -----------------------------------
             *                 time
             *
             * Therefore:
             *
             *     positive = target approaching
             *     negative = target moving away
             */
            relative_speed_cm_s =
                (context->previous_distance_cm -
                 radar->distance_cm) /
                dt_seconds;
        }
    }


    /* --------------------------------------------------------
     * Populate valid tracking data
     * -------------------------------------------------------- */

    tracking->distance_cm =
        radar->distance_cm;

    tracking->relative_speed_cm_s =
        relative_speed_cm_s;

    tracking->timestamp_ns =
        current_timestamp_ns;

    tracking->status =
        SENSOR_OK;


    /* --------------------------------------------------------
     * Update previous measurement
     * -------------------------------------------------------- */

    context->previous_distance_cm =
        radar->distance_cm;

    context->previous_timestamp_ns =
        radar->timestamp_ns;

    context->last_radar_timestamp_ns =
        radar->timestamp_ns;

    context->sensor_status =
        SENSOR_OK;


    return ACC_SUCCESS;
}


/* ============================================================
 * TRACKING DATA VALIDATION
 * ============================================================ */

int tracking_validate_data(
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
 * SEND TRACKING DATA
 * ============================================================ */

static int tracking_send_data(
    const tracking_data_t *tracking
)
{
    ipc_message_t message;

    int result;


    if (tracking == NULL)
    {
        return ACC_FAILURE;
    }


    if (!tracking_ipc_ready)
    {
        fprintf(
            stderr,
            "TRACKING IPC ERROR: IPC not ready\n"
        );

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
            IPC_MSG_TRACKING_DATA,
            tracking->sequence
        );

    if (result != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "TRACKING IPC ERROR: header initialization failed\n"
        );

        return ACC_FAILURE;
    }


    message.header.sender_pid =
        (int32_t)getpid();


    /* --------------------------------------------------------
     * Copy Tracking data
     * -------------------------------------------------------- */

    message.payload.tracking =
        *tracking;


    /* --------------------------------------------------------
     * Send to Controller
     * -------------------------------------------------------- */

    result =
        ipc_send(
            &controller_queue,
            &message
        );

    if (result != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "TRACKING IPC ERROR: send failed "
            "(sequence=%u)\n",
            tracking->sequence
        );

        return ACC_FAILURE;
    }


    return ACC_SUCCESS;
}


/* ============================================================
 * TRACKING THREAD
 * ============================================================ */

void *tracking_thread(
    void *argument
)
{
    tracking_context_t *context;

    ipc_message_t message;

    tracking_data_t tracking;

    task_timing_t timing;

    uint64_t next_release_ns;

    unsigned int receive_priority;

    uint64_t receive_timestamp_ns;

    uint64_t ipc_latency_ns;

    int receive_result;
    int process_result;
    int send_result;


    context =
        (tracking_context_t *)argument;


    if (context == NULL)
    {
        fprintf(
            stderr,
            "TRACKING THREAD ERROR: NULL context\n"
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
        TRACKING_PERIOD_MS
    );


    timing_period_init(
        &next_release_ns,
        TRACKING_PERIOD_MS
    );


    printf(
        "TRACKING THREAD: started\n"
    );


    /* ========================================================
     * TRACKING LOOP
     * ======================================================== */

    while (context->running)
    {
        /* ----------------------------------------------------
         * Start task timing
         * ---------------------------------------------------- */


        /* ----------------------------------------------------
         * Receive Radar message
         * ---------------------------------------------------- */

        memset(
            &message,
            0,
            sizeof(message)
        );

        receive_priority = 0;


        receive_result =
            ipc_receive(
                &radar_queue,
                &message,
                &receive_priority
            );


        receive_timestamp_ns =
            timing_now_ns();


        if (receive_result != ACC_SUCCESS)
        {
            fprintf(
                stderr,
                "TRACKING IPC ERROR: receive failed\n"
            );

            tracking.distance_cm =
                0.0;

            tracking.relative_speed_cm_s =
                0.0;

            tracking.timestamp_ns =
                receive_timestamp_ns;

            tracking.sequence =
                context->sequence++;

            tracking.status =
                SENSOR_TIMEOUT;

            tracking.timing =
                timing;

            /*
             * Complete timing before continuing.
             */
            timing_stop(
                &timing
            );

            /*
             * Wait for next periodic release.
             */
            timing_period_wait(
                &next_release_ns,
                TRACKING_PERIOD_MS
            );

            continue;
        }

        /* Start execution timing only after a successful IPC receive. */
        if (timing_start(&timing) != ACC_SUCCESS)
        {
            fprintf(
                stderr,
                "TRACKING TIMING ERROR: timing_start failed\n"
            );
        }


        /* ----------------------------------------------------
         * Verify message type
         * ---------------------------------------------------- */

        if (message.header.type !=
            IPC_MSG_RADAR_DATA)
        {
            fprintf(
                stderr,
                "TRACKING IPC ERROR: unexpected message type "
                "%d\n",
                message.header.type
            );

            timing_stop(
                &timing
            );

            timing_period_wait(
                &next_release_ns,
                TRACKING_PERIOD_MS
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
         * Process Radar data
         * ---------------------------------------------------- */

        process_result =
            tracking_process_radar(
                context,
                &message.payload.radar,
                &tracking
            );


        /* ----------------------------------------------------
         * Complete task timing
         * ---------------------------------------------------- */

        if (timing_stop(
                &timing
            ) != ACC_SUCCESS)
        {
            fprintf(
                stderr,
                "TRACKING TIMING ERROR: timing_stop failed\n"
            );
        }


        tracking.timing =
            timing;


        /* ----------------------------------------------------
         * Check deadline
         * ---------------------------------------------------- */

        if (timing_deadline_missed(
                &timing
            ))
        {
            fprintf(
                stderr,
                "TRACKING WARNING: deadline missed "
                "(%.3f ms)\n",
                timing_execution_time_ms(
                    &timing
                )
            );
        }


        /* ----------------------------------------------------
         * Send Tracking data
         * ---------------------------------------------------- */

        send_result =
            tracking_send_data(
                &tracking
            );


        if (send_result != ACC_SUCCESS)
        {
            fprintf(
                stderr,
                "TRACKING WARNING: IPC send failure\n"
            );
        }


        /* ----------------------------------------------------
         * Diagnostic output
         * ---------------------------------------------------- */

        if (process_result == ACC_SUCCESS)
        {
            printf(
                "[TRACKING]   | #%04u | DIST %7.2f cm | REL SPD %8.2f cm/s | IPC %6.3f ms | EXEC %6.3f ms\n",
                tracking.sequence,
                tracking.distance_cm,
                tracking.relative_speed_cm_s,
                timing_ns_to_ms(
                    ipc_latency_ns
                ),
                timing_execution_time_ms(
                    &timing
                )
            );
        }
        else
        {
            printf(
                "[TRACKING]   | #%04u | SENSOR FAULT | STATUS %d | IPC %6.3f ms | EXEC %6.3f ms\n",
                tracking.sequence,
                tracking.status,
                timing_ns_to_ms(
                    ipc_latency_ns
                ),
                timing_execution_time_ms(
                    &timing
                )
            );
        }


        /* ----------------------------------------------------
         * Periodic release
         * ---------------------------------------------------- */

        if (timing_period_wait(
                &next_release_ns,
                TRACKING_PERIOD_MS
            ) != ACC_SUCCESS)
        {
            fprintf(
                stderr,
                "TRACKING TIMING ERROR: periodic wait failed\n"
            );
        }
    }


    printf(
        "TRACKING THREAD: stopped\n"
    );


    return NULL;
}


/* ============================================================
 * TRACKING SHUTDOWN
 * ============================================================ */

int tracking_shutdown(
    tracking_context_t *context
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
     * Close Radar queue
     * -------------------------------------------------------- */

    if (tracking_ipc_ready)
    {
        result =
            ipc_close(
                &radar_queue
            );

        if (result != ACC_SUCCESS)
        {
            fprintf(
                stderr,
                "TRACKING WARNING: Radar queue close failed\n"
            );

            shutdown_result =
                ACC_FAILURE;
        }


        /* ----------------------------------------------------
         * Close Controller queue
         * ---------------------------------------------------- */

        result =
            ipc_close(
                &controller_queue
            );

        if (result != ACC_SUCCESS)
        {
            fprintf(
                stderr,
                "TRACKING WARNING: Controller queue close failed\n"
            );

            shutdown_result =
                ACC_FAILURE;
        }


        tracking_ipc_ready = 0;
    }


    printf(
        "TRACKING: shutdown complete\n"
    );


    return shutdown_result;
}
