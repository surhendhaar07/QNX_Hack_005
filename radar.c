/*
 * QNX Intelligent Adaptive Cruise Control
 *
 * Radar Process
 *
 * Responsibilities:
 *
 *     - Initialize HC-SR04
 *     - Periodically acquire real sensor data
 *     - Detect sensor timeout / invalid measurements
 *     - Measure Radar task execution time
 *     - Measure IPC timestamp information
 *     - Send radar data to Tracking
 *     - Handle clean shutdown
 *
 * No simulated or random sensor values are generated.
 */

#include "radar.h"

#include "hc_sr04.h"

#include "../common/acc_config.h"
#include "../common/ipc.h"
#include "../common/ipc_types.h"
#include "../common/timing.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>


/* ============================================================
 * RADAR IPC
 * ============================================================ */

/*
 * Radar is a sender.
 *
 * Radar sends:
 *
 *     Radar -> /acc_radar_q -> Tracking
 *
 * The queue descriptor belongs only to this process.
 */
static ipc_queue_t radar_tracking_queue;


/*
 * Indicates whether the IPC queue was successfully opened.
 */
static int radar_ipc_ready = 0;


/* ============================================================
 * INTERNAL HELPERS
 * ============================================================ */

static void radar_initialize_context(
    radar_context_t *context
)
{
    if (context == NULL)
    {
        return;
    }

    memset(
        context,
        0,
        sizeof(*context)
    );

    context->running = 1;

    context->sequence = 0;

    context->sensor_fault_count = 0;

    context->sensor_status = SENSOR_TIMEOUT;

    context->distance_cm = 0.0;

    context->last_measurement_ns = 0;
}


/* ============================================================
 * RADAR INITIALIZATION
 * ============================================================ */

int radar_init(
    radar_context_t *context
)
{
    int result;

    if (context == NULL)
    {
        fprintf(
            stderr,
            "RADAR ERROR: NULL context\n"
        );

        return ACC_FAILURE;
    }


    /* --------------------------------------------------------
     * Initialize Radar context
     * -------------------------------------------------------- */

    radar_initialize_context(
        context
    );


    /* --------------------------------------------------------
     * Validate IPC message size
     * -------------------------------------------------------- */

    result =
        ipc_validate_message_size();

    if (result != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "RADAR ERROR: IPC message size validation failed\n"
        );

        return ACC_FAILURE;
    }


    /* --------------------------------------------------------
     * Open Radar -> Tracking IPC queue
     * -------------------------------------------------------- */

    result =
        ipc_open_sender(
            &radar_tracking_queue,
            ACC_RADAR_QUEUE_NAME
        );

    if (result != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "RADAR ERROR: unable to open Tracking IPC queue\n"
        );

        radar_ipc_ready = 0;

        return ACC_FAILURE;
    }

    radar_ipc_ready = 1;


    /* --------------------------------------------------------
     * Initialize HC-SR04
     * -------------------------------------------------------- */

    result =
        hc_sr04_init();

    if (result != ACC_SUCCESS)
    {
        /*
         * The sensor is currently not configured with a
         * Raspberry Pi QNX GPIO backend.
         *
         * Do not generate a fake measurement.
         *
         * Keep the Radar process alive so that the system
         * can report a sensor fault through the ACC pipeline.
         */
        fprintf(
            stderr,
            "RADAR WARNING: HC-SR04 hardware initialization "
            "failed\n"
        );

        context->sensor_status =
            SENSOR_TIMEOUT;

        context->running = 1;

        /*
         * IPC remains available.
         *
         * The worker thread will publish SENSOR_TIMEOUT
         * rather than inventing a distance.
         */
    }


    printf(
        "RADAR: initialization complete\n"
    );

    return ACC_SUCCESS;
}


/* ============================================================
 * RADAR DATA ACQUISITION
 * ============================================================ */

int radar_read(
    radar_context_t *context,
    radar_data_t *data
)
{
    uint64_t measurement_start_ns;
    uint64_t measurement_end_ns;

    int result;


    if (context == NULL ||
        data == NULL)
    {
        return ACC_FAILURE;
    }


    /* --------------------------------------------------------
     * Clear output structure
     * -------------------------------------------------------- */

    memset(
        data,
        0,
        sizeof(*data)
    );


    data->sequence =
        context->sequence;


    /* --------------------------------------------------------
     * Record task start time
     * -------------------------------------------------------- */

    measurement_start_ns =
        timing_now_ns();

    if (measurement_start_ns == 0)
    {
        data->status =
            SENSOR_INVALID;

        return ACC_FAILURE;
    }


    /* --------------------------------------------------------
     * Read actual HC-SR04 sensor
     * -------------------------------------------------------- */

    result =
        hc_sr04_read(
            data,
            context->sequence
        );


    /* --------------------------------------------------------
     * Record task end time
     * -------------------------------------------------------- */

    measurement_end_ns =
        timing_now_ns();


    /*
     * If the sensor read failed, do not use a fabricated
     * distance.
     */
    if (result != ACC_SUCCESS)
    {
        /*
         * hc_sr04_read() already reports SENSOR_TIMEOUT or
         * SENSOR_INVALID where appropriate.
         *
         * If no valid status was produced, classify it as
         * SENSOR_TIMEOUT.
         */
        if (data->status != SENSOR_INVALID &&
            data->status != SENSOR_TIMEOUT)
        {
            data->status =
                SENSOR_TIMEOUT;
        }

        data->distance_cm = 0.0;

        data->timestamp_ns =
            measurement_end_ns;

        return ACC_FAILURE;
    }


    /* --------------------------------------------------------
     * Validate successful measurement
     * -------------------------------------------------------- */

    if (data->status != SENSOR_OK)
    {
        data->distance_cm = 0.0;

        return ACC_FAILURE;
    }


    /*
     * A successful sensor read must contain a valid timestamp.
     */
    if (data->timestamp_ns == 0)
    {
        data->status =
            SENSOR_INVALID;

        data->distance_cm = 0.0;

        return ACC_FAILURE;
    }


    /*
     * Make sure the timing timestamps are valid.
     */
    if (measurement_end_ns < measurement_start_ns)
    {
        data->status =
            SENSOR_INVALID;

        data->distance_cm = 0.0;

        return ACC_FAILURE;
    }


    return ACC_SUCCESS;
}


/* ============================================================
 * SENSOR FAULT HANDLING
 * ============================================================ */

void radar_handle_sensor_fault(
    radar_context_t *context,
    radar_data_t *data
)
{
    if (context == NULL ||
        data == NULL)
    {
        return;
    }


    /* --------------------------------------------------------
     * Increment fault counter
     * -------------------------------------------------------- */

    context->sensor_fault_count++;


    /* --------------------------------------------------------
     * Preserve sensor status
     * -------------------------------------------------------- */

    context->sensor_status =
        data->status;


    /*
     * Do not preserve an invalid distance as usable data.
     */
    context->distance_cm =
        0.0;


    /*
     * Keep the timestamp so downstream processes can
     * determine when the fault report was generated.
     */
    context->last_measurement_ns =
        data->timestamp_ns;


    /*
     * No fake distance is generated.
     */
    data->distance_cm =
        0.0;
}


/* ============================================================
 * RADAR DATA VALIDATION
 * ============================================================ */

int radar_validate_data(
    const radar_data_t *data
)
{
    if (data == NULL)
    {
        return ACC_FAILURE;
    }


    /* --------------------------------------------------------
     * Sensor status must be OK
     * -------------------------------------------------------- */

    if (data->status != SENSOR_OK)
    {
        return ACC_FAILURE;
    }


    /* --------------------------------------------------------
     * Timestamp must exist
     * -------------------------------------------------------- */

    if (data->timestamp_ns == 0)
    {
        return ACC_FAILURE;
    }


    /* --------------------------------------------------------
     * Physical distance validation
     * -------------------------------------------------------- */

    if (data->distance_cm <
        ACC_MIN_VALID_DISTANCE_CM)
    {
        return ACC_FAILURE;
    }


    if (data->distance_cm >
        ACC_MAX_VALID_DISTANCE_CM)
    {
        return ACC_FAILURE;
    }


    return ACC_SUCCESS;
}


/* ============================================================
 * SEND RADAR DATA
 * ============================================================ */

static int radar_send_data(
    radar_context_t *context,
    const radar_data_t *data
)
{
    ipc_message_t message;

    int result;


    if (context == NULL ||
        data == NULL)
    {
        return ACC_FAILURE;
    }


    if (!radar_ipc_ready)
    {
        fprintf(
            stderr,
            "RADAR IPC ERROR: Tracking queue unavailable\n"
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
            IPC_MSG_RADAR_DATA,
            data->sequence
        );

    if (result != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "RADAR IPC ERROR: header initialization failed\n"
        );

        return ACC_FAILURE;
    }


    /* --------------------------------------------------------
     * Set sender PID
     * -------------------------------------------------------- */

    message.header.sender_pid =
        (int32_t)getpid();


    /* --------------------------------------------------------
     * Copy Radar data into IPC payload
     * -------------------------------------------------------- */

    message.payload.radar =
        *data;


    /* --------------------------------------------------------
     * Send message
     * -------------------------------------------------------- */

    result =
        ipc_send(
            &radar_tracking_queue,
            &message
        );

    if (result != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "RADAR IPC ERROR: unable to send Radar data "
            "(sequence=%u)\n",
            data->sequence
        );

        return ACC_FAILURE;
    }


    return ACC_SUCCESS;
}


/* ============================================================
 * RADAR THREAD
 * ============================================================ */

void *radar_thread(
    void *argument
)
{
    radar_context_t *context;

    radar_data_t radar_data;

    task_timing_t timing;

    uint64_t next_release_ns;

    int sensor_result;
    int send_result;
    int timing_result;


    context =
        (radar_context_t *)argument;


    if (context == NULL)
    {
        fprintf(
            stderr,
            "RADAR THREAD ERROR: NULL context\n"
        );

        return NULL;
    }


    /* --------------------------------------------------------
     * Initialize task timing
     * -------------------------------------------------------- */

    timing_init(
        &timing
    );


    timing_set_deadline(
        &timing,
        RADAR_PERIOD_MS
    );


    /* --------------------------------------------------------
     * Initialize periodic release
     * -------------------------------------------------------- */

    timing_period_init(
        &next_release_ns,
        RADAR_PERIOD_MS
    );


    printf(
        "RADAR THREAD: started\n"
    );


    /* ========================================================
     * PERIODIC RADAR LOOP
     * ======================================================== */

    while (context->running)
    {
        /* ----------------------------------------------------
         * Start timing measurement
         * ---------------------------------------------------- */

        timing_result =
            timing_start(
                &timing
            );

        if (timing_result != ACC_SUCCESS)
        {
            fprintf(
                stderr,
                "RADAR TIMING ERROR: timing_start failed\n"
            );
        }


        /* ----------------------------------------------------
         * Update sequence number
         * ---------------------------------------------------- */

        context->sequence++;


        /* ----------------------------------------------------
         * Acquire real sensor data
         * ---------------------------------------------------- */

        sensor_result =
            radar_read(
                context,
                &radar_data
            );


        /* ----------------------------------------------------
         * Handle sensor fault
         * ---------------------------------------------------- */

        if (sensor_result != ACC_SUCCESS)
        {
            radar_handle_sensor_fault(
                context,
                &radar_data
            );
        }
        else
        {
            context->sensor_status =
                SENSOR_OK;

            context->distance_cm =
                radar_data.distance_cm;

            context->last_measurement_ns =
                radar_data.timestamp_ns;
        }


        /* ----------------------------------------------------
         * Complete task timing
         * ---------------------------------------------------- */

        timing_result =
            timing_stop(
                &timing
            );

        if (timing_result != ACC_SUCCESS)
        {
            fprintf(
                stderr,
                "RADAR TIMING ERROR: timing_stop failed\n"
            );
        }


        /*
         * Store task timing inside the radar message.
         */
        radar_data.timing =
            timing;


        /* ----------------------------------------------------
         * Check deadline
         * ---------------------------------------------------- */

        if (timing_deadline_missed(&timing))
        {
            fprintf(
                stderr,
                "RADAR WARNING: task deadline missed "
                "(%.3f ms)\n",
                timing_execution_time_ms(&timing)
            );
        }


        /* ----------------------------------------------------
         * Send data to Tracking
         * ---------------------------------------------------- */

        send_result =
            radar_send_data(
                context,
                &radar_data
            );

        if (send_result != ACC_SUCCESS)
        {
            /*
             * IPC failure is reported, but the Radar thread
             * does not generate replacement sensor data.
             */
            fprintf(
                stderr,
                "RADAR WARNING: Radar data IPC failure\n"
            );
        }


        /* ----------------------------------------------------
         * Diagnostic output
         * ---------------------------------------------------- */

        if (radar_data.status == SENSOR_OK)
        {
            printf(
                "[RADAR]      | #%04u | DIST %7.2f cm | SENSOR OK    | EXEC %7.3f ms\n",
                radar_data.sequence,
                radar_data.distance_cm,
                timing_execution_time_ms(&timing)
            );
        }
        else
        {
            printf(
                "[RADAR]      | #%04u | SENSOR FAULT | STATUS %d | FAULTS %u | EXEC %7.3f ms\n",
                radar_data.sequence,
                radar_data.status,
                context->sensor_fault_count,
                timing_execution_time_ms(&timing)
            );
        }


        /* ----------------------------------------------------
         * Periodic release
         * ---------------------------------------------------- */

        if (timing_period_wait(
                &next_release_ns,
                RADAR_PERIOD_MS
            ) != ACC_SUCCESS)
        {
            fprintf(
                stderr,
                "RADAR TIMING ERROR: periodic wait failed\n"
            );
        }
    }


    printf(
        "RADAR THREAD: stopped\n"
    );


    return NULL;
}


/* ============================================================
 * RADAR SHUTDOWN
 * ============================================================ */

int radar_shutdown(
    radar_context_t *context
)
{
    int result;
    int shutdown_result = ACC_SUCCESS;


    if (context == NULL)
    {
        return ACC_FAILURE;
    }


    /* --------------------------------------------------------
     * Request worker thread termination
     * -------------------------------------------------------- */

    context->running = 0;


    /* --------------------------------------------------------
     * Shutdown HC-SR04
     * -------------------------------------------------------- */

    result =
        hc_sr04_shutdown();

    if (result != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "RADAR WARNING: HC-SR04 shutdown failed\n"
        );

        shutdown_result =
            ACC_FAILURE;
    }


    /* --------------------------------------------------------
     * Close IPC queue
     * -------------------------------------------------------- */

    if (radar_ipc_ready)
    {
        result =
            ipc_close(
                &radar_tracking_queue
            );

        if (result != ACC_SUCCESS)
        {
            fprintf(
                stderr,
                "RADAR WARNING: IPC close failed\n"
            );

            shutdown_result =
                ACC_FAILURE;
        }

        radar_ipc_ready = 0;
    }


    printf(
        "RADAR: shutdown complete\n"
    );


    return shutdown_result;
}


/* ============================================================
 * GET SENSOR STATUS
 * ============================================================ */

sensor_status_t radar_get_sensor_status(
    const radar_context_t *context
)
{
    if (context == NULL)
    {
        return SENSOR_INVALID;
    }

    return context->sensor_status;
}


/* ============================================================
 * GET SENSOR FAULT COUNT
 * ============================================================ */

uint32_t radar_get_fault_count(
    const radar_context_t *context
)
{
    if (context == NULL)
    {
        return 0;
    }

    return context->sensor_fault_count;
}
