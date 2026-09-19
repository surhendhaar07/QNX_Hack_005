#ifndef ACC_TRACKING_H
#define ACC_TRACKING_H

/*
 * QNX Intelligent Adaptive Cruise Control
 *
 * Tracking Process Interface
 *
 * Process:
 *     acc_tracking
 *
 * Responsibilities:
 *     - Receive Radar data
 *     - Validate sensor information
 *     - Calculate relative target speed
 *     - Detect stale sensor data
 *     - Measure execution time and IPC latency
 *     - Send tracking data to Controller
 */

#include <stdint.h>
#include <pthread.h>

#include "../common/acc_types.h"


/* ============================================================
 * TRACKING CONTEXT
 * ============================================================ */

typedef struct
{
    /*
     * Worker thread execution state.
     *
     * 1 = running
     * 0 = stop requested
     */
    volatile int running;

    /*
     * Tracking sequence number.
     */
    uint32_t sequence;

    /*
     * Previous valid Radar distance.
     */
    double previous_distance_cm;

    /*
     * Timestamp of previous Radar measurement.
     */
    uint64_t previous_timestamp_ns;

    /*
     * Most recent Radar timestamp.
     */
    uint64_t last_radar_timestamp_ns;

    /*
     * Current sensor status.
     */
    sensor_status_t sensor_status;

} tracking_context_t;


/* ============================================================
 * INITIALIZATION
 * ============================================================ */

/*
 * Initialize Tracking process.
 *
 * Opens:
 *
 *     Radar -> Tracking
 *     Tracking -> Controller
 *
 * Returns:
 *
 *     ACC_SUCCESS
 *     ACC_FAILURE
 */
int tracking_init(
    tracking_context_t *context
);


/* ============================================================
 * TRACKING THREAD
 * ============================================================ */

/*
 * Tracking worker thread.
 *
 * The thread:
 *
 *     1. Receives Radar IPC data.
 *     2. Measures IPC latency.
 *     3. Checks sensor status.
 *     4. Detects stale data.
 *     5. Calculates relative speed.
 *     6. Measures execution time.
 *     7. Sends tracking data to Controller.
 */
void *tracking_thread(
    void *argument
);


/* ============================================================
 * RADAR DATA PROCESSING
 * ============================================================ */

/*
 * Process one Radar measurement.
 *
 * Relative speed is calculated from two valid measurements.
 *
 * Positive value:
 *     target is approaching.
 *
 * Negative value:
 *     target is moving away.
 *
 * Returns:
 *
 *     ACC_SUCCESS = valid tracking result
 *     ACC_FAILURE = invalid/stale sensor data
 */
int tracking_process_radar(
    tracking_context_t *context,
    const radar_data_t *radar,
    tracking_data_t *tracking
);


/* ============================================================
 * TRACKING DATA VALIDATION
 * ============================================================ */

/*
 * Validate a tracking result before it is transmitted
 * to the Controller process.
 *
 * Returns:
 *
 *     ACC_SUCCESS = valid
 *     ACC_FAILURE = invalid
 */
int tracking_validate_data(
    const tracking_data_t *tracking
);


/* ============================================================
 * SHUTDOWN
 * ============================================================ */

/*
 * Stop Tracking and close IPC resources.
 *
 * Returns:
 *
 *     ACC_SUCCESS
 *     ACC_FAILURE
 */
int tracking_shutdown(
    tracking_context_t *context
);


#endif /* ACC_TRACKING_H */
