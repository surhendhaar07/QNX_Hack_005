#ifndef ACC_RADAR_H
#define ACC_RADAR_H

/*
 * QNX Intelligent Adaptive Cruise Control
 *
 * Radar / HC-SR04 Process Interface
 *
 * Process:
 *     acc_radar
 *
 * Responsibilities:
 *     - Initialize HC-SR04
 *     - Acquire real sensor measurements
 *     - Detect sensor timeout/invalid conditions
 *     - Publish radar data through POSIX IPC
 *     - Measure task timing
 *     - Support clean shutdown
 *
 * No simulated or random sensor values are generated.
 */

#include <stdint.h>
#include <pthread.h>

#include "../common/acc_types.h"


/* ============================================================
 * RADAR CONTEXT
 * ============================================================ */

typedef struct
{
    /*
     * Indicates whether the Radar worker thread
     * should continue running.
     *
     * 1 = running
     * 0 = stop requested
     */
    volatile int running;

    /*
     * HC-SR04 measurement sequence number.
     */
    uint32_t sequence;

    /*
     * Number of sensor faults detected.
     */
    uint32_t sensor_fault_count;

    /*
     * Last sensor status.
     */
    sensor_status_t sensor_status;

    /*
     * Last measured distance.
     *
     * This is only valid when sensor_status == SENSOR_OK.
     */
    double distance_cm;

    /*
     * Last measurement timestamp.
     */
    uint64_t last_measurement_ns;

} radar_context_t;


/* ============================================================
 * RADAR INITIALIZATION
 * ============================================================ */

/*
 * Initialize the Radar process.
 *
 * Initializes:
 *     - Radar context
 *     - HC-SR04 hardware interface
 *
 * Returns:
 *     ACC_SUCCESS
 *     ACC_FAILURE
 */
int radar_init(
    radar_context_t *context
);


/* ============================================================
 * RADAR THREAD
 * ============================================================ */

/*
 * Main Radar worker thread.
 *
 * The thread:
 *
 *     1. Performs a real HC-SR04 measurement.
 *     2. Validates the measurement.
 *     3. Detects timeout/fault conditions.
 *     4. Creates radar_data_t.
 *     5. Sends the data through IPC.
 *     6. Measures execution/response timing.
 *     7. Runs periodically.
 */
void *radar_thread(
    void *argument
);


/* ============================================================
 * RADAR SHUTDOWN
 * ============================================================ */

/*
 * Stop the Radar worker and release sensor resources.
 *
 * Returns:
 *     ACC_SUCCESS
 *     ACC_FAILURE
 */
int radar_shutdown(
    radar_context_t *context
);


/* ============================================================
 * RADAR DATA ACQUISITION
 * ============================================================ */

/*
 * Acquire one radar measurement.
 *
 * The measurement must come from the real HC-SR04 sensor.
 *
 * No random/default distance is generated when the sensor
 * is unavailable.
 *
 * On failure:
 *     data->status is set to SENSOR_TIMEOUT or
 *     SENSOR_INVALID.
 *
 * Returns:
 *     ACC_SUCCESS = valid measurement
 *     ACC_FAILURE = sensor fault/invalid measurement
 */
int radar_read(
    radar_context_t *context,
    radar_data_t *data
);


/* ============================================================
 * SENSOR FAULT HANDLING
 * ============================================================ */

/*
 * Process a sensor failure.
 *
 * This function updates the Radar context and produces
 * an appropriate sensor status.
 */
void radar_handle_sensor_fault(
    radar_context_t *context,
    radar_data_t *data
);


/* ============================================================
 * RADAR DATA VALIDATION
 * ============================================================ */

/*
 * Validate radar data before it is sent to Tracking.
 *
 * Returns:
 *     ACC_SUCCESS = valid
 *     ACC_FAILURE = invalid
 */
int radar_validate_data(
    const radar_data_t *data
);


/* ============================================================
 * RADAR STATUS
 * ============================================================ */

/*
 * Return the most recent sensor status.
 */
sensor_status_t radar_get_sensor_status(
    const radar_context_t *context
);


/*
 * Return the number of detected sensor faults.
 */
uint32_t radar_get_fault_count(
    const radar_context_t *context
);


#endif /* ACC_RADAR_H */
