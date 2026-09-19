#ifndef HC_SR04_H
#define HC_SR04_H

/*
 * QNX Intelligent Adaptive Cruise Control
 *
 * HC-SR04 Ultrasonic Sensor Interface
 *
 * Hardware:
 *
 *     HC-SR04 TRIG -> Raspberry Pi GPIO23
 *     HC-SR04 ECHO -> Raspberry Pi GPIO24
 *
 * IMPORTANT:
 *
 * HC-SR04 ECHO is a 5V signal.
 * Raspberry Pi GPIO input is 3.3V.
 *
 * A voltage divider / level shifter MUST be used
 * between HC-SR04 ECHO and GPIO24.
 *
 * GPIO interface:
 *
 *     QNX rpi_gpio resource manager
 *     /dev/gpio/23
 *     /dev/gpio/24
 *
 * No simulated or random sensor values are used.
 */

#include <stdint.h>

#include "../common/acc_types.h"


/* ============================================================
 * SENSOR INITIALIZATION
 * ============================================================ */

int hc_sr04_init(void);


/* ============================================================
 * SENSOR SHUTDOWN
 * ============================================================ */

int hc_sr04_shutdown(void);


/* ============================================================
 * DISTANCE MEASUREMENT
 * ============================================================ */

/*
 * Perform one actual HC-SR04 measurement.
 *
 * distance_cm:
 *     receives the measured distance.
 *
 * Returns:
 *
 *      0 = valid measurement
 *     -1 = measurement failure
 *
 * No synthetic/default distance is generated on failure.
 */
int hc_sr04_measure_distance(
    double *distance_cm
);


/* ============================================================
 * COMPLETE SENSOR MEASUREMENT
 * ============================================================ */

/*
 * Perform one measurement and fill radar_data_t.
 *
 * sequence:
 *     Radar measurement sequence number.
 *
 * Returns:
 *
 *      0 = valid measurement
 *     -1 = measurement failure
 */
int hc_sr04_read(
    radar_data_t *data,
    uint32_t sequence
);


/* ============================================================
 * SENSOR VALIDATION
 * ============================================================ */

int hc_sr04_validate_distance(
    double distance_cm
);

#endif /* HC_SR04_H */
