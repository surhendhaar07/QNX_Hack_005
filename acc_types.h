#ifndef ACC_TYPES_H
#define ACC_TYPES_H

#include <stdint.h>

#include "timing_types.h"


/* ============================================================
 * SYSTEM STATE
 * ============================================================ */

typedef enum
{
    ACC_STATE_INIT = 0,
    ACC_STATE_ACTIVE,
    ACC_STATE_WARNING,
    ACC_STATE_EMERGENCY,
    ACC_STATE_FAULT,
    ACC_STATE_STOPPED

} acc_state_t;


/* ============================================================
 * SENSOR STATUS
 * ============================================================ */

typedef enum
{
    SENSOR_OK = 0,
    SENSOR_TIMEOUT,
    SENSOR_INVALID,
    SENSOR_STALE

} sensor_status_t;


/* ============================================================
 * MOTOR DIRECTION
 * ============================================================ */

typedef enum
{
    MOTOR_STOP = 0,
    MOTOR_FORWARD,
    MOTOR_REVERSE

} motor_direction_t;


/* ============================================================
 * MOTOR STATUS
 * ============================================================ */

typedef enum
{
    MOTOR_DISABLED = 0,
    MOTOR_ENABLED,
    MOTOR_FAULT

} motor_status_t;


/* ============================================================
 * RADAR / HC-SR04 DATA
 * ============================================================ */

typedef struct
{
    /*
     * Actual measured distance.
     *
     * No synthetic or random value is generated.
     */
    double distance_cm;

    /*
     * Sensor measurement timestamp.
     */
    uint64_t timestamp_ns;

    /*
     * Sensor validity state.
     */
    sensor_status_t status;

    /*
     * Measurement sequence number.
     */
    uint32_t sequence;

    /*
     * Timing measurement for the radar task.
     */
    task_timing_t timing;

} radar_data_t;


/* ============================================================
 * TRACKING DATA
 * ============================================================ */

typedef struct
{
    /*
     * Current measured distance.
     */
    double distance_cm;

    /*
     * Relative speed in cm/s.
     *
     * Negative value:
     *     object is getting closer
     *
     * Positive value:
     *     object is getting farther away
     */
    double relative_speed_cm_s;

    /*
     * Timestamp of the tracking calculation.
     */
    uint64_t timestamp_ns;

    /*
     * Validity of tracking data.
     */
    sensor_status_t status;

    /*
     * Sequence number.
     */
    uint32_t sequence;

    /*
     * Timing information.
     */
    task_timing_t timing;

} tracking_data_t;


/* ============================================================
 * CONTROL COMMAND
 * ============================================================ */

typedef struct
{
    /*
     * Requested motor direction.
     */
    motor_direction_t direction;

    /*
     * PWM duty-cycle request.
     *
     * Range:
     *     0.0 - 100.0 %
     *
     * This is a command value.
     * The physical PWM backend will be implemented separately.
     */
    double duty_percent;

    /*
     * Requested target speed.
     */
    double target_speed_kmh;

    /*
     * Safety state associated with this command.
     */
    acc_state_t state;

    /*
     * Whether actuator operation is allowed.
     */
    int actuator_enable;

    /*
     * Timestamp when command was generated.
     */
    uint64_t timestamp_ns;

    /*
     * Timing information.
     */
    task_timing_t timing;

} control_command_t;


/* ============================================================
 * SAFETY STATUS
 * ============================================================ */

typedef struct
{
    /*
     * Current safety state.
     */
    acc_state_t state;

    /*
     * Current sensor status.
     */
    sensor_status_t sensor_status;

    /*
     * Current distance.
     */
    double distance_cm;

    /*
     * Controller deadline condition.
     */
    int controller_deadline_missed;

    /*
     * Safety fault flag.
     */
    int fault_active;

    /*
     * Emergency stop flag.
     */
    int emergency_stop;

    /*
     * Timestamp of safety decision.
     */
    uint64_t timestamp_ns;

    /*
     * Safety task timing.
     */
    task_timing_t timing;

} safety_status_t;


/* ============================================================
 * PROCESS HEALTH
 * ============================================================ */

typedef enum
{
    PROCESS_HEALTH_UNKNOWN = 0,
    PROCESS_HEALTH_RUNNING,
    PROCESS_HEALTH_FAULT,
    PROCESS_HEALTH_STOPPED

} process_health_t;


typedef struct
{
    /*
     * PID of monitored process.
     */
    int32_t pid;

    /*
     * Process health.
     */
    process_health_t health;

    /*
     * Last heartbeat timestamp.
     */
    uint64_t last_heartbeat_ns;

    /*
     * Number of missed heartbeats.
     */
    uint32_t missed_heartbeats;

} process_status_t;


/* ============================================================
 * SUPERVISOR STATUS
 * ============================================================ */

typedef struct
{
    process_status_t radar;
    process_status_t tracking;
    process_status_t controller;
    process_status_t safety;
    process_status_t actuator;

    /*
     * Overall supervisor fault.
     */
    int system_fault;

    /*
     * Request for global safe stop.
     */
    int safe_stop_requested;

    /*
     * Timestamp of supervisor decision.
     */
    uint64_t timestamp_ns;

} supervisor_status_t;


#endif /* ACC_TYPES_H */
