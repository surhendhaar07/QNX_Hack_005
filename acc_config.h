#ifndef ACC_CONFIG_H
#define ACC_CONFIG_H

/*
 * QNX Intelligent Adaptive Cruise Control
 * Common Configuration
 *
 * Hardware:
 *   Raspberry Pi 4/5
 *   HC-SR04 Ultrasonic Sensor
 *   L298N Motor Driver
 *   DC Motor
 *
 * No simulated sensor or motor values are used.
 */

/* ============================================================
 * ACC PARAMETERS
 * ============================================================ */

/* Target vehicle speed */
#define ACC_TARGET_SPEED_KMH          20.0

/* Distance thresholds */
/* >= 30 cm : normal ACC operation */
/* < 30 cm and > 15 cm : reduced-speed operation */
/* <= 15 cm : emergency stop */
#define ACC_SAFE_DISTANCE_CM          30.0
#define ACC_WARNING_DISTANCE_CM       30.0
#define ACC_EMERGENCY_DISTANCE_CM     15.0

/* Reduced-speed command in the warning zone. */
#define ACC_NORMAL_DUTY_PERCENT       100.0
#define ACC_WARNING_DUTY_PERCENT       30.0


/* ============================================================
 * HC-SR04 GPIO CONFIGURATION
 * ============================================================ */

/*
 * Raspberry Pi BCM GPIO numbering
 */

#define HC_SR04_TRIG_GPIO             23
#define HC_SR04_ECHO_GPIO             24

/*
 * Maximum time allowed for HC-SR04 echo response.
 * 30 ms corresponds to the configured sensor timeout.
 */
#define HC_SR04_TIMEOUT_US            30000


/* ============================================================
 * L298N MOTOR DRIVER GPIO CONFIGURATION
 * ============================================================ */

#define L298N_ENA_GPIO                18
#define L298N_IN1_GPIO                17
#define L298N_IN2_GPIO                27


/* ============================================================
 * REAL-TIME TASK PERIODS
 * ============================================================ */

/*
 * Safety has the fastest periodic execution.
 */
#define SAFETY_PERIOD_MS              10

/*
 * Main ACC processing tasks.
 */
#define RADAR_PERIOD_MS               60
#define TRACKING_PERIOD_MS            60
#define CONTROLLER_PERIOD_MS          60
#define ACTUATOR_PERIOD_MS            60

/*
 * Supervisor periodically checks process health.
 */
#define SUPERVISOR_PERIOD_MS          100
#define SUPERVISOR_SAFETY_STATUS_TIMEOUT_MS 300


/* ============================================================
 * DEADLINES AND WATCHDOG LIMITS
 * ============================================================ */

/*
 * Maximum allowed controller execution time.
 */
#define CONTROLLER_DEADLINE_MS        20

/*
 * If radar data is older than this value,
 * it is considered stale.
 */
#define SENSOR_STALE_TIMEOUT_MS       150


/* ============================================================
 * QNX THREAD PRIORITIES
 * ============================================================ */

/*
 * Higher numerical priority = higher scheduling priority
 * for the selected scheduling policy.
 *
 * We intentionally stay away from reserved priority values.
 */

#define RADAR_PRIORITY                20
#define TRACKING_PRIORITY             25
#define CONTROLLER_PRIORITY           30
#define ACTUATOR_PRIORITY             35
#define SAFETY_PRIORITY               40
#define SUPERVISOR_PRIORITY           45


/* ============================================================
 * IPC QUEUE NAMES
 * ============================================================ */

#define ACC_RADAR_QUEUE_NAME          "/acc_radar_q"
#define ACC_TRACKING_QUEUE_NAME       "/acc_tracking_q"
#define ACC_CONTROLLER_QUEUE_NAME     "/acc_controller_q"
#define ACC_SAFETY_QUEUE_NAME         "/acc_safety_q"
#define ACC_ACTUATOR_QUEUE_NAME       "/acc_actuator_q"
#define ACC_SUPERVISOR_QUEUE_NAME     "/acc_supervisor_q"


/* ============================================================
 * SHARED MEMORY
 * ============================================================ */

/*
 * This is a live runtime status area.
 *
 * It is NOT used as permanent data storage.
 */
#define ACC_SHARED_MEMORY_NAME        "/acc_shared_memory"


/* ============================================================
 * MESSAGE QUEUE PARAMETERS
 * ============================================================ */

#define ACC_MQ_MAX_MESSAGES           16
#define ACC_MQ_MESSAGE_SIZE           512


/* ============================================================
 * CPU AFFINITY
 * ============================================================ */

/*
 * The actual valid runmask will be verified on the Raspberry Pi
 * QNX target before enabling a specific affinity configuration.
 */
#define ACC_DEFAULT_RUNMASK           0x1U


/* ============================================================
 * SAFETY
 * ============================================================ */

#define ACC_MAX_VALID_DISTANCE_CM     400.0
#define ACC_MIN_VALID_DISTANCE_CM     2.0


/* ============================================================
 * GENERAL
 * ============================================================ */

#define ACC_SUCCESS                   0
#define ACC_FAILURE                  -1

#endif /* ACC_CONFIG_H */
