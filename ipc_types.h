#ifndef IPC_TYPES_H
#define IPC_TYPES_H

#include <stdint.h>

#include "acc_types.h"


/* ============================================================
 * IPC MESSAGE TYPES
 * ============================================================ */

typedef enum
{
    IPC_MSG_RADAR_DATA = 1,
    IPC_MSG_TRACKING_DATA,
    IPC_MSG_CONTROL_COMMAND,
    IPC_MSG_SAFETY_STATUS,
    IPC_MSG_ACTUATOR_COMMAND,
    IPC_MSG_PROCESS_HEARTBEAT,
    IPC_MSG_PROCESS_STATUS,
    IPC_MSG_SAFE_STOP,
    IPC_MSG_SHUTDOWN

} ipc_message_type_t;


/* ============================================================
 * COMMON IPC MESSAGE HEADER
 * ============================================================ */

typedef struct
{
    /*
     * Message type.
     */
    ipc_message_type_t type;

    /*
     * Sender process identifier.
     */
    int32_t sender_pid;

    /*
     * Sequence number.
     */
    uint32_t sequence;

    /*
     * Timestamp when the message was created.
     */
    uint64_t timestamp_ns;

} ipc_message_header_t;


/* ============================================================
 * RADAR IPC MESSAGE
 * ============================================================ */

typedef struct
{
    ipc_message_header_t header;

    radar_data_t data;

} ipc_radar_message_t;


/* ============================================================
 * TRACKING IPC MESSAGE
 * ============================================================ */

typedef struct
{
    ipc_message_header_t header;

    tracking_data_t data;

} ipc_tracking_message_t;


/* ============================================================
 * CONTROLLER IPC MESSAGE
 * ============================================================ */

typedef struct
{
    ipc_message_header_t header;

    control_command_t command;

} ipc_control_message_t;


/* ============================================================
 * SAFETY IPC MESSAGE
 * ============================================================ */

typedef struct
{
    ipc_message_header_t header;

    safety_status_t status;

} ipc_safety_message_t;


/* ============================================================
 * ACTUATOR IPC MESSAGE
 * ============================================================ */

typedef struct
{
    ipc_message_header_t header;

    control_command_t command;

} ipc_actuator_message_t;


/* ============================================================
 * HEARTBEAT MESSAGE
 * ============================================================ */

typedef struct
{
    ipc_message_header_t header;

    /*
     * PID of the process sending heartbeat.
     */
    int32_t process_pid;

    /*
     * Process health state.
     */
    process_health_t health;

} ipc_heartbeat_message_t;


/* ============================================================
 * PROCESS STATUS MESSAGE
 * ============================================================ */

typedef struct
{
    ipc_message_header_t header;

    process_status_t status;

} ipc_process_status_message_t;


/* ============================================================
 * SAFE STOP MESSAGE
 * ============================================================ */

typedef struct
{
    ipc_message_header_t header;

    /*
     * Reason for safe stop.
     */
    int32_t reason;

} ipc_safe_stop_message_t;


/* ============================================================
 * GENERAL IPC MESSAGE
 * ============================================================ */

typedef struct
{
    ipc_message_header_t header;

    union
    {
        radar_data_t radar;
        tracking_data_t tracking;
        control_command_t control;
        safety_status_t safety;
        process_status_t process;
        ipc_heartbeat_message_t heartbeat;

    } payload;

} ipc_message_t;


/* ============================================================
 * QNX NATIVE PULSE CODES
 * ============================================================ */

/*
 * These are used later for QNX event notification.
 */

#define ACC_PULSE_TIMER              1
#define ACC_PULSE_HEARTBEAT          2
#define ACC_PULSE_PROCESS_FAULT      3
#define ACC_PULSE_SAFE_STOP          4
#define ACC_PULSE_SHUTDOWN           5


/* ============================================================
 * SAFE STOP REASONS
 * ============================================================ */

#define SAFE_STOP_SENSOR_FAULT       1
#define SAFE_STOP_SENSOR_STALE       2
#define SAFE_STOP_CONTROLLER_DEADLINE 3
#define SAFE_STOP_PROCESS_FAULT      4
#define SAFE_STOP_IPC_FAULT          5
#define SAFE_STOP_EMERGENCY_DISTANCE 6
#define SAFE_STOP_INVALID_COMMAND    7


#endif /* IPC_TYPES_H */
