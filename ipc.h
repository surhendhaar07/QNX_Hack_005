#ifndef ACC_IPC_H
#define ACC_IPC_H

/*
 * QNX Intelligent Adaptive Cruise Control
 *
 * IPC interface
 *
 * Main IPC mechanism:
 *     POSIX message queues
 *
 * Data path:
 *
 * Radar
 *   -> Tracking
 *   -> Controller
 *   -> Safety
 *   -> Actuator
 *
 * The queues carry timestamps so that IPC latency
 * can be measured at the receiving process.
 */

#include <mqueue.h>
#include <stddef.h>

#include "acc_config.h"
#include "ipc_types.h"


/* ============================================================
 * IPC QUEUE HANDLE
 * ============================================================ */

/*
 * Each process keeps its own queue descriptor.
 *
 * A queue descriptor is local to the process.
 */
typedef struct
{
    mqd_t descriptor;

    /*
     * Queue name.
     */
    char name[64];

    /*
     * Indicates whether the descriptor is valid.
     */
    int is_open;

} ipc_queue_t;


/* ============================================================
 * QUEUE OPEN FUNCTIONS
 * ============================================================ */

/*
 * Open/create a receiving queue.
 *
 * The queue is created if it does not already exist.
 *
 * Receiver:
 *     blocking receive
 */
int ipc_open_receiver(
    ipc_queue_t *queue,
    const char *name
);


/*
 * Open a sending queue.
 *
 * The sender uses non-blocking operation so that a
 * full IPC queue cannot indefinitely block a real-time task.
 *
 * If the queue is full, ipc_send() reports an error.
 */
int ipc_open_sender(
    ipc_queue_t *queue,
    const char *name
);


/*
 * Close a queue descriptor.
 */
int ipc_close(
    ipc_queue_t *queue
);


/*
 * Remove a POSIX message queue from the system namespace.
 *
 * Normally called during controlled shutdown/cleanup.
 */
int ipc_unlink(
    const char *name
);


/* ============================================================
 * MESSAGE SEND / RECEIVE
 * ============================================================ */

/*
 * Send one ACC IPC message.
 *
 * The function also verifies that the message size is
 * compatible with the configured queue message size.
 */
int ipc_send(
    ipc_queue_t *queue,
    const ipc_message_t *message
);


/*
 * Receive one ACC IPC message.
 *
 * Blocking receive is used for normal process operation.
 *
 * The received message priority is returned through
 * receive_priority if it is not NULL.
 */
int ipc_receive(
    ipc_queue_t *queue,
    ipc_message_t *message,
    unsigned int *receive_priority
);

/*
 * Receive one message without blocking.
 *
 * Returns ACC_FAILURE when the queue is empty.
 */
int ipc_receive_nonblocking(
    ipc_queue_t *queue,
    ipc_message_t *message,
    unsigned int *receive_priority
);


/* ============================================================
 * IPC MESSAGE INITIALIZATION
 * ============================================================ */

/*
 * Initialize the common message header.
 *
 * timestamp_ns is generated from CLOCK_MONOTONIC.
 */
int ipc_initialize_header(
    ipc_message_header_t *header,
    ipc_message_type_t type,
    unsigned int sequence
);


/* ============================================================
 * IPC LATENCY
 * ============================================================ */

/*
 * Calculate IPC latency.
 *
 * receiver_timestamp_ns should be captured immediately after
 * receiving the message.
 */
uint64_t ipc_calculate_latency_ns(
    uint64_t sender_timestamp_ns,
    uint64_t receiver_timestamp_ns
);


/* ============================================================
 * IPC DIAGNOSTICS
 * ============================================================ */

/*
 * Return the configured message size.
 */
size_t ipc_message_size(void);


/*
 * Check whether the IPC message structure fits inside
 * the configured POSIX message queue message size.
 *
 * Returns:
 *     0  = valid
 *    -1  = invalid
 */
int ipc_validate_message_size(void);


/* ============================================================
 * LIVE IPC STATUS
 * ============================================================ */

/*
 * Return number of messages currently waiting in a queue.
 */
long ipc_get_message_count(
    ipc_queue_t *queue
);


/*
 * Return queue descriptor.
 */
mqd_t ipc_get_descriptor(
    ipc_queue_t *queue
);


#endif /* ACC_IPC_H */
