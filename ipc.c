/*
 * QNX Intelligent Adaptive Cruise Control
 *
 * POSIX IPC implementation
 *
 * This module provides:
 *
 *     - POSIX message queues
 *     - IPC error handling
 *     - IPC timestamps
 *     - IPC latency calculation
 *     - Queue status
 *
 * No data is permanently stored.
 */

#include "ipc.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>


/* ============================================================
 * INTERNAL HELPER
 * ============================================================ */

static int ipc_get_timestamp_ns(
    uint64_t *timestamp_ns
)
{
    struct timespec ts;

    if (timestamp_ns == NULL)
    {
        return ACC_FAILURE;
    }

    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
    {
        perror("clock_gettime");

        return ACC_FAILURE;
    }

    *timestamp_ns =
        ((uint64_t)ts.tv_sec * 1000000000ULL) +
        (uint64_t)ts.tv_nsec;

    return ACC_SUCCESS;
}


/* ============================================================
 * MESSAGE SIZE
 * ============================================================ */

size_t ipc_message_size(void)
{
    return sizeof(ipc_message_t);
}


int ipc_validate_message_size(void)
{
    size_t required_size;
    size_t configured_size;

    required_size = sizeof(ipc_message_t);
    configured_size = ACC_MQ_MESSAGE_SIZE;

    if (required_size > configured_size)
    {
        fprintf(
            stderr,
            "IPC ERROR: message size too large\n"
            "Required : %zu bytes\n"
            "Queue    : %zu bytes\n",
            required_size,
            configured_size
        );

        return ACC_FAILURE;
    }

    return ACC_SUCCESS;
}


/* ============================================================
 * QUEUE INITIALIZATION
 * ============================================================ */

static int ipc_initialize_queue(
    ipc_queue_t *queue,
    const char *name
)
{
    if (queue == NULL || name == NULL)
    {
        return ACC_FAILURE;
    }

    memset(queue, 0, sizeof(*queue));

    queue->descriptor = (mqd_t)-1;

    /*
     * Store the queue name locally.
     */
    strncpy(
        queue->name,
        name,
        sizeof(queue->name) - 1
    );

    queue->name[sizeof(queue->name) - 1] = '\0';

    return ACC_SUCCESS;
}


/* ============================================================
 * OPEN RECEIVER
 * ============================================================ */

int ipc_open_receiver(
    ipc_queue_t *queue,
    const char *name
)
{
    struct mq_attr attr;

    if (queue == NULL || name == NULL)
    {
        fprintf(
            stderr,
            "IPC ERROR: invalid receiver arguments\n"
        );

        return ACC_FAILURE;
    }

    if (ipc_validate_message_size() != ACC_SUCCESS)
    {
        return ACC_FAILURE;
    }

    if (ipc_initialize_queue(queue, name) != ACC_SUCCESS)
    {
        return ACC_FAILURE;
    }

    memset(&attr, 0, sizeof(attr));

    /*
     * POSIX message queue configuration.
     */
    attr.mq_flags = 0;

    attr.mq_maxmsg = ACC_MQ_MAX_MESSAGES;

    attr.mq_msgsize = ACC_MQ_MESSAGE_SIZE;

    attr.mq_curmsgs = 0;

    /*
     * O_CREAT:
     *     create the queue if it doesn't exist.
     *
     * O_RDONLY:
     *     this descriptor is used for receiving.
     */
    queue->descriptor = mq_open(
        name,
        O_CREAT | O_RDONLY,
        0666,
        &attr
    );

    if (queue->descriptor == (mqd_t)-1)
    {
        fprintf(
            stderr,
            "IPC ERROR: mq_open receiver %s failed: %s\n",
            name,
            strerror(errno)
        );

        queue->is_open = 0;

        return ACC_FAILURE;
    }

    queue->is_open = 1;

    printf(
        "IPC: receiver queue opened: %s\n",
        name
    );

    return ACC_SUCCESS;
}


/* ============================================================
 * OPEN SENDER
 * ============================================================ */

int ipc_open_sender(
    ipc_queue_t *queue,
    const char *name
)
{
    if (queue == NULL || name == NULL)
    {
        fprintf(
            stderr,
            "IPC ERROR: invalid sender arguments\n"
        );

        return ACC_FAILURE;
    }

    if (ipc_validate_message_size() != ACC_SUCCESS)
    {
        return ACC_FAILURE;
    }

    if (ipc_initialize_queue(queue, name) != ACC_SUCCESS)
    {
        return ACC_FAILURE;
    }

    /*
     * Sender opens the queue in non-blocking mode.
     *
     * This is important for the real-time design:
     *
     * A full queue must not indefinitely block a
     * high-priority real-time thread.
     *
     * Instead, mq_send() returns EAGAIN and the
     * caller can enter its error-handling path.
     */
    queue->descriptor = mq_open(
        name,
        O_WRONLY | O_NONBLOCK
    );

    if (queue->descriptor == (mqd_t)-1)
    {
        fprintf(
            stderr,
            "IPC ERROR: mq_open sender %s failed: %s\n",
            name,
            strerror(errno)
        );

        queue->is_open = 0;

        return ACC_FAILURE;
    }

    queue->is_open = 1;

    printf(
        "IPC: sender queue opened: %s\n",
        name
    );

    return ACC_SUCCESS;
}


/* ============================================================
 * CLOSE QUEUE
 * ============================================================ */

int ipc_close(
    ipc_queue_t *queue
)
{
    if (queue == NULL)
    {
        return ACC_FAILURE;
    }

    if (!queue->is_open)
    {
        return ACC_SUCCESS;
    }

    if (mq_close(queue->descriptor) != 0)
    {
        fprintf(
            stderr,
            "IPC ERROR: mq_close(%s) failed: %s\n",
            queue->name,
            strerror(errno)
        );

        queue->is_open = 0;

        return ACC_FAILURE;
    }

    queue->descriptor = (mqd_t)-1;
    queue->is_open = 0;

    printf(
        "IPC: queue closed: %s\n",
        queue->name
    );

    return ACC_SUCCESS;
}


/* ============================================================
 * UNLINK QUEUE
 * ============================================================ */

int ipc_unlink(
    const char *name
)
{
    if (name == NULL)
    {
        return ACC_FAILURE;
    }

    if (mq_unlink(name) != 0)
    {
        /*
         * ENOENT means the queue does not exist.
         *
         * That is not considered a fatal cleanup error.
         */
        if (errno == ENOENT)
        {
            return ACC_SUCCESS;
        }

        fprintf(
            stderr,
            "IPC ERROR: mq_unlink(%s) failed: %s\n",
            name,
            strerror(errno)
        );

        return ACC_FAILURE;
    }

    printf(
        "IPC: queue removed: %s\n",
        name
    );

    return ACC_SUCCESS;
}


/* ============================================================
 * SEND MESSAGE
 * ============================================================ */

int ipc_send(
    ipc_queue_t *queue,
    const ipc_message_t *message)
{
    if (queue == NULL || message == NULL)
    {
        return ACC_FAILURE;
    }

    if (!queue->is_open)
    {
        return ACC_FAILURE;
    }

    if (mq_send(
            queue->descriptor,
            (const char *)message,
            sizeof(ipc_message_t),
            0) == -1)
    {
        fprintf(
            stderr,
            "IPC ERROR: mq_send(%s) failed: %s\n",
            queue->name,
            strerror(errno)
        );

        return ACC_FAILURE;
    }

    return ACC_SUCCESS;
}

/* ============================================================
 * RECEIVE MESSAGE
 * ============================================================ */

/* ============================================================
 * RECEIVE MESSAGE
 * ============================================================ */

static int ipc_receive_internal(
    ipc_queue_t *queue,
    ipc_message_t *message,
    unsigned int *receive_priority,
    int nonblocking)
{
    struct mq_attr attr;
    char *buffer;
    ssize_t received;
    int old_flags = 0;

    if (queue == NULL || message == NULL || !queue->is_open)
        return ACC_FAILURE;

    memset(message, 0, sizeof(*message));

    if (mq_getattr(queue->descriptor, &attr) != 0)
    {
        fprintf(stderr, "IPC ERROR: mq_getattr(%s) failed: %s\n",
                queue->name, strerror(errno));
        return ACC_FAILURE;
    }

    if (attr.mq_msgsize < (long)sizeof(ipc_message_t))
    {
        fprintf(stderr,
                "IPC ERROR: queue %s message size %ld is smaller than "
                "ipc_message_t size %zu\n",
                queue->name, attr.mq_msgsize, sizeof(ipc_message_t));
        return ACC_FAILURE;
    }

    if (nonblocking)
    {
        struct mq_attr set_attr = attr;
        set_attr.mq_flags |= O_NONBLOCK;

        if (mq_setattr(queue->descriptor, &set_attr, NULL) != 0)
        {
            fprintf(stderr, "IPC ERROR: mq_setattr(%s) failed: %s\n",
                    queue->name, strerror(errno));
            return ACC_FAILURE;
        }
        old_flags = (int)attr.mq_flags;
    }

    buffer = (char *)malloc((size_t)attr.mq_msgsize);
    if (buffer == NULL)
    {
        if (nonblocking)
        {
            struct mq_attr restore = attr;
            restore.mq_flags = old_flags;
            mq_setattr(queue->descriptor, &restore, NULL);
        }
        return ACC_FAILURE;
    }

    received = mq_receive(queue->descriptor, buffer,
                          (size_t)attr.mq_msgsize, receive_priority);

    if (nonblocking)
    {
        struct mq_attr restore = attr;
        restore.mq_flags = old_flags;
        mq_setattr(queue->descriptor, &restore, NULL);
    }

    if (received < 0)
    {
        int saved_errno = errno;
        free(buffer);

        if (nonblocking && (saved_errno == EAGAIN || saved_errno == EINTR))
            return ACC_FAILURE;

        fprintf(stderr, "IPC ERROR: %s mq_receive(%s) failed: %s\n",
                nonblocking ? "nonblocking" : "",
                queue->name, strerror(saved_errno));
        return ACC_FAILURE;
    }

    if ((size_t)received != sizeof(ipc_message_t))
    {
        fprintf(stderr,
                "IPC ERROR: invalid message size on %s: expected %zu, "
                "received %zd\n",
                queue->name, sizeof(ipc_message_t), received);
        free(buffer);
        return ACC_FAILURE;
    }

    memcpy(message, buffer, sizeof(*message));
    free(buffer);
    return ACC_SUCCESS;
}

int ipc_receive(
    ipc_queue_t *queue,
    ipc_message_t *message,
    unsigned int *receive_priority)
{
    return ipc_receive_internal(queue, message, receive_priority, 0);
}

int ipc_receive_nonblocking(
    ipc_queue_t *queue,
    ipc_message_t *message,
    unsigned int *receive_priority)
{
    return ipc_receive_internal(queue, message, receive_priority, 1);
}

/* ============================================================
 * MESSAGE HEADER INITIALIZATION
 * ============================================================ */

int ipc_initialize_header(
    ipc_message_header_t *header,
    ipc_message_type_t type,
    unsigned int sequence
)
{
    if (header == NULL)
    {
        return ACC_FAILURE;
    }

    memset(
        header,
        0,
        sizeof(*header)
    );

    header->type = type;

    /*
     * getpid() identifies the QNX process that generated
     * this message.
     */
    header->sender_pid = (int32_t)getpid();

    header->sequence = sequence;

    if (ipc_get_timestamp_ns(
            &header->timestamp_ns) != ACC_SUCCESS)
    {
        return ACC_FAILURE;
    }

    return ACC_SUCCESS;
}


/* ============================================================
 * IPC LATENCY
 * ============================================================ */

uint64_t ipc_calculate_latency_ns(
    uint64_t sender_timestamp_ns,
    uint64_t receiver_timestamp_ns
)
{
    /*
     * Protect against timestamp ordering errors.
     */
    if (receiver_timestamp_ns < sender_timestamp_ns)
    {
        return 0;
    }

    return receiver_timestamp_ns - sender_timestamp_ns;
}


/* ============================================================
 * QUEUE MESSAGE COUNT
 * ============================================================ */

long ipc_get_message_count(
    ipc_queue_t *queue
)
{
    struct mq_attr attr;

    if (queue == NULL || !queue->is_open)
    {
        return -1;
    }

    if (mq_getattr(
            queue->descriptor,
            &attr) != 0)
    {
        fprintf(
            stderr,
            "IPC ERROR: mq_getattr(%s) failed: %s\n",
            queue->name,
            strerror(errno)
        );

        return -1;
    }

    return attr.mq_curmsgs;
}


/* ============================================================
 * QUEUE DESCRIPTOR
 * ============================================================ */

mqd_t ipc_get_descriptor(
    ipc_queue_t *queue
)
{
    if (queue == NULL || !queue->is_open)
    {
        return (mqd_t)-1;
    }

    return queue->descriptor;
}
