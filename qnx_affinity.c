/*
 * QNX Intelligent Adaptive Cruise Control
 *
 * QNX CPU Affinity / Runmask Implementation
 *
 * QNX-specific API:
 *
 *     ThreadCtl()
 *     _NTO_TCTL_RUNMASK
 *
 * No Raspberry Pi-specific GPIO or hardware API is used here.
 */

#include "qnx_affinity.h"

#include "acc_config.h"

#include <stdio.h>
#include <errno.h>
#include <string.h>


/*
 * QNX-specific headers.
 *
 * These are available when compiling for QNX.
 */
#ifdef __QNXNTO__

#include <sys/neutrino.h>
#include <sys/syspage.h>

#endif


/* ============================================================
 * RUNMASK VALIDATION
 * ============================================================ */

int qnx_affinity_validate(
    qnx_runmask_t runmask
)
{
    /*
     * A zero runmask would allow the thread to run on
     * no processor.
     */
    if (runmask == 0U)
    {
        fprintf(
            stderr,
            "AFFINITY ERROR: runmask cannot be zero\n"
        );

        return ACC_FAILURE;
    }

    return ACC_SUCCESS;
}


/* ============================================================
 * CPU COUNT
 * ============================================================ */

unsigned int qnx_affinity_cpu_count(
    qnx_runmask_t runmask
)
{
    unsigned int count = 0U;

    while (runmask != 0U)
    {
        if ((runmask & 1U) != 0U)
        {
            count++;
        }

        runmask >>= 1U;
    }

    return count;
}


/* ============================================================
 * SET CURRENT THREAD AFFINITY
 * ============================================================ */

int qnx_affinity_set_current(
    qnx_runmask_t runmask
)
{
#ifdef __QNXNTO__

    int result;

    if (qnx_affinity_validate(
            runmask) != ACC_SUCCESS)
    {
        return ACC_FAILURE;
    }

    /*
     * QNX ThreadCtl() applies thread control operations
     * to the calling thread.
     *
     * _NTO_TCTL_RUNMASK sets the processor runmask.
     *
     * The cast is required by the QNX ThreadCtl interface.
     */
    result = ThreadCtl(
        _NTO_TCTL_RUNMASK,
        (void *)(uintptr_t)runmask
    );

    if (result == -1)
    {
        fprintf(
            stderr,
            "AFFINITY ERROR: ThreadCtl("
            "_NTO_TCTL_RUNMASK) failed: %s\n",
            strerror(errno)
        );

        return ACC_FAILURE;
    }

    return ACC_SUCCESS;

#else

    /*
     * This branch exists only so the source tree can be
     * inspected/compiled on a non-QNX development host.
     *
     * It does NOT emulate CPU affinity.
     *
     * The real affinity operation is performed only by
     * the QNX target implementation above.
     */
    fprintf(
        stderr,
        "AFFINITY ERROR: CPU affinity requires QNX target\n"
    );

    (void)runmask;

    return ACC_FAILURE;

#endif
}


/* ============================================================
 * SET THREAD AFFINITY
 * ============================================================ */

int qnx_affinity_set(
    pthread_t thread,
    qnx_runmask_t runmask
)
{
    /*
     * QNX ThreadCtl(_NTO_TCTL_RUNMASK) operates on the
     * calling thread.
     *
     * Therefore this common function does not attempt to
     * manipulate another thread by pretending that the
     * operation is POSIX pthread_setaffinity_np().
     *
     * The intended use in this project is:
     *
     *     thread starts
     *          |
     *          v
     *     qnx_affinity_set_current()
     *
     * from inside that thread.
     */

    (void)thread;

    return qnx_affinity_set_current(
        runmask
    );
}


/* ============================================================
 * DEFAULT RUNMASK
 * ============================================================ */

qnx_runmask_t qnx_affinity_get_default(void)
{
    return (qnx_runmask_t)ACC_DEFAULT_RUNMASK;
}


int qnx_affinity_set_default(void)
{
    qnx_runmask_t runmask;

    runmask = qnx_affinity_get_default();

    return qnx_affinity_set_current(
        runmask
    );
}


/* ============================================================
 * RUNMASK REPORT
 * ============================================================ */

void qnx_affinity_print(
    const char *task_name,
    qnx_runmask_t runmask
)
{
    if (task_name == NULL)
    {
        task_name = "UNKNOWN";
    }

    printf(
        "AFFINITY: %-12s runmask=0x%08X CPUs=%u\n",
        task_name,
        (unsigned int)runmask,
        qnx_affinity_cpu_count(runmask)
    );
}
