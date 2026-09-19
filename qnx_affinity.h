#ifndef QNX_AFFINITY_H
#define QNX_AFFINITY_H

/*
 * QNX Intelligent Adaptive Cruise Control
 *
 * QNX CPU Affinity / Runmask Interface
 *
 * QNX-specific mechanism:
 *
 *     ThreadCtl(_NTO_TCTL_RUNMASK, ...)
 *
 * CPU affinity controls the set of processors on which
 * a thread is permitted to execute.
 *
 * The actual runmask used by the application will be
 * verified on the Raspberry Pi QNX target.
 */

#include <stdint.h>
#include <pthread.h>


/* ============================================================
 * RUNMASK TYPE
 * ============================================================ */

/*
 * QNX runmask representation used by this application.
 *
 * A bit represents an allowed processor.
 *
 * Example:
 *
 *     0x1  -> processor 0
 *     0x2  -> processor 1
 *     0x3  -> processors 0 and 1
 *
 * The valid mask depends on the target CPU configuration.
 */
typedef uint32_t qnx_runmask_t;


/* ============================================================
 * CURRENT THREAD
 * ============================================================ */

/*
 * Apply a runmask to the calling thread.
 *
 * Returns:
 *
 *      0 = success
 *     -1 = failure
 */
int qnx_affinity_set_current(
    qnx_runmask_t runmask
);


/* ============================================================
 * THREAD AFFINITY
 * ============================================================ */

/*
 * Apply CPU affinity to the specified thread.
 *
 * This function is provided as the common application
 * interface. The QNX implementation may require the
 * operation to be performed by the target thread itself.
 */
int qnx_affinity_set(
    pthread_t thread,
    qnx_runmask_t runmask
);


/* ============================================================
 * RUNMASK VALIDATION
 * ============================================================ */

/*
 * Validate that a runmask is not zero.
 *
 * A zero runmask would specify no processor.
 *
 * Returns:
 *
 *      0 = valid
 *     -1 = invalid
 */
int qnx_affinity_validate(
    qnx_runmask_t runmask
);


/* ============================================================
 * RUNMASK INFORMATION
 * ============================================================ */

/*
 * Return the number of processors represented by a runmask.
 */
unsigned int qnx_affinity_cpu_count(
    qnx_runmask_t runmask
);


/*
 * Print a runmask in hexadecimal form.
 */
void qnx_affinity_print(
    const char *task_name,
    qnx_runmask_t runmask
);


/* ============================================================
 * APPLICATION DEFAULT
 * ============================================================ */

/*
 * Configure the default ACC runmask.
 *
 * The default mask is taken from ACC_DEFAULT_RUNMASK.
 */
int qnx_affinity_set_default(void);


/*
 * Return the configured default ACC runmask.
 */
qnx_runmask_t qnx_affinity_get_default(void);


#endif /* QNX_AFFINITY_H */
