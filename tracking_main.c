/*
 * QNX Intelligent Adaptive Cruise Control
 *
 * Tracking Process Main
 *
 * Executable:
 *     acc_tracking
 *
 * Responsibilities:
 *     - Initialize Tracking process
 *     - Create Tracking worker thread
 *     - Configure real-time priority
 *     - Wait for worker termination
 *     - Perform clean shutdown
 */

#include "tracking.h"

#include "../common/acc_config.h"
#include "../common/qnx_priority.h"

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>


/* ============================================================
 * MAIN
 * ============================================================ */

int main(
    int argc,
    char *argv[]
)
{
    tracking_context_t context;

    pthread_t tracking_worker;

    int result;


    (void)argc;
    (void)argv;


    /* --------------------------------------------------------
     * PROCESS START
     * -------------------------------------------------------- */

    printf("\n");
    printf("+------------------------------------------------------------+\n");
    printf("| QNX ACC | TRACKING    PROCESS                         |\n");
    printf("+------------------------------------------------------------+\n");

    printf(
        "Process  : acc_tracking\n"
    );

    printf(
        "Period   : %d ms\n",
        TRACKING_PERIOD_MS
    );

    printf(
        "Priority : %d\n",
        TRACKING_PRIORITY
    );


    /* --------------------------------------------------------
     * INITIALIZE TRACKING
     * -------------------------------------------------------- */

    result =
        tracking_init(
            &context
        );

    if (result != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "TRACKING MAIN ERROR: tracking_init() failed\n"
        );

        return EXIT_FAILURE;
    }


    /* --------------------------------------------------------
     * CREATE TRACKING WORKER THREAD
     * -------------------------------------------------------- */

    result =
        pthread_create(
            &tracking_worker,
            NULL,
            tracking_thread,
            &context
        );

    if (result != 0)
    {
        fprintf(
            stderr,
            "TRACKING MAIN ERROR: pthread_create() failed "
            "(error=%d)\n",
            result
        );

        tracking_shutdown(
            &context
        );

        return EXIT_FAILURE;
    }


    /* --------------------------------------------------------
     * CONFIGURE REAL-TIME PRIORITY
     * -------------------------------------------------------- */

    result =
        qnx_priority_configure_tracking(
            tracking_worker
        );

    if (result != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "TRACKING MAIN ERROR: unable to configure "
            "Tracking thread priority\n"
        );

        /*
         * Request worker termination.
         */
        context.running = 0;

        /*
         * Wait for worker termination.
         */
        pthread_join(
            tracking_worker,
            NULL
        );

        tracking_shutdown(
            &context
        );

        return EXIT_FAILURE;
    }


    /* --------------------------------------------------------
     * DISPLAY REAL-TIME PRIORITY
     * -------------------------------------------------------- */

    qnx_priority_print(
        "Tracking",
        tracking_worker
    );


    /* --------------------------------------------------------
     * WAIT FOR TRACKING THREAD
     * -------------------------------------------------------- */

    result =
        pthread_join(
            tracking_worker,
            NULL
        );

    if (result != 0)
    {
        fprintf(
            stderr,
            "TRACKING MAIN ERROR: pthread_join() failed "
            "(error=%d)\n",
            result
        );

        context.running = 0;

        tracking_shutdown(
            &context
        );

        return EXIT_FAILURE;
    }


    /* --------------------------------------------------------
     * SHUTDOWN
     * -------------------------------------------------------- */

    result =
        tracking_shutdown(
            &context
        );

    if (result != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "TRACKING MAIN ERROR: tracking_shutdown() failed\n"
        );

        return EXIT_FAILURE;
    }


    /* --------------------------------------------------------
     * PROCESS TERMINATION
     * -------------------------------------------------------- */

    printf(
        "TRACKING: process terminated\n"
    );

    return EXIT_SUCCESS;
}
