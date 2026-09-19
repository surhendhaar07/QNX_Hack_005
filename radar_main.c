/*
 * QNX Intelligent Adaptive Cruise Control
 *
 * Radar Process Main
 *
 * Executable:
 *     acc_radar
 *
 * Responsibilities:
 *     - Initialize Radar process
 *     - Create Radar worker thread
 *     - Configure QNX/POSIX real-time priority
 *     - Wait for worker termination
 *     - Perform clean shutdown
 *
 * Hardware:
 *     HC-SR04 ultrasonic sensor
 */

#include "radar.h"

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
    radar_context_t context;

    pthread_t radar_worker;

    int result;

    (void)argc;
    (void)argv;


    /* --------------------------------------------------------
     * PROCESS START
     * -------------------------------------------------------- */

    printf("\n");
    printf("+------------------------------------------------------------+\n");
    printf("| QNX ACC | RADAR       PROCESS                         |\n");
    printf("+------------------------------------------------------------+\n");

    printf(
        "Process  : acc_radar\n"
    );

    printf(
        "Period   : %d ms\n",
        RADAR_PERIOD_MS
    );

    printf(
        "Priority : %d\n",
        RADAR_PRIORITY
    );


    /* --------------------------------------------------------
     * INITIALIZE RADAR
     * -------------------------------------------------------- */

    result = radar_init(
        &context
    );

    if (result != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "RADAR MAIN ERROR: radar_init() failed\n"
        );

        return EXIT_FAILURE;
    }


    /* --------------------------------------------------------
     * CREATE RADAR WORKER THREAD
     * -------------------------------------------------------- */

    result = pthread_create(
        &radar_worker,
        NULL,
        radar_thread,
        &context
    );

    if (result != 0)
    {
        fprintf(
            stderr,
            "RADAR MAIN ERROR: pthread_create() failed "
            "(error=%d)\n",
            result
        );

        radar_shutdown(
            &context
        );

        return EXIT_FAILURE;
    }


    /* --------------------------------------------------------
     * CONFIGURE REAL-TIME PRIORITY
     * -------------------------------------------------------- */

    result = qnx_priority_configure_radar(
        radar_worker
    );

    if (result != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "RADAR MAIN ERROR: unable to configure "
            "Radar thread priority\n"
        );

        /*
         * Request the worker thread to stop.
         */
        context.running = 0;

        /*
         * Wait for the worker thread to terminate.
         */
        pthread_join(
            radar_worker,
            NULL
        );

        radar_shutdown(
            &context
        );

        return EXIT_FAILURE;
    }


    /* --------------------------------------------------------
     * DISPLAY REAL-TIME PRIORITY
     * -------------------------------------------------------- */

    qnx_priority_print(
        "Radar",
        radar_worker
    );


    /* --------------------------------------------------------
     * WAIT FOR RADAR WORKER
     * -------------------------------------------------------- */

    result = pthread_join(
        radar_worker,
        NULL
    );

    if (result != 0)
    {
        fprintf(
            stderr,
            "RADAR MAIN ERROR: pthread_join() failed "
            "(error=%d)\n",
            result
        );

        context.running = 0;

        radar_shutdown(
            &context
        );

        return EXIT_FAILURE;
    }


    /* --------------------------------------------------------
     * RADAR SHUTDOWN
     * -------------------------------------------------------- */

    result = radar_shutdown(
        &context
    );

    if (result != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "RADAR MAIN ERROR: radar_shutdown() failed\n"
        );

        return EXIT_FAILURE;
    }


    /* --------------------------------------------------------
     * PROCESS TERMINATION
     * -------------------------------------------------------- */

    printf(
        "RADAR: process terminated\n"
    );

    return EXIT_SUCCESS;
}
