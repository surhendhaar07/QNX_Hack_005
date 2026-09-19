/*
 * QNX Intelligent Adaptive Cruise Control
 *
 * Controller Process Main
 *
 * Executable:
 *     acc_controller
 *
 * Responsibilities:
 *     - Initialize Controller process
 *     - Create Controller worker thread
 *     - Configure real-time priority
 *     - Wait for worker termination
 *     - Perform clean shutdown
 */

#include "controller.h"

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
    controller_context_t context;

    pthread_t controller_worker;

    int result;


    (void)argc;
    (void)argv;


    /* --------------------------------------------------------
     * PROCESS START
     * -------------------------------------------------------- */

    printf("\n");
    printf("+------------------------------------------------------------+\n");
    printf("| QNX ACC | CONTROLLER  PROCESS                         |\n");
    printf("+------------------------------------------------------------+\n");

    printf(
        "Process  : acc_controller\n"
    );

    printf(
        "Period   : %d ms\n",
        CONTROLLER_PERIOD_MS
    );

    printf(
        "Deadline : %d ms\n",
        CONTROLLER_DEADLINE_MS
    );

    printf(
        "Priority : %d\n",
        CONTROLLER_PRIORITY
    );


    /* --------------------------------------------------------
     * INITIALIZE CONTROLLER
     * -------------------------------------------------------- */

    result =
        controller_init(
            &context
        );

    if (result != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "CONTROLLER MAIN ERROR: "
            "controller_init() failed\n"
        );

        return EXIT_FAILURE;
    }


    /* --------------------------------------------------------
     * CREATE CONTROLLER WORKER THREAD
     * -------------------------------------------------------- */

    result =
        pthread_create(
            &controller_worker,
            NULL,
            controller_thread,
            &context
        );

    if (result != 0)
    {
        fprintf(
            stderr,
            "CONTROLLER MAIN ERROR: "
            "pthread_create() failed "
            "(error=%d)\n",
            result
        );

        controller_shutdown(
            &context
        );

        return EXIT_FAILURE;
    }


    /* --------------------------------------------------------
     * CONFIGURE REAL-TIME PRIORITY
     * -------------------------------------------------------- */

    result =
        qnx_priority_configure_controller(
            controller_worker
        );

    if (result != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "CONTROLLER MAIN ERROR: "
            "unable to configure Controller "
            "thread priority\n"
        );

        /*
         * Request worker termination.
         */
        context.running = 0;

        /*
         * Wait for the worker thread to terminate.
         */
        pthread_join(
            controller_worker,
            NULL
        );

        controller_shutdown(
            &context
        );

        return EXIT_FAILURE;
    }


    /* --------------------------------------------------------
     * DISPLAY REAL-TIME PRIORITY
     * -------------------------------------------------------- */

    qnx_priority_print(
        "Controller",
        controller_worker
    );


    /* --------------------------------------------------------
     * WAIT FOR CONTROLLER THREAD
     * -------------------------------------------------------- */

    result =
        pthread_join(
            controller_worker,
            NULL
        );

    if (result != 0)
    {
        fprintf(
            stderr,
            "CONTROLLER MAIN ERROR: "
            "pthread_join() failed "
            "(error=%d)\n",
            result
        );

        context.running = 0;

        controller_shutdown(
            &context
        );

        return EXIT_FAILURE;
    }


    /* --------------------------------------------------------
     * SHUTDOWN
     * -------------------------------------------------------- */

    result =
        controller_shutdown(
            &context
        );

    if (result != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "CONTROLLER MAIN ERROR: "
            "controller_shutdown() failed\n"
        );

        return EXIT_FAILURE;
    }


    /* --------------------------------------------------------
     * PROCESS TERMINATION
     * -------------------------------------------------------- */

    printf(
        "CONTROLLER: process terminated\n"
    );

    return EXIT_SUCCESS;
}
