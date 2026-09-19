#include "actuator.h"

#include "../common/acc_config.h"
#include "../common/qnx_priority.h"

#include <stdio.h>
#include <pthread.h>


int main(void)
{
    actuator_context_t context;

    pthread_t actuator_thread_id;

    /*
     * Initialize Actuator process.
     */
    if (actuator_init(&context) != 0)
    {
        fprintf(stderr,
                "[ACTUATOR_MAIN] "
                "Actuator initialization failed\n");

        return 1;
    }

    /*
     * Create Actuator worker thread.
     */
    if (pthread_create(
            &actuator_thread_id,
            NULL,
            actuator_thread,
            &context) != 0)
    {
        fprintf(stderr,
                "[ACTUATOR_MAIN] "
                "Failed to create actuator thread\n");

        actuator_shutdown(&context);

        return 1;
    }

    /*
     * Configure QNX real-time priority.
     *
     * Actuator priority = 35.
     */
    if (qnx_priority_configure_actuator(
            actuator_thread_id) != 0)
    {
        fprintf(stderr,
                "[ACTUATOR_MAIN] "
                "Failed to configure "
                "actuator priority\n");
    }

    printf("[ACTUATOR_MAIN] "
           "Actuator process running\n");

    /*
     * Wait for worker thread.
     */
    pthread_join(
        actuator_thread_id,
        NULL);

    /*
     * Shutdown.
     */
    actuator_shutdown(&context);

    printf("[ACTUATOR_MAIN] "
           "Actuator process terminated\n");

    return 0;
}
