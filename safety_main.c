#include "safety.h"

#include "../common/acc_config.h"
#include "../common/qnx_priority.h"

#include <stdio.h>
#include <pthread.h>


int main(void)
{
    safety_context_t context;

    pthread_t safety_thread_id;

    /*
     * Initialize Safety process.
     */
    if (safety_init(&context) != 0)
    {
        fprintf(stderr,
                "[SAFETY_MAIN] "
                "Safety initialization failed\n");

        return 1;
    }

    /*
     * Create Safety worker thread.
     */
    if (pthread_create(
            &safety_thread_id,
            NULL,
            safety_thread,
            &context) != 0)
    {
        fprintf(stderr,
                "[SAFETY_MAIN] "
                "Failed to create safety thread\n");

        safety_shutdown(&context);

        return 1;
    }

    /*
     * Configure QNX real-time priority.
     *
     * Safety priority = 40.
     */
    if (qnx_priority_configure_safety(
            safety_thread_id) != 0)
    {
        fprintf(stderr,
                "[SAFETY_MAIN] "
                "Failed to configure "
                "safety priority\n");
    }

    printf("[SAFETY_MAIN] "
           "Safety process running\n");

    /*
     * Wait for Safety thread.
     */
    pthread_join(
        safety_thread_id,
        NULL);

    /*
     * Shutdown.
     */
    safety_shutdown(&context);

    printf("[SAFETY_MAIN] "
           "Safety process terminated\n");

    return 0;
}
