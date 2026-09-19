#include "../common/acc_config.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>
#include <errno.h>
#include <string.h>
#include <mqueue.h>

/*
 * Process information.
 */
typedef struct
{
    const char *name;
    const char *path;
    pid_t pid;

} process_info_t;


/*
 * ACC processes.
 *
 * Receiver processes are started before their
 * corresponding senders so that POSIX message
 * queues already exist.
 */
static process_info_t processes[] =
{
	{
		"Supervisor",
		"/usr/bin/acc_supervisor",
		-1
	},

	{
		"Actuator",
		"/usr/bin/acc_actuator",
		-1
	},

    {
        "Safety",
        "/usr/bin/acc_safety",
        -1
    },

	{
		"Controller",
		"/usr/bin/acc_controller",
		-1
	},

    {
        "Tracking",
        "/usr/bin/acc_tracking",
        -1
    },

    {
        "Radar",
        "/usr/bin/acc_radar",
        -1
    }


};


#define PROCESS_COUNT \
    (sizeof(processes) / sizeof(processes[0]))


static volatile sig_atomic_t
launcher_running = 1;


/*
 * Launcher signal handler.
 */
static void launcher_signal_handler(
    int signal_number)
{
    if ((signal_number == SIGTERM) ||
        (signal_number == SIGINT))
    {
        launcher_running = 0;
    }
}


/*
 * Start one process.
 */
static int launcher_validate_binaries(void)
{
    size_t i;

    for (i = 0; i < PROCESS_COUNT; ++i)
    {
        if (access(processes[i].path, X_OK) != 0)
        {
            fprintf(
                stderr,
                "[LAUNCHER] ERROR | Missing executable: %s (%s)\n",
                processes[i].path,
                strerror(errno)
            );
            return -1;
        }
    }

    return 0;
}


/*
 * Start one process.
 */
static int launcher_start_process(
    process_info_t *process)
{
    pid_t pid;

    if (process == NULL)
    {
        return -1;
    }

    pid = fork();

    if (pid < 0)
    {
        fprintf(stderr,
                "[LAUNCHER] Failed to fork %s: %s\n",
                process->name,
                strerror(errno));

        return -1;
    }

    if (pid == 0)
    {
        /*
         * Child process.
         */
        execl(process->path,
              process->path,
              (char *)NULL);

        /*
         * execl() returns only if execution
         * failed.
         */
        fprintf(stderr,
                "[LAUNCHER] Failed to execute %s: %s\n",
                process->path,
                strerror(errno));

        _exit(EXIT_FAILURE);
    }

    /*
     * Parent process.
     */
    process->pid = pid;

    printf("[LAUNCHER] START  | %-10s | PID %5d\n",
           process->name,
           (int)pid);

    return 0;
}


/*
 * Stop one process.
 */
static void launcher_stop_process(
    process_info_t *process)
{
    if (process == NULL)
    {
        return;
    }

    if (process->pid <= 0)
    {
        return;
    }

    printf("[LAUNCHER] STOP   | %-10s | PID %5d\n",
           process->name,
           (int)process->pid);

    /*
     * Request graceful termination.
     */
    if (kill(process->pid, SIGTERM) != 0)
    {
        if (errno != ESRCH)
        {
            fprintf(stderr,
                    "[LAUNCHER] Failed to stop %s: %s\n",
                    process->name,
                    strerror(errno));
        }
    }
}


/*
 * Stop all processes in reverse startup order.
 */
static void launcher_stop_all(void)
{
    size_t i;

    printf("[LAUNCHER] STOP   | Requesting ACC process shutdown\n");

    for (i = PROCESS_COUNT; i > 0; --i)
    {
        launcher_stop_process(
            &processes[i - 1]);
    }
}


/*
 * Wait for one process.
 */
static void launcher_reap_process(
    process_info_t *process)
{
    int status;
    pid_t result;

    if (process == NULL ||
        process->pid <= 0)
    {
        return;
    }

    result = waitpid(
        process->pid,
        &status,
        0);

    if (result < 0)
    {
        if (errno != ECHILD)
        {
            fprintf(stderr,
                    "[LAUNCHER] waitpid failed for %s: %s\n",
                    process->name,
                    strerror(errno));
        }

        return;
    }

    if (WIFEXITED(status))
    {
        printf("[LAUNCHER] EXIT   | %-10s | STATUS %d\n",
               process->name,
               WEXITSTATUS(status));
    }
    else if (WIFSIGNALED(status))
    {
        printf("[LAUNCHER] EXIT   | %-10s | SIGNAL %d\n",
               process->name,
               WTERMSIG(status));
    }

    process->pid = -1;
}

static void cleanup_acc_queues(void)
{
    mq_unlink("/acc_radar_q");
    mq_unlink("/acc_tracking_q");
    mq_unlink("/acc_controller_q");
    mq_unlink("/acc_safety_q");
    mq_unlink("/acc_supervisor_q");

    printf("[LAUNCHER] IPC    | Old POSIX message queues cleaned\n");
}

/*
 * Main launcher.
 */
int main(void)
{
    size_t i;

    /*
     * Install signal handlers.
     */
    signal(SIGTERM,
           launcher_signal_handler);

    signal(SIGINT,
           launcher_signal_handler);

    printf("\n");
    printf("╔══════════════════════════════════════════════╗\n");
    printf("║        QNX INTELLIGENT ACC SYSTEM          ║\n");
    printf("╠══════════════════════════════════════════════╣\n");
    printf("║ HC-SR04  → Tracking → Controller → Safety ║\n");
    printf("║                                      ↓      ║\n");
    printf("║                              L298N → Motor ║\n");
    printf("╠══════════════════════════════════════════════╣\n");
    printf("║ DISTANCE POLICY                             ║\n");
    printf("║   >= 30 cm       NORMAL OPERATION           ║\n");
    printf("║   15–<30 cm      SLOW / REDUCED SPEED       ║\n");
    printf("║   <= 15 cm       EMERGENCY BRAKE            ║\n");
    printf("╚══════════════════════════════════════════════╝\n");
    printf("\n");

    cleanup_acc_queues();

    /*
     * Fail before starting any process if one required
     * executable is missing.  This prevents a partial system
     * where Tracking starts without Controller and then reports
     * /acc_tracking_q as missing.
     */
    if (launcher_validate_binaries() != 0)
    {
        fprintf(
            stderr,
            "[LAUNCHER] ERROR  | Startup aborted: build/install all ACC executables first.\n"
        );
        return EXIT_FAILURE;
    }

    printf("[SYSTEM] START  | Launching QNX ACC processes...\n");
    printf("[SYSTEM] HW     | HC-SR04 sensor + GPIO/L298N actuator enabled\n");
    printf("\n");

    /*
     * Start processes sequentially.
     */
    for (i = 0; i < PROCESS_COUNT; ++i)
    {
        if (launcher_start_process(
                &processes[i]) != 0)
        {
            fprintf(stderr,
                    "[LAUNCHER] "
                    "Startup failed at %s\n",
                    processes[i].name);

            launcher_stop_all();

            return EXIT_FAILURE;
        }

        /*
         * Small startup interval.
         *
         * The receiver needs time to create its
         * POSIX message queue before the sender
         * attempts to open it.
         */
        usleep(100000);
    }

    printf("\n");
    printf("┌──────────────────────────────────────────────┐\n");
    printf("│              ACC SYSTEM ONLINE               │\n");
    printf("├──────────────────────────────────────────────┤\n");
    printf("│ Radar      : RUNNING                         │\n");
    printf("│ Tracking   : RUNNING                         │\n");
    printf("│ Controller : RUNNING                         │\n");
    printf("│ Safety     : RUNNING                         │\n");
    printf("│ Actuator   : RUNNING                         │\n");
    printf("│ Supervisor : RUNNING                         │\n");
    printf("└──────────────────────────────────────────────┘\n");
    printf("[SYSTEM] READY  | All ACC processes started\n");
    printf("[SYSTEM] RUN    | Live monitoring active | Press Ctrl+C to stop\n");
    printf("\n");

    /*
     * Monitor child processes.
     */
    while (launcher_running)
    {
        int status;
        pid_t result;

        result = waitpid(
            -1,
            &status,
            WNOHANG);

        if (result > 0)
        {
            /*
             * Identify the terminated process.
             */
            for (i = 0; i < PROCESS_COUNT; ++i)
            {
                if (processes[i].pid == result)
                {
                    printf("[LAUNCHER] EXIT   | %s terminated\n",
                           processes[i].name);

                    processes[i].pid = -1;

                    /*
                     * A critical process failure
                     * causes the complete ACC system
                     * to enter shutdown.
                     */
                    launcher_running = 0;

                    break;
                }
            }
        }
        else if (result < 0)
        {
            if (errno != EINTR)
            {
                fprintf(stderr,
                        "[LAUNCHER] waitpid error: %s\n",
                        strerror(errno));

                launcher_running = 0;
            }
        }

        usleep(100000);
    }

    printf("\n");
    printf("[SYSTEM] STOP   | System shutdown requested\n");

    /*
     * Request termination of all remaining
     * processes.
     */
    launcher_stop_all();

    /*
     * Reap children.
     */
    for (i = 0; i < PROCESS_COUNT; ++i)
    {
        if (processes[i].pid > 0)
        {
            launcher_reap_process(
                &processes[i]);
        }
    }

    printf("\n");
    printf("┌──────────────────────────────────────────────┐\n");
    printf("│              ACC SYSTEM OFFLINE              │\n");
    printf("└──────────────────────────────────────────────┘\n");

    return EXIT_SUCCESS;
}
