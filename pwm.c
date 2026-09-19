#include "pwm.h"

#include "../common/acc_config.h"

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <time.h>
#include <stdint.h>

#define SOFTWARE_PWM_PERIOD_US 10000U
#define SOFTWARE_PWM_STEP_US     500U

static int g_pwm_initialized = 0;
static int g_pwm_fd = -1;
static volatile int g_pwm_running = 0;
static double g_pwm_duty = 0.0;
static pthread_t g_pwm_thread;
static pthread_mutex_t g_pwm_mutex = PTHREAD_MUTEX_INITIALIZER;

static int pwm_gpio_write(int on)
{
    const char *value = on ? "on" : "off";
    ssize_t n;

    if (g_pwm_fd < 0)
    {
        return ACC_FAILURE;
    }

    n = write(g_pwm_fd, value, 2);
    return (n == 2) ? ACC_SUCCESS : ACC_FAILURE;
}

static void *pwm_worker(void *arg)
{
    (void)arg;

    while (g_pwm_running)
    {
        double duty;
        unsigned int on_steps;
        unsigned int step;

        pthread_mutex_lock(&g_pwm_mutex);
        duty = g_pwm_duty;
        pthread_mutex_unlock(&g_pwm_mutex);

        if (duty <= 0.0)
        {
            pwm_gpio_write(0);
            usleep(SOFTWARE_PWM_STEP_US);
            continue;
        }

        if (duty >= 100.0)
        {
            pwm_gpio_write(1);
            usleep(SOFTWARE_PWM_STEP_US);
            continue;
        }

        on_steps = (unsigned int)((duty / 100.0) *
                                  (SOFTWARE_PWM_PERIOD_US /
                                   SOFTWARE_PWM_STEP_US) + 0.5);

        for (step = 0;
             step < (SOFTWARE_PWM_PERIOD_US / SOFTWARE_PWM_STEP_US) &&
             g_pwm_running;
             ++step)
        {
            pwm_gpio_write(step < on_steps);
            usleep(SOFTWARE_PWM_STEP_US);
        }
    }

    pwm_gpio_write(0);
    return NULL;
}

int pwm_init(void)
{
    char path[64];

    if (g_pwm_initialized)
    {
        return ACC_SUCCESS;
    }

    snprintf(path, sizeof(path), "/dev/gpio/%d", L298N_ENA_GPIO);

    g_pwm_fd = open(path, O_RDWR);
    if (g_pwm_fd < 0)
    {
        fprintf(stderr,
                "[PWM] Cannot open ENA GPIO %d: %s\n",
                L298N_ENA_GPIO,
                strerror(errno));
        return ACC_FAILURE;
    }

    if (write(g_pwm_fd, "out", 3) != 3)
    {
        fprintf(stderr,
                "[PWM] Failed to configure ENA GPIO %d\n",
                L298N_ENA_GPIO);
        close(g_pwm_fd);
        g_pwm_fd = -1;
        return ACC_FAILURE;
    }

    g_pwm_duty = 0.0;
    pwm_gpio_write(0);

    g_pwm_running = 1;

    if (pthread_create(&g_pwm_thread, NULL, pwm_worker, NULL) != 0)
    {
        fprintf(stderr, "[PWM] Failed to create PWM worker thread\n");
        g_pwm_running = 0;
        pwm_gpio_write(0);
        close(g_pwm_fd);
        g_pwm_fd = -1;
        return ACC_FAILURE;
    }

    g_pwm_initialized = 1;

    printf("[PWM] QNX GPIO software-PWM backend initialized\n");
    printf("[PWM] ENA GPIO %d | period=%u us | step=%u us\n",
           L298N_ENA_GPIO,
           SOFTWARE_PWM_PERIOD_US,
           SOFTWARE_PWM_STEP_US);

    return ACC_SUCCESS;
}

void pwm_shutdown(void)
{
    if (!g_pwm_initialized)
    {
        return;
    }

    g_pwm_running = 0;
    pthread_join(g_pwm_thread, NULL);

    pwm_gpio_write(0);

    if (g_pwm_fd >= 0)
    {
        close(g_pwm_fd);
        g_pwm_fd = -1;
    }

    pthread_mutex_lock(&g_pwm_mutex);
    g_pwm_duty = 0.0;
    pthread_mutex_unlock(&g_pwm_mutex);

    g_pwm_initialized = 0;

    printf("[PWM] Shutdown\n");
}

int pwm_set_duty(double duty_percent)
{
    if (!g_pwm_initialized)
    {
        return ACC_FAILURE;
    }

    if (duty_percent < 0.0)
    {
        duty_percent = 0.0;
    }

    if (duty_percent > 100.0)
    {
        duty_percent = 100.0;
    }

    pthread_mutex_lock(&g_pwm_mutex);
    g_pwm_duty = duty_percent;
    pthread_mutex_unlock(&g_pwm_mutex);

    return ACC_SUCCESS;
}

int pwm_stop(void)
{
    if (!g_pwm_initialized)
    {
        return ACC_SUCCESS;
    }

    pthread_mutex_lock(&g_pwm_mutex);
    g_pwm_duty = 0.0;
    pthread_mutex_unlock(&g_pwm_mutex);

    return pwm_gpio_write(0);
}
