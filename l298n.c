#include "l298n.h"

#include "../common/acc_config.h"

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

static int g_l298n_initialized = 0;
static int g_in1_fd = -1;
static int g_in2_fd = -1;

static int gpio_configure_output(int fd)
{
    const char *mode = "out";
    ssize_t n;

    if (fd < 0)
    {
        return ACC_FAILURE;
    }

    n = write(fd, mode, 3);
    return (n == 3) ? ACC_SUCCESS : ACC_FAILURE;
}

static int gpio_write_state(int fd, int on)
{
    const char *value = on ? "on" : "off";
    ssize_t n;

    if (fd < 0)
    {
        return ACC_FAILURE;
    }

    n = write(fd, value, 2);
    return (n == 2) ? ACC_SUCCESS : ACC_FAILURE;
}

int l298n_init(void)
{
    char path[64];

    g_l298n_initialized = 0;

    snprintf(path, sizeof(path), "/dev/gpio/%d", L298N_IN1_GPIO);
    g_in1_fd = open(path, O_RDWR);
    if (g_in1_fd < 0)
    {
        fprintf(stderr, "[L298N] Cannot open IN1 GPIO %d: %s\n",
                L298N_IN1_GPIO, strerror(errno));
        return ACC_FAILURE;
    }

    snprintf(path, sizeof(path), "/dev/gpio/%d", L298N_IN2_GPIO);
    g_in2_fd = open(path, O_RDWR);
    if (g_in2_fd < 0)
    {
        fprintf(stderr, "[L298N] Cannot open IN2 GPIO %d: %s\n",
                L298N_IN2_GPIO, strerror(errno));
        close(g_in1_fd);
        g_in1_fd = -1;
        return ACC_FAILURE;
    }

    if (gpio_configure_output(g_in1_fd) != ACC_SUCCESS ||
        gpio_configure_output(g_in2_fd) != ACC_SUCCESS)
    {
        fprintf(stderr, "[L298N] Failed to configure direction GPIOs\n");
        l298n_shutdown();
        return ACC_FAILURE;
    }

    if (l298n_stop() != ACC_SUCCESS)
    {
        l298n_shutdown();
        return ACC_FAILURE;
    }

    g_l298n_initialized = 1;

    /*
     * Keep the outputs stopped during initialization.
     */
    l298n_stop();

    printf("[L298N] QNX GPIO backend initialized\n");
    printf("[L298N] IN1 GPIO %d -> /dev/gpio/%d\n",
           L298N_IN1_GPIO, L298N_IN1_GPIO);
    printf("[L298N] IN2 GPIO %d -> /dev/gpio/%d\n",
           L298N_IN2_GPIO, L298N_IN2_GPIO);
    printf("[L298N] Forward = IN1 ON, IN2 OFF\n");

    return ACC_SUCCESS;
}

void l298n_shutdown(void)
{
    if (g_in1_fd >= 0)
    {
        gpio_write_state(g_in1_fd, 0);
        close(g_in1_fd);
        g_in1_fd = -1;
    }

    if (g_in2_fd >= 0)
    {
        gpio_write_state(g_in2_fd, 0);
        close(g_in2_fd);
        g_in2_fd = -1;
    }

    g_l298n_initialized = 0;
    printf("[L298N] Shutdown\n");
}

int l298n_set_direction(motor_direction_t direction)
{
    if (!g_l298n_initialized)
    {
        return ACC_FAILURE;
    }

    switch (direction)
    {
        case MOTOR_FORWARD:
            if (gpio_write_state(g_in1_fd, 1) != ACC_SUCCESS ||
                gpio_write_state(g_in2_fd, 0) != ACC_SUCCESS)
            {
                return ACC_FAILURE;
            }
            return ACC_SUCCESS;

        case MOTOR_REVERSE:
            if (gpio_write_state(g_in1_fd, 0) != ACC_SUCCESS ||
                gpio_write_state(g_in2_fd, 1) != ACC_SUCCESS)
            {
                return ACC_FAILURE;
            }
            return ACC_SUCCESS;

        case MOTOR_STOP:
        default:
            return l298n_stop();
    }
}

int l298n_set_pwm(double duty_percent)
{
    /*
     * ENA PWM is implemented in pwm.c. This function remains
     * available for interface compatibility.
     */
    (void)duty_percent;
    return ACC_FAILURE;
}

int l298n_stop(void)
{
    int result = ACC_SUCCESS;

    if (g_in1_fd >= 0 &&
        gpio_write_state(g_in1_fd, 0) != ACC_SUCCESS)
    {
        result = ACC_FAILURE;
    }

    if (g_in2_fd >= 0 &&
        gpio_write_state(g_in2_fd, 0) != ACC_SUCCESS)
    {
        result = ACC_FAILURE;
    }

    return result;
}
