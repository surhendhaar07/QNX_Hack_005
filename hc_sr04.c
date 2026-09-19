/*
 * QNX Intelligent Adaptive Cruise Control
 *
 * HC-SR04 Ultrasonic Sensor Driver
 *
 * Hardware:
 *
 *     HC-SR04 TRIG -> Raspberry Pi GPIO23
 *     HC-SR04 ECHO -> Raspberry Pi GPIO24
 *
 * QNX GPIO:
 *
 *     /dev/gpio/23
 *     /dev/gpio/24
 *
 * No simulated or random sensor values are used.
 */

#include "hc_sr04.h"

#include "../common/acc_config.h"
#include "../common/timing.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>


/* ============================================================
 * GPIO DEVICE PATHS
 * ============================================================ */

#define HC_SR04_TRIG_PATH "/dev/gpio/23"
#define HC_SR04_ECHO_PATH "/dev/gpio/24"


/* ============================================================
 * TIMING CONSTANTS
 * ============================================================ */

#define HC_SR04_TRIGGER_LOW_US       2U
#define HC_SR04_TRIGGER_HIGH_US     10U

#define HC_SR04_ECHO_START_TIMEOUT_US \
        HC_SR04_TIMEOUT_US

#define HC_SR04_ECHO_END_TIMEOUT_US \
        HC_SR04_TIMEOUT_US


/* ============================================================
 * INTERNAL STATE
 * ============================================================ */

static int g_sensor_initialized = 0;

static int g_trig_fd = -1;

static int g_echo_fd = -1;


/* ============================================================
 * MONOTONIC TIME
 * ============================================================ */

static uint64_t hc_sr04_now_ns(void)
{
    struct timespec ts;

    if (clock_gettime(
            CLOCK_MONOTONIC,
            &ts) != 0)
    {
        return 0;
    }

    return
        ((uint64_t)ts.tv_sec * 1000000000ULL) +
        (uint64_t)ts.tv_nsec;
}


/* ============================================================
 * MICROSECOND DELAY
 * ============================================================ */

static void hc_sr04_delay_us(
    uint32_t microseconds
)
{
    struct timespec requested;
    struct timespec remaining;

    requested.tv_sec =
        microseconds / 1000000U;

    requested.tv_nsec =
        (long)(microseconds % 1000000U) * 1000L;

    remaining = requested;

    while (nanosleep(
            &remaining,
            &remaining) != 0)
    {
        if (errno != EINTR)
        {
            break;
        }
    }
}


/* ============================================================
 * GPIO COMMAND
 *
 * rpi_gpio documentation:
 *
 *     echo -n out > /dev/gpio/23
 *     echo -n on  > /dev/gpio/23
 *     echo -n off > /dev/gpio/23
 *     echo -n in  > /dev/gpio/24
 * ============================================================ */

static int gpio_write_command(
    int fd,
    const char *command
)
{
    size_t length;
    ssize_t result;

    if (fd < 0 || command == NULL)
    {
        return ACC_FAILURE;
    }

    length = strlen(command);

    result =
        write(
            fd,
            command,
            length
        );

    if (result != (ssize_t)length)
    {
        return ACC_FAILURE;
    }

    return ACC_SUCCESS;
}


/* ============================================================
 * GPIO READ
 *
 * The QNX rpi_gpio resource manager exposes:
 *
 *     /dev/gpio/<gpio>
 *
 * Reading the node returns:
 *
 *     0
 *     1
 * ============================================================ */

static int gpio_read_level(
    const char *path,
    int *level
)
{
    int fd;
    char buffer[8];
    ssize_t bytes_read;

    if (path == NULL || level == NULL)
    {
        return ACC_FAILURE;
    }

    /*
     * The QNX rpi_gpio resource manager exposes each GPIO as
     * a resource-manager file.  A fresh open/read/close is used
     * here intentionally because it matches the known-good
     * shell operation:
     *
     *     cat /dev/gpio/24
     *
     * and avoids depending on a persistent file offset between
     * successive GPIO samples.
     */
    fd = open(path, O_RDONLY);

    if (fd < 0)
    {
        return ACC_FAILURE;
    }

    bytes_read =
        read(
            fd,
            buffer,
            sizeof(buffer) - 1
        );

    close(fd);

    if (bytes_read <= 0)
    {
        return ACC_FAILURE;
    }

    buffer[bytes_read] = '\0';

    if (buffer[0] == '0')
    {
        *level = 0;
        return ACC_SUCCESS;
    }

    if (buffer[0] == '1')
    {
        *level = 1;
        return ACC_SUCCESS;
    }

    return ACC_FAILURE;
}


/* ============================================================
 * WAIT FOR GPIO LEVEL
 * ============================================================ */

static int gpio_wait_for_level(
    const char *path,
    int required_level,
    uint64_t timeout_ns
)
{
    uint64_t start_ns;
    uint64_t current_ns;

    int level;

    start_ns =
        hc_sr04_now_ns();

    if (start_ns == 0)
    {
        return ACC_FAILURE;
    }

    while (1)
    {
        if (gpio_read_level(
                path,
                &level) != ACC_SUCCESS)
        {
            return ACC_FAILURE;
        }

        if (level == required_level)
        {
            return ACC_SUCCESS;
        }

        current_ns =
            hc_sr04_now_ns();

        if (current_ns == 0)
        {
            return ACC_FAILURE;
        }

        if ((current_ns - start_ns) >= timeout_ns)
        {
            return ACC_FAILURE;
        }

        /*
         * Small delay prevents the Radar process from
         * continuously consuming the CPU while polling.
         */
        hc_sr04_delay_us(1);
    }
}


/* ============================================================
 * GPIO INITIALIZATION
 * ============================================================ */

static int gpio_backend_init(void)
{
    /*
     * Open TRIG GPIO23.
     */
    g_trig_fd =
        open(
            HC_SR04_TRIG_PATH,
            O_RDWR
        );

    if (g_trig_fd < 0)
    {
        fprintf(
            stderr,
            "HC_SR04 ERROR: cannot open %s: %s\n",
            HC_SR04_TRIG_PATH,
            strerror(errno)
        );

        return ACC_FAILURE;
    }


    /*
     * Open ECHO GPIO24.
     */
    g_echo_fd =
        open(
            HC_SR04_ECHO_PATH,
            O_RDWR
        );

    if (g_echo_fd < 0)
    {
        fprintf(
            stderr,
            "HC_SR04 ERROR: cannot open %s: %s\n",
            HC_SR04_ECHO_PATH,
            strerror(errno)
        );

        close(g_trig_fd);
        g_trig_fd = -1;

        return ACC_FAILURE;
    }


    /*
     * GPIO23 = output.
     */
    if (gpio_write_command(
            g_trig_fd,
            "out") != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "HC_SR04 ERROR: GPIO23 output configuration failed\n"
        );

        close(g_echo_fd);
        close(g_trig_fd);

        g_echo_fd = -1;
        g_trig_fd = -1;

        return ACC_FAILURE;
    }


    /*
     * GPIO24 = input.
     */
    if (gpio_write_command(
            g_echo_fd,
            "in") != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "HC_SR04 ERROR: GPIO24 input configuration failed\n"
        );

        close(g_echo_fd);
        close(g_trig_fd);

        g_echo_fd = -1;
        g_trig_fd = -1;

        return ACC_FAILURE;
    }


    /*
     * Ensure TRIG is initially LOW.
     */
    if (gpio_write_command(
            g_trig_fd,
            "off") != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "HC_SR04 ERROR: cannot drive GPIO23 LOW\n"
        );

        close(g_echo_fd);
        close(g_trig_fd);

        g_echo_fd = -1;
        g_trig_fd = -1;

        return ACC_FAILURE;
    }

    return ACC_SUCCESS;
}


/* ============================================================
 * GPIO SHUTDOWN
 * ============================================================ */

static void gpio_backend_shutdown(void)
{
    if (g_trig_fd >= 0)
    {
        /*
         * Always leave TRIG LOW.
         */
        gpio_write_command(
            g_trig_fd,
            "off"
        );

        close(g_trig_fd);

        g_trig_fd = -1;
    }

    if (g_echo_fd >= 0)
    {
        close(g_echo_fd);

        g_echo_fd = -1;
    }
}


/* ============================================================
 * SENSOR INITIALIZATION
 * ============================================================ */

int hc_sr04_init(void)
{
    if (g_sensor_initialized)
    {
        return ACC_SUCCESS;
    }

    printf(
        "HC_SR04: TRIG GPIO = %d\n",
        HC_SR04_TRIG_GPIO
    );

    printf(
        "HC_SR04: ECHO GPIO = %d\n",
        HC_SR04_ECHO_GPIO
    );


    if (gpio_backend_init() != ACC_SUCCESS)
    {
        fprintf(
            stderr,
            "HC_SR04 ERROR: GPIO backend initialization failed\n"
        );

        g_sensor_initialized = 0;

        return ACC_FAILURE;
    }


    g_sensor_initialized = 1;


    printf(
        "HC_SR04: QNX GPIO backend initialized\n"
    );

    printf(
        "HC_SR04: GPIO%d -> TRIG\n",
        HC_SR04_TRIG_GPIO
    );

    printf(
        "HC_SR04: GPIO%d -> ECHO\n",
        HC_SR04_ECHO_GPIO
    );


    return ACC_SUCCESS;
}


/* ============================================================
 * SENSOR SHUTDOWN
 * ============================================================ */

int hc_sr04_shutdown(void)
{
    if (!g_sensor_initialized)
    {
        return ACC_SUCCESS;
    }

    gpio_backend_shutdown();

    g_sensor_initialized = 0;

    printf(
        "HC_SR04: shutdown complete\n"
    );

    return ACC_SUCCESS;
}


/* ============================================================
 * MEASURE DISTANCE
 * ============================================================ */

int hc_sr04_measure_distance(
    double *distance_cm
)
{
    uint64_t echo_start_ns;
    uint64_t echo_end_ns;
    uint64_t pulse_width_ns;

    double distance;


    if (distance_cm == NULL)
    {
        return ACC_FAILURE;
    }

    /*
     * Do not return a synthetic value.
     */
    *distance_cm = 0.0;


    if (!g_sensor_initialized)
    {
        return ACC_FAILURE;
    }


    /* ========================================================
     * TRIGGER PULSE
     * ======================================================== */

    /*
     * Ensure TRIG starts LOW.
     */
    if (gpio_write_command(
            g_trig_fd,
            "off") != ACC_SUCCESS)
    {
        return ACC_FAILURE;
    }


    hc_sr04_delay_us(
        HC_SR04_TRIGGER_LOW_US
    );


    /*
     * HIGH for 10 us.
     */
    if (gpio_write_command(
            g_trig_fd,
            "on") != ACC_SUCCESS)
    {
        return ACC_FAILURE;
    }


    hc_sr04_delay_us(
        HC_SR04_TRIGGER_HIGH_US
    );


    /*
     * End trigger pulse.
     */
    if (gpio_write_command(
            g_trig_fd,
            "off") != ACC_SUCCESS)
    {
        return ACC_FAILURE;
    }


    /* ========================================================
     * WAIT FOR ECHO HIGH
     * ======================================================== */

    if (gpio_wait_for_level(
            HC_SR04_ECHO_PATH,
            1,
            timing_us_to_ns(
                HC_SR04_ECHO_START_TIMEOUT_US
            )) != ACC_SUCCESS)
    {
        return ACC_FAILURE;
    }


    /*
     * Echo pulse starts.
     */
    echo_start_ns =
        hc_sr04_now_ns();

    if (echo_start_ns == 0)
    {
        return ACC_FAILURE;
    }


    /* ========================================================
     * WAIT FOR ECHO LOW
     * ======================================================== */

    if (gpio_wait_for_level(
            HC_SR04_ECHO_PATH,
            0,
            timing_us_to_ns(
                HC_SR04_ECHO_END_TIMEOUT_US
            )) != ACC_SUCCESS)
    {
        return ACC_FAILURE;
    }


    echo_end_ns =
        hc_sr04_now_ns();

    if (echo_end_ns <= echo_start_ns)
    {
        return ACC_FAILURE;
    }


    /* ========================================================
     * CALCULATE DISTANCE
     * ======================================================== */

    pulse_width_ns =
        echo_end_ns - echo_start_ns;


    /*
     * HC-SR04:
     *
     * distance(cm) = echo_time_us / 58
     */
    distance =
        ((double)pulse_width_ns / 1000.0) /
        58.0;


    /* ========================================================
     * VALIDATE
     * ======================================================== */

    if (hc_sr04_validate_distance(
            distance) != ACC_SUCCESS)
    {
        return ACC_FAILURE;
    }


    *distance_cm =
        distance;


    return ACC_SUCCESS;
}


/* ============================================================
 * COMPLETE SENSOR READ
 * ============================================================ */

int hc_sr04_read(
    radar_data_t *data,
    uint32_t sequence
)
{
    double distance_cm;

    int result;


    if (data == NULL)
    {
        return ACC_FAILURE;
    }


    memset(
        data,
        0,
        sizeof(*data)
    );


    data->sequence =
        sequence;

    data->timestamp_ns =
        hc_sr04_now_ns();

    data->status =
        SENSOR_TIMEOUT;


    /*
     * Actual hardware measurement.
     */
    result =
        hc_sr04_measure_distance(
            &distance_cm
        );


    if (result != ACC_SUCCESS)
    {
        data->timestamp_ns =
            hc_sr04_now_ns();

        data->status =
            SENSOR_TIMEOUT;

        /*
         * IMPORTANT:
         *
         * No fake distance is inserted.
         */
        data->distance_cm =
            0.0;

        return ACC_FAILURE;
    }


    /*
     * Validate actual result.
     */
    if (hc_sr04_validate_distance(
            distance_cm) != ACC_SUCCESS)
    {
        data->timestamp_ns =
            hc_sr04_now_ns();

        data->status =
            SENSOR_INVALID;

        data->distance_cm =
            0.0;

        return ACC_FAILURE;
    }


    /*
     * Valid actual measurement.
     */
    data->distance_cm =
        distance_cm;

    data->timestamp_ns =
        hc_sr04_now_ns();

    data->status =
        SENSOR_OK;


    return ACC_SUCCESS;
}


/* ============================================================
 * DISTANCE VALIDATION
 * ============================================================ */

int hc_sr04_validate_distance(
    double distance_cm
)
{
    if (distance_cm <
            ACC_MIN_VALID_DISTANCE_CM)
    {
        return ACC_FAILURE;
    }

    if (distance_cm >
            ACC_MAX_VALID_DISTANCE_CM)
    {
        return ACC_FAILURE;
    }

    return ACC_SUCCESS;
}
