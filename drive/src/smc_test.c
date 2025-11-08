#include "zephyr/toolchain.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/thread_stack.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/usb_device.h>

#include <Tarzan/lib/arm.h>
#include <Tarzan/lib/cobs.h>
#include <Tarzan/lib/drive.h>
#include <Tarzan/lib/sbus.h>
#include <math.h>
#include "smc_test.h"



static const float Iz = 1500.0f;   //rover yaw inertia
static const float a  = 0.1f;      //damping term
static const float q1 = 100.0f;    //wt yaw error
static const float q2 = 1.0f;      //wt yaw rate
static const float r  = 1.0f;      //wt control effort

static float K[2] = {0.0f, 0.0f};   // k1 k2
static int   lqr_init_done = 0;
static float prev_err = 0.0f;
static int   first_call = 1;
static int64_t prev_time_ms = 0;

static inline float wrap_pi(float ang)
{
    const float PI = 3.14159265359f;
    while (ang >  PI) ang -= 2.0f * PI;
    while (ang < -PI) ang += 2.0f * PI;
    return ang;
}

static void compute_lqr_gains(void)
{
    float b = 1.0f / Iz;

    float p22 = sqrtf(q2 / r);
    float term = (a - b * p22);
    float disc = term * term + 4.0f * q1;
    if (disc < 0.0f) disc = 0.0f;
    float p12 = (p22 * b - a + sqrtf(disc)) / 2.0f;

    K[0] = p12 / r;
    K[1] = p22 / r;

    lqr_init_done = 1;
}

// Replace `yaw_actual` with magnetometer-based yaw input

float lqr_yaw_correction(float yaw_actual)
{
    if (!lqr_init_done)
        compute_lqr_gains();
     

    //time in ms
    int64_t now_ms = k_uptime_get();

    float dt = 0.05f; // default
    if (!first_call)
        dt = (now_ms - prev_time_ms) / 1000.0f; // convert ms to s

    if (dt <= 0.0f)
        dt = 1e-3f; // safety clamp

    float yaw_desired = 0.0f;  //fixed yaw to 0
    float err = wrap_pi(yaw_actual - yaw_desired);

    float err_dot = 0.0f;
    if (!first_call)
        err_dot = (err - prev_err) / dt;

    float u = -(K[0] * err + K[1] * err_dot);

    prev_err = err;
    prev_time_ms = now_ms;
    first_call = 0;
    printk("U: %.2f\n",u);
    printk("Total: %.2f\n",u+yaw_actual);
    return u;
}
