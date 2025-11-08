/*
 * Copyright (c) 2024 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <stdio.h>

/*
 * Get the device handle from the device tree alias.
 * This ensures we are using the correct sensor.
 */
static const struct device *const imu_dev = DEVICE_DT_GET(DT_ALIAS(imu-lsmds));

/**
 * @brief Helper function to convert a sensor_value to a double.
 */
/*static inline double sensor_value_to_double(const struct sensor_value *val)
{
    return (double)val->val1 + (double)val->val2 / 1000000.0;
}*/
static const struct device *const latte_panda_uart =
    DEVICE_DT_GET(DT_ALIAS(latte_panda_uart));
void main(void)
{
    
/* latte panda uart ready check */
  if (!device_is_ready(latte_panda_uart)) {
    printk("LATTE PANDA UART device not ready");
  }

  /* enable usb for latte panda com */
  if (usb_enable(NULL)) {
    return 0;
  }
    /* Check if the device was found */
    if (imu_dev == NULL) {
        printk("Error: Could not find sensor device node.\n");
        return;
    }

    /* Check if the device is ready to use */
    if (!device_is_ready(imu_dev)) {
        printk("Error: Sensor device is not ready.\n");
        return;
    }

    printk("Found sensor: %s. Reading Accel/Gyro data...\n", imu_dev->name);

    /* Loop forever, reading and printing data */
    while (1) {
        struct sensor_value accel_x, accel_y, accel_z;
        struct sensor_value gyro_x, gyro_y, gyro_z;

        /*
         * Fetch a new sample from the sensor. This fetches data for ALL
         * available channels on the LSM9DS1 (accel, gyro, and mag).
         */
        if (sensor_sample_fetch(imu_dev) < 0) {
            printk("Failed to fetch sample from device.\n");
            return;
        }

        /* Get the accelerometer data */
        sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_X, &accel_x);
        sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Y, &accel_y);
        sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Z, &accel_z);

        /* Get the gyroscope data */
        sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_X, &gyro_x);
        sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Y, &gyro_y);
        sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Z, &gyro_z);

        /* Print the accelerometer values in m/s^2 */
        printk("Accel (m/s^2): X=%.3f, Y=%.3f, Z=%.3f\n",
               sensor_value_to_double(&accel_x),
               sensor_value_to_double(&accel_y),
               sensor_value_to_double(&accel_z));

        /* Print the gyroscope values in rad/s */
        printk("Gyro (rad/s):  X=%.3f, Y=%.3f, Z=%.3f\n\n",
               sensor_value_to_double(&gyro_x),
               sensor_value_to_double(&gyro_y),
               sensor_value_to_double(&gyro_z));

        /* Wait for 1 second before the next reading */
        k_msleep(1000);
    }
}
