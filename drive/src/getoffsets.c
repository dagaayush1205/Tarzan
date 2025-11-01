#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>
#include <stdio.h>
#include <float.h> // Required for FLT_MAX / FLT_MIN

/* --- Calibration Settings --- */
#define CAL_SAMPLES 1000 // Number of samples to average
#define CAL_DELAY_MS 20  // 50 Hz

/* --- Device Setup --- */
static const struct device *const mag_dev = DEVICE_DT_GET(DT_ALIAS(magnetometer));
static const struct device *const imu_dev = DEVICE_DT_GET(DT_ALIAS(imu));
static const struct device *const latte_panda_uart =
    DEVICE_DT_GET(DT_ALIAS(latte_panda_uart));


int main(void)
{
    /* --- Device Setup --- */
    if (!device_is_ready(latte_panda_uart)) { printk("UART not ready\n"); }
    if (usb_enable(NULL)) { return 0; }
    if(!device_is_ready(imu_dev)) { printk("IMU not ready\n"); return 1; }
    if(!device_is_ready(mag_dev)) { printk("Mag not ready\n"); return 1; }

    /* --- Set ODR (Sampling Frequency) --- */
    struct sensor_value odr_attr;
    odr_attr.val1 = 50; /* 50 Hz */
    odr_attr.val2 = 0;
    sensor_attr_set(mag_dev, SENSOR_CHAN_MAGN_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr);
    sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr);
    sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr);


    /* * =========================================
     * == PART 1: GYRO + ACCEL BIAS CALIBRATION
     * =========================================
     */
    printk("\n--- Gyro & Accel Calibration ---\n");
    printk("Place device ON A PERFECTLY FLAT, LEVEL SURFACE.\n");
    printk("KEEP IT PERFECTLY STILL for %d samples...\n", CAL_SAMPLES);
    k_sleep(K_SECONDS(5)); // Give user time
    printk("Starting calibration...\n");

    double sum_gx = 0.0, sum_gy = 0.0, sum_gz = 0.0;
    double sum_ax = 0.0, sum_ay = 0.0, sum_az = 0.0;
    struct sensor_value gyro[3];
    struct sensor_value accel[3];

    for (int i = 0; i < CAL_SAMPLES; i++) {
        if (sensor_sample_fetch(imu_dev) < 0) {
            i--; // Retry
            continue;
        }
        sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_X, &gyro[0]);
        sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Y, &gyro[1]);
        sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Z, &gyro[2]);

        sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_X, &accel[0]);
        sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Y, &accel[1]);
        sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Z, &accel[2]);

        sum_gx += sensor_value_to_double(&gyro[0]);
        sum_gy += sensor_value_to_double(&gyro[1]);
        sum_gz += sensor_value_to_double(&gyro[2]);

        sum_ax += sensor_value_to_double(&accel[0]);
        sum_ay += sensor_value_to_double(&accel[1]);
        sum_az += sensor_value_to_double(&accel[2]); // We average Z, but don't subtract it

        k_msleep(CAL_DELAY_MS);
    }

    float bias_gx = (float)(sum_gx / CAL_SAMPLES);
    float bias_gy = (float)(sum_gy / CAL_SAMPLES);
    float bias_gz = (float)(sum_gz / CAL_SAMPLES);
    
    // We only find the bias for X and Y, as Z should be ~1g (9.81)
    float bias_ax = (float)(sum_ax / CAL_SAMPLES); 
    float bias_ay = (float)(sum_ay / CAL_SAMPLES);
    // We assume the Z bias is the difference from 1G, but it's safer
    // to just correct X and Y, which are the ones causing Pitch/Roll error.

    printk("\n--- Gyro & Accel Calibration COMPLETE ---\n");
    printk("Copy these defines into your main.c:\n");
    printk("#define GYRO_BIAS_X (%.8ff)\n", bias_gx);
    printk("#define GYRO_BIAS_Y (%.8ff)\n", bias_gy);
    printk("#define GYRO_BIAS_Z (%.8ff)\n", bias_gz);
    printk("\n");
    printk("#define ACCEL_BIAS_X (%.8ff)\n", bias_ax);
    printk("#define ACCEL_BIAS_Y (%.8ff)\n", bias_ay);
    printk("#define ACCEL_BIAS_Z (0.0f) // Keep Z-bias 0 for now\n");
    printk("\n");


    /* * ==================================================
     * == PART 2: MAGNETOMETER HARD-IRON CALIBRATION
     * ==================================================
     */
    printk("--- Magnetometer Calibration ---\n");
    printk(">>> MOVE AWAY from laptops, monitors, and metal. <<<\n");
    printk("Start slowly rotating the device in all directions (figure-eight motion).\n");
    printk("Note the final values and reset the device.\n");
    k_sleep(K_SECONDS(5)); // Give user time to read

    float min_mx = FLT_MAX, max_mx = FLT_MIN;
    float min_my = FLT_MAX, max_my = FLT_MIN;
    float min_mz = FLT_MAX, max_mz = FLT_MIN;
    
    struct sensor_value mag[3];

    while (1) {
        if (sensor_sample_fetch(mag_dev) < 0) { continue; }
        sensor_channel_get(mag_dev, SENSOR_CHAN_MAGN_X, &mag[0]);
        sensor_channel_get(mag_dev, SENSOR_CHAN_MAGN_Y, &mag[1]);
        sensor_channel_get(mag_dev, SENSOR_CHAN_MAGN_Z, &mag[2]);

        float mx = (float)sensor_value_to_double(&mag[0]);
        float my = (float)sensor_value_to_double(&mag[1]);
        float mz = (float)sensor_value_to_double(&mag[2]);

        if (mx < min_mx) min_mx = mx; if (mx > max_mx) max_mx = mx;
        if (my < min_my) min_my = my; if (my > max_my) max_my = my;
        if (mz < min_mz) min_mz = mz; if (mz > max_mz) max_mz = mz;

        float offset_x = (max_mx + min_mx) / 2.0f;
        float offset_y = (max_my + min_my) / 2.0f;
        float offset_z = (max_mz + min_mz) / 2.0f;

        printk("\rOffsets: X=%.4f | Y=%.4f | Z=%.4f   ", offset_x, offset_y, offset_z);
        
        k_msleep(100); 
    }
 
  return 0; // Unreachable
}
