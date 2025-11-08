#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <Tarzan/lib/arm.h>
#include "lsm9ds1.h"
#include "smc_test.h"
#include "madgwick.h"

#include <stdio.h>

#define DEG2RAD (0.017453292519943295f)
#define RAD2DEG (57.29577951308232f)
#define MAG_OFFSET_X (0.0190f)
#define MAG_OFFSET_Y (0.0740f)
#define MAG_OFFSET_Z (0.3251f)


static const struct device *const mag_dev= DEVICE_DT_GET(DT_ALIAS(magnetometer));
static const struct device *const imu_dev= DEVICE_DT_GET(DT_ALIAS(imu));
static const struct device *const latte_panda_uart =
    DEVICE_DT_GET(DT_ALIAS(latte_panda_uart));

int main(void)
{
  struct joint mag;
  struct joint imu_data;
  /* latte panda uart ready check */
  if (!device_is_ready(latte_panda_uart)) {
    printk("LATTE PANDA UART device not ready");
  }

  /* enable usb for latte panda com */
  if (usb_enable(NULL)) {
    return 0;
  }

  if(!device_is_ready(imu_dev))
  {
    printk("Error in imu\n");
    return 1;
  }
  if(!device_is_ready(mag_dev))
  {
    printk("Error: Sensor device \"%s\" is not ready.\n",mag_dev->name);
        return 1;
  }

  printk("Found sensor: %s. Reading magnetometer data...\n",mag_dev->name);

 struct sensor_value odr_attr;

    odr_attr.val1 = 50; 
    odr_attr.val2 = 0;

    if (sensor_attr_set(mag_dev, SENSOR_CHAN_MAGN_XYZ,
                        SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
        printk("Cannot set sampling frequency for mag.\n");
        return 1;
    }
if (sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ,
                        SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
        printk("Cannot set sampling frequency for accelerometer.\n");
        return 1;
    }
if (sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ,
                        SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
        printk("Cannot set sampling frequency for gyro.\n");
        return 1;
    }

    printk("Accelerometer sampling frequency set to 50 Hz.\n");

uint32_t last_time =k_uptime_get_32();
if (calibrationlsm(imu_dev, &imu_data) != 0) {
        printk("Gyroscope calibration FAILED. Halting.\n");
        return 1; // Or handle this error as needed
    }
while (1) {
    uint32_t now = k_uptime_get_32();
    float dt = (float)(now - last_time)/1000.0f; //calculating dt in seconds
    last_time =now;
    
    //skip loop if dt is too large
    if (dt <= 0.0f||dt > 0.1f) { 
        k_msleep(1);
        continue;
    }
    struct sensor_value mag[3];
    struct sensor_value accel[3];
    struct sensor_value gyro[3];

    if (sensor_sample_fetch(mag_dev) < 0 || sensor_sample_fetch(imu_dev) < 0) {
        printk("Failed to fetch IMU sample.\n");
        continue;
    }

    //getting sensor values
    sensor_channel_get(mag_dev, SENSOR_CHAN_MAGN_X, &mag[0]);
    sensor_channel_get(mag_dev, SENSOR_CHAN_MAGN_Y, &mag[1]);
    sensor_channel_get(mag_dev, SENSOR_CHAN_MAGN_Z, &mag[2]);

    sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_X, &gyro[0]);
    sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Y, &gyro[1]);
    sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Z, &gyro[2]);

    sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_X, &accel[0]);
    sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Y, &accel[1]);
    sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Z, &accel[2]);

    printk("Accelerometer Values: %d, %d, %d \n",accel[0],accel[1],accel[2]);
    printk("Gyroscope Values: %d, %d, %d \n",gyro[0],gyro[1],gyro[2]);
    printk("Magnetometer Values: %d, %d, %d \n",mag[0],mag[1],mag[2]);
    printk("\n");

    //applying offset bias correction to prevent drift
    //change the orientation if needed after testing
        float ax = (float)sensor_value_to_double(&accel[0]);
        float ay = (float)sensor_value_to_double(&accel[1]);
        float az = (float)sensor_value_to_double(&accel[2]);

        // Replace the hard-coded GYRO_BIAS with the values from our struct
        float gx = (float)sensor_value_to_double(&gyro[0])-imu_data.gyro_offset[0];
        float gy = (float)sensor_value_to_double(&gyro[1])-imu_data.gyro_offset[1];
        float gz = (float)sensor_value_to_double(&gyro[2])-imu_data.gyro_offset[2];

        // You still need to calibrate the magnetometer!
        // The magnetometer calibration example I sent before would fill
        // imu_data.mag_offset[0..2]
        float mx = (float)sensor_value_to_double(&mag[0])-MAG_OFFSET_X;
        float my = (float)sensor_value_to_double(&mag[1])-MAG_OFFSET_Y;
        float mz = (float)sensor_value_to_double(&mag[2])-MAG_OFFSET_Z;

 printk("Before madgiwck filter: ax= %d, ay=%d, az=%d \n",ax,ay,az);
  printk("Before madgiwck filter: gx= %d, gy=%d, gz=%d \n",gx,gy,gz);
  printk("Before madgiwck filter: gx= %d, gy=%d, gz=%d \n",gx,gy,gz);
    
    printk("\n");
    //9dof madgwick filter
    imu_filter(ax, ay, az, gx, gy, gz, mx, my, mz, dt);

 printk("AFTER madgiwck filter: ax= %d, ay=%d, az=%d \n",ax,ay,az);
  printk("AFTER madgiwck filter: gx= %d, gy=%d, gz=%d \n",gx,gy,gz);
  printk("AFTER madgiwck filter: gx= %d, gy=%d, gz=%d \n",gx,gy,gz);

    //converting quaternion to euler
    float roll, pitch, yaw;
    eulerAngles(q_est, &roll, &pitch, &yaw);

    printk("Yaw = %.2f°, Pitch = %.2f°, Roll = %.2f°\n", yaw, pitch, roll);
   // lqr_yaw_correction(yaw); //from smc_test.c

    k_msleep(20);  //~50hz loop rate
}
 
  return 0;
}
