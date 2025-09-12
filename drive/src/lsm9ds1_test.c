#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>

#include "lsm9ds1.h"
#include "arm.h"

#include <stdio.h>


static const struct device *const mag_dev = DEVICE_DT_GET(DT_ALIAS(magnetometer));
static const struct device *const imu_dev = DEVICE_DT_GET(DT_ALIAS(imu));
static const struct device *const latte_panda_uart =
    DEVICE_DT_GET(DT_ALIAS(latte_panda_uart));

int main(void)
{
  struct joint mag;
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
    printk("Error: Sensor device \"%s\" is not ready.\n", mag_dev->name);
		return 1;
  }

  printk("Found sensor: %s. Reading magnetometer data...\n",mag_dev->name);

 struct sensor_value odr_attr;

	odr_attr.val1 = 50; /* 50 Hz */
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


	while (1) {

		struct sensor_value mag[3];
		struct sensor_value accel[3];
		struct sensor_value gyro[3];

		/* Fetch a new sample from the sensor */
		if (sensor_sample_fetch(mag_dev) < 0) {
			printk("Failed to fetch sample from mag.\n");
			return 1;
		}
	  if (sensor_sample_fetch(imu_dev) < 0) {
			printk("Failed to fetch sample from IMU.\n");
			return 1;
		}


		/* Get Accelerometer Data */
		sensor_channel_get(mag_dev, SENSOR_CHAN_MAGN_X, &mag[0]);
		sensor_channel_get(mag_dev, SENSOR_CHAN_MAGN_Y, &mag[1]);
		sensor_channel_get(mag_dev, SENSOR_CHAN_MAGN_Z, &mag[2]);

		/* Get Gyroscope Data */
		sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_X, &gyro[0]);
		sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Y, &gyro[1]);
		sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Z, &gyro[2]);

    sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_X, &accel[0]);
		sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Y, &accel[1]);
		sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Z, &accel[2]);


    printk("Mag [T]: X=%.3f, Y=%.3f, Z=%.3f\n",
		       sensor_value_to_double(&mag[0]),
		       sensor_value_to_double(&mag[1]),
		       sensor_value_to_double(&mag[2]));

		printk("Accel [m/s^2]: X=%.3f, Y=%.3f, Z=%.3f\n",
		       sensor_value_to_double(&accel[0]),
		       sensor_value_to_double(&accel[1]),
		       sensor_value_to_double(&accel[2]));

		printk("Gyro  [rad/s]: X=%.3f, Y=%.3f, Z=%.3f\n\n",
		       sensor_value_to_double(&gyro[0]),
		       sensor_value_to_double(&gyro[1]),
		       sensor_value_to_double(&gyro[2]));

		
	}  
  return 0;
}
