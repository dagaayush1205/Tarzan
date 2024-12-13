#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <Tarzan/lib/arm.h>

const struct device *const lower = DEVICE_DT_GET(DT_ALIAS(imu_lower_joint));
const struct device *const upper = DEVICE_DT_GET(DT_ALIAS(imu_upper_joint));
const struct device *const base = DEVICE_DT_GET(DT_ALIAS(mm_turn_table));
const struct device *const end = DEVICE_DT_GET(DT_ALIAS(imu_pitch_roll));

void process_bmm150(const struct device *dev) {
  struct sensor_value mag[3];
  int rc = sensor_sample_fetch(dev);

  if (rc == 0)
    rc = sensor_channel_get(dev, SENSOR_CHAN_MAGN_XYZ, mag);

  printf("%f %f %f\n", sensor_value_to_double(&mag[0]),
         sensor_value_to_double(&mag[1]), sensor_value_to_double(&mag[2]));
}
int main() {
  printk("This is tarzan version %s\nFile: %s\n", GIT_BRANCH_NAME, __FILE__);
  struct joint lowerIMU = {{0, 0, 0}, {0, 0, 0}, 0, 0, 0, {0, 0, 0}};
  struct joint upperIMU = {{0, 0, 0}, {0, 0, 0}, 0, 0, 0, {0, 0, 0}};
  struct joint endIMU = {{0, 0, 0}, {0, 0, 0}, 0, 0, 0, {0, 0, 0}};
  /*Device checks*/
  if (!device_is_ready(lower)) {
    printk("Device %s is not ready\n", lower->name);
    // return 0;
  }
  printk("ready1\n");
  if (!device_is_ready(upper)) {
    printk("Device %s is not ready\n", upper->name);
    // return 0;
  }
  // printk("Ready2\n");
  // if (!device_is_ready(base)) {
  //   printk("Device %s is not ready\n", base->name);
  //   // return 0;
  // }
  // printk("ready3");
  // if (!device_is_ready(end)) {
  //   printk("Device %s is not ready\n", upper->name);
  //   // return 0;
  // }

  /*Calibration */
  printk("Calibrating IMU %s\n", base->name);
  printk("Calibrating IMU %s\n", lower->name);
  if (calibration(lower, &lowerIMU)) {
    printk("Calibration failed for device %s\n", lower->name);
  }

  printk("Calibrating IMU %s\n", upper->name);
  if (calibration(upper, &upperIMU)) {
    printk("Calibration failed for device %s\n", upper->name);
  }

  // printk("Calibrating IMU %s\n", end->name);
  // if (calibration(end, &endIMU)) {
  //   printk("Calibration failed for device %s\n", end->name);
  // }
  //
  printk("Initialization completed successfully!\n");

  while (true) {

    process_mpu6050(lower, &lowerIMU);
    process_mpu6050(upper, &upperIMU);
    // process_mpu6050(end, &endIMU);
    // process_bmm150(base);
    k_sleep(K_MSEC(20));
  }
}
