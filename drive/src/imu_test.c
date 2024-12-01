#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

const struct device *const lower = DEVICE_DT_GET(DT_ALIAS(imu_lower_joint));
const struct device *const upper = DEVICE_DT_GET(DT_ALIAS(imu_upper_joint));
const struct device *const base = DEVICE_DT_GET(DT_ALIAS(imu_turn_table));
const struct device *const end = DEVICE_DT_GET(DT_ALIAS(imu_pr));
# define M_PI  3.14159265358979323846
float accel_offset[3], gyro_offset[3];
float angle = 0;
float k = 0.96;
uint64_t prev_time = 0;
struct joint {
	float accel[3];
	float gyro[3];
	float pitch;
};
float true_acc[3] = {0, 0, 9.8};
float true_gyro[3] = {0, 0, 0};

int calibration(const struct device *dev) {
  struct sensor_value accel[3];
  struct sensor_value gyro[3];

  for (int i = 0; i < 1000; i++) {
    int rc = sensor_sample_fetch(dev);
  
    if (rc == 0)
      rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
    
    if (rc == 0)
      rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
    
    for (int i = 0; i < 3; i++) {
      accel_offset[i] += (sensor_value_to_double(&accel[i]) - true_acc[i]);
      gyro_offset[i] += (sensor_value_to_double(&gyro[i]) - true_gyro[i]);
    
    }
    k_sleep(K_MSEC(1));
  }
  for (int i = 0; i < 3; i++) {
    accel_offset[i] = accel_offset[i] / 1000.0;
    gyro_offset[i] = gyro_offset[i] / 1000.0;
  }
  printk("Calibration done\n");
  printk("accel_offset: %0.4f %0.4f %0.4f\n", accel_offset[0], accel_offset[1], accel_offset[2]);
  printk("gyroOffset: %0.4f %0.4f %0.4f\n", gyro_offset[0], gyro_offset[1], gyro_offset[2]);
  return 0;
}

static int process_mpu6050(const struct device *dev, int n) {
  struct sensor_value accel[3];
  struct sensor_value gyro[3];
  uint64_t current_time = k_uptime_get();
  float dt = (current_time - prev_time) / 1000.0;
  prev_time = current_time;
  float a[3], g[3];
  int rc = sensor_sample_fetch(dev);

  if (rc == 0)
    rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
  if (rc == 0)
    rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);

  a[0] = sensor_value_to_double(&accel[0]) - accel_offset[0];
  a[1] = sensor_value_to_double(&accel[1]) - accel_offset[1];
  a[2] = sensor_value_to_double(&accel[2]) - accel_offset[2];
  g[0] = sensor_value_to_double(&gyro[0]) - gyro_offset[0];
  g[1] = sensor_value_to_double(&gyro[1]) - gyro_offset[1];
  g[3] = sensor_value_to_double(&gyro[2]) - gyro_offset[2];

  if (rc == 0) {
    printk("%d accel % .4f % .4f % .4f m/s/s\t"
           " gyro % .4f % .4f % .4f rad/s\n",
           n, a[0], a[1], a[2], g[0], g[1], g[2]);
  float pitch_acc = (180 * atan2(-1 * a[0], sqrt(pow(a[1], 2) + pow(a[2], 2))) /M_PI);

  angle = k * (angle + (g[1]) * (dt)) + (1 - k) * pitch_acc;
  printk("ANGLE: % .2f\n", angle);
  } else
    printk("sample fetch/get failed: %d\n", rc);
  return rc;
}

int main() {
  printk("This is tarzan version %s\nFile: %s\n", GIT_BRANCH_NAME, __FILE__);

//  if (!device_is_ready(lower)) {
//    printk("Device %s is not ready\n", lower->name);
//    return 0;
//  }
//
//  if (!device_is_ready(upper)) {
//    printk("Device %s is not ready\n", upper->name);
//    return 0;
//  }

  if (!device_is_ready(base)) {
    printk("Device %s is not ready\n", lower->name);
    return 0;
  }

//  if (!device_is_ready(end)) {
//    printk("Device %s is not ready\n", upper->name);
//    return 0;
//  }
  printk("Calibrating...\n");
  if(calibration(base)){
    printk("Calibration failed for device %s\n",lower->name);
    return 0;
  }
  printk("Initialization completed successfully!\n");

  while (true) {
//    process_mpu6050(lower, 1);
//    process_mpu6050(upper, 2);
    process_mpu6050(base, 3);
//    process_mpu6050(end, 4);
    k_sleep(K_MSEC(20));
  }
}
