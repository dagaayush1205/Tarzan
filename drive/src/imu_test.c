#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

const struct device *const lower = DEVICE_DT_GET(DT_ALIAS(imu_lower_joint));
const struct device *const upper = DEVICE_DT_GET(DT_ALIAS(imu_upper_joint));
const struct device *const base = DEVICE_DT_GET(DT_ALIAS(imu_turn_table));
//const struct device *const end = DEVICE_DT_GET(DT_ALIAS(imu_pitch_roll));

#define M_PI 3.14159265358979323846

float k = 0.90; // k here is tau

struct joint {
  float accel[3];
  float gyro[3];
  float pitch;
  float roll;
  uint64_t prev_time;
  float gyro_offset[3];
};
int calibration(const struct device *dev, struct joint *IMU) {
  struct sensor_value accel[3];
  struct sensor_value gyro[3];

  for (int i = 0; i < 1000; i++) {

    int rc = sensor_sample_fetch(dev);

    if (rc == 0)
      rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);

    if (rc == 0)
      rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);

    for (int i = 0; i < 3; i++)
      IMU->gyro_offset[i] += (sensor_value_to_double(&gyro[i]));
    k_sleep(K_MSEC(1));
  }
  for (int i = 0; i < 3; i++)
    IMU->gyro_offset[i] = IMU->gyro_offset[i] / 1000.0;

  printk("Calibration done\n");
  printk("gyroOffset: %0.4f %0.4f %0.4f\t", IMU->gyro_offset[0], IMU->gyro_offset[1],
         IMU->gyro_offset[2]);
  k_sleep(K_MSEC(10));
  return 0;
}


static int process_mpu6050(const struct device *dev, struct joint *IMU, int n) {

  struct sensor_value accel[3];
  struct sensor_value gyro[3];

  uint64_t current_time = k_uptime_get();

  float dt = (current_time - IMU->prev_time) / 1000.0;

  IMU->prev_time = current_time;


  int rc = sensor_sample_fetch(dev);

  if (rc == 0)
    rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
  if (rc == 0)
    rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);

  IMU->accel[0] = sensor_value_to_double(&accel[0]);
  IMU->accel[1] = sensor_value_to_double(&accel[1]);
  IMU->accel[2] = sensor_value_to_double(&accel[2]);
  IMU->gyro[0] = sensor_value_to_double(&gyro[0]) - IMU->gyro_offset[0];
  IMU->gyro[1] = sensor_value_to_double(&gyro[1]) - IMU->gyro_offset[1];
  IMU->gyro[2] = sensor_value_to_double(&gyro[2]) - IMU->gyro_offset[2];

  if (rc == 0) {

    float pitch_acc =
        (180 * atan2(-1 * IMU->accel[0], sqrt(pow(IMU->accel[1], 2) + pow(IMU->accel[2], 2))) / M_PI);
    float roll_acc =
        (180 * atan2(-1 * IMU->accel[1], sqrt(pow(IMU->accel[2], 2) + pow(IMU->accel[0], 2))) / M_PI);
    IMU->pitch = k * (IMU->pitch + (IMU->gyro[1]) * (dt)) + (1 - k) * pitch_acc;
    IMU->roll = k * (IMU->roll + (IMU->gyro[2]) * (dt)) + (1 - k) * roll_acc;
    printk("pitch: % .0f\t roll: % .0f, %d\t", IMU->pitch, IMU->roll, n);
  } else
    printk("sample fetch/get failed: %d\n", rc);
  return rc;
}

int main() {
  printk("This is tarzan version %s\nFile: %s\n", GIT_BRANCH_NAME, __FILE__);
  struct joint baseIMU;
  struct joint lowerIMU;
  struct joint upperIMU;
  // struct joint endIMU;
/*Device checks*/
  if (!device_is_ready(lower)) {
    printk("Device %s is not ready\n", lower->name);
    return 0;
  }
  printk("ready1\n");
  if (!device_is_ready(upper)) {
    printk("Device %s is not ready\n", upper->name);
    return 0;
  }
  printk("Ready2\n");
  if (!device_is_ready(base)) {
    printk("Device %s is not ready\n", base->name);
    return 0;
  }
  printk("ready3");
  // if (!device_is_ready(end)) {
  //   printk("Device %s is not ready\n", upper->name);
  //   return 0;
  // }


/*Calibration */
  printk("Calibrating IMU %s\n", base->name);
  if (calibration(base,&baseIMU)) {
    printk("Calibration failed for device %s\n", base->name);
    return 0;
  }

  printk("Calibrating IMU %s\n", lower->name);
  if(calibration(lower, &lowerIMU)) {
    printk("Calibration failed for device %s\n", lower->name);
  }

  printk("Calibrating IMU %s\n", upper->name);
  if(calibration(upper, &upperIMU)){
    printk("Calibration failed for device %s\n", upper->name);
  }

  // printk("Calibrating IMU %s\n", end->name);
  // if(calibration(end, &endIMU)){
  //   printk("Calibration failed for device %s\n", end->name);
  // }


  printk("Initialization completed successfully!\n");
  
  while (true) {
    
    process_mpu6050(lower, &lowerIMU, 1);
    process_mpu6050(upper, &upperIMU, 2);
    // process_mpu6050(end,&endIMU);
    process_mpu6050(base, &baseIMU, 3);
    k_sleep(K_MSEC(20));
  }
}
