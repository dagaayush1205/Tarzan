#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/sensor.h>

static const struct device *imu_joint = DEVICE_DT_GET(DT_ALIAS(imu_joint));

static const struct device *const pico_uart = DEVICE_DT_GET(DT_ALIAS(pico_uart));

#define M_PI 3.14159265358979323846

float accel_offset[3], gyro_offset[3];
float angle_pitch = 0.0, angle_roll = 0.0;
float k = 0.90; // k here is tau
float target_angle = -45;
uint64_t prev_time = 0;

float true_acc[3] = {0, 0, -9.8};
float true_gyro[3] = {0, 0, 0};

int calibration(const struct device *dev);
int process_pitch_roll(const struct device *dev);

int main(void)
{
    if (!device_is_ready(pico_uart)) {
    printk("Uart device not ready\n");
  }
    if (!device_is_ready(imu_joint) {
        printk("IMU not ready.\n");
    }
    if (calibration(imu_joint)) {
    printk("Calibration failed for device %s\n", base->name);
  }
}
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
  printk("accel_offset: %f %f %f\n", accel_offset[0], accel_offset[1],
         accel_offset[2]);
  printk("gyroOffset: %f %f %f\n", gyro_offset[0], gyro_offset[1],
         gyro_offset[2]);

  return 0;
}

int process_pitch_roll(const struct device *dev, struct joint *IMU) {

  struct sensor_value accel[3];
  struct sensor_value gyro[3];

  uint64_t current_time = k_uptime_get();

  double dt = (current_time - IMU->prev_time) / 1000.0;

  int rc = sensor_sample_fetch(dev);

  if (rc == 0)
    rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
  if (rc == 0)
    rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);

  IMU->prev_time = current_time;
  IMU->accel[0] = sensor_value_to_double(&accel[0]);
  IMU->accel[1] = sensor_value_to_double(&accel[1]);
  IMU->accel[2] = sensor_value_to_double(&accel[2]);
  IMU->gyro[0] = sensor_value_to_double(&gyro[0]) - IMU->gyro_offset[0];
  IMU->gyro[1] = sensor_value_to_double(&gyro[1]) - IMU->gyro_offset[1];
  IMU->gyro[2] = sensor_value_to_double(&gyro[2]) - IMU->gyro_offset[2];

  // printk("%03.3f %03.3f %03.3f | %03.3f %03.3f %03.3f\n", IMU->accel[0],
  // IMU->accel[1], IMU->accel[2], IMU->gyro[0], IMU->gyro[1], IMU->gyro[2]);

  if (rc == 0) {

    double pitch_acc = (atan2(-1 * IMU->accel[0], sqrt(pow(IMU->accel[1], 2) +
                                                       pow(IMU->accel[2], 2))));
    double roll_acc = (atan2(-1 * IMU->accel[1], sqrt(pow(IMU->accel[2], 2) +
                                                      pow(IMU->accel[0], 2))));
    IMU->pitch =
        TAU * (IMU->pitch + (IMU->gyro[1]) * (dt)) + (1 - TAU) * pitch_acc;
    IMU->roll =
        TAU * (IMU->roll + (IMU->gyro[2]) * (dt)) + (1 - TAU) * roll_acc;
  } else {
    return 1;
  }
  return 0;
}

