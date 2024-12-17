#include <kyvernitis/lib/kyvernitis.h>

#include <Tarzan/lib/arm.h>

/* wrapper around gpio pin set dt which returns updated position
 * param:
 * motor - dt spec for stepper motor
 * dir - clockwise or anti-clockwise
 * pos - stepper position*/
int Stepper_motor_write(const struct stepper_motor *motor, int dir, int pos) {
  if (dir == HIGH_PULSE) {
    gpio_pin_set_dt(&(motor->dir), 1);
    pos += 1; // clockwise
  } else {
    gpio_pin_set_dt(&(motor->dir), 0);
    pos -= 1; // anticlockwise
  }
  switch (pos & 0x03) {
  case 0:
    gpio_pin_set_dt(&(motor->step), 0);
    break;
  case 1:
    gpio_pin_set_dt(&(motor->step), 1);
    break;
  case 2:
    gpio_pin_set_dt(&(motor->step), 1);
    break;
  case 3:
    gpio_pin_set_dt(&(motor->step), 0);
    break;
  }
  return pos;
}

/*Calibration for imu
 * param:
 * dev: imu device
 * IMU: struct joint
 * returns: 0 if calibration succesfull*/

int calibration(const struct device *dev, struct joint *IMU) {
  struct sensor_value accel[3];
  struct sensor_value gyro[3];
  double true_gyro = 0;
  for (int i = 0; i < 1000; i++) {

    int rc = sensor_sample_fetch(dev);

    if (rc == 0)
      rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);

    if (rc == 0)
      rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);

    for (int i = 0; i < 3; i++)
      IMU->gyro_offset[i] += (sensor_value_to_double(&gyro[i]) - true_gyro);
    k_sleep(K_MSEC(1));
  }
  for (int i = 0; i < 3; i++)
    IMU->gyro_offset[i] = IMU->gyro_offset[i] / 1000.0;

  return 0;
}

/* process imu data and compute roll and pitch angle
 * param:
 * dev: imu device
 * IMU: struct joint
 * returns: 0 if succesfull*/
void process_mpu6050(const struct device *dev, struct joint *IMU) {

  struct sensor_value accel[3];
  struct sensor_value gyro[3];

  uint64_t current_time = k_uptime_get();

  double dt = (current_time - IMU->prev_time) / 1000.0;

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

    double pitch_acc = (atan2(-1 * IMU->accel[0], sqrt(pow(IMU->accel[1], 2) +
                                                       pow(IMU->accel[2], 2))));
    double roll_acc = (atan2(-1 * IMU->accel[1], sqrt(pow(IMU->accel[2], 2) +
                                                      pow(IMU->accel[0], 2))));
    IMU->pitch =
        TAU * (IMU->pitch + (IMU->gyro[1]) * (dt)) + (1 - TAU) * pitch_acc;
    IMU->roll =
        TAU * (IMU->roll + (IMU->gyro[2]) * (dt)) + (1 - TAU) * roll_acc;
  } else
    printk("sample fetch/get failed: %d\n", rc);
}
int update_proportional(float target_angel, float current_angel) {
  if (target_angel > current_angel + 0.5)
    return HIGH_PULSE;
  else if (target_angel < current_angel - 0.5)
    return LOW_PULSE;
  else
    return NULL;
}
