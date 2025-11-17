#include <Tarzan/lib/arm.h>
#include <Tarzan/lib/lsm9ds1.h>
#include <Tarzan/lib/madgwick.h>
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

/* wrapper around gpio pin set dt which returns updated position
 * param:
 * motor - dt spec for stepper motor
 * dir - clockwise or anti-clockwise
 * pos - stepper position*/
int Stepper_motor_write(const struct stepper *motor, int dir, int pos) {
  if (dir == HIGH_PULSE) {
    gpio_pin_set_dt(&(motor->dir), 1);
    pos += 1; // clockwise
  } else if (dir == LOW_PULSE) {
    gpio_pin_set_dt(&(motor->dir), 0);
    pos -= 1; // anticlockwise
  } else
    return pos;
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
int calibrate_gyro(const struct device *dev, struct joint *IMU) {
  struct sensor_value gyro[3];
  float true_gyro = 0;
  int rc;
  for (int i = 0; i < 1000; i++) {

    rc = sensor_sample_fetch(dev);

    if (rc == 0) {
      rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);

      for (int i = 0; i < 3; i++)
        IMU->gyro_offset[i] += (sensor_value_to_float(&gyro[i]) - true_gyro);
      k_sleep(K_MSEC(1));
    } else
      break;
  }
  if (rc == 0) {
    for (int i = 0; i < 3; i++)
      IMU->gyro_offset[i] = IMU->gyro_offset[i] / 1000.0f;
  } else
    return 1;

  return 0;
}

/* complementary filter to compute pitch & roll
 * param:
 * dev: imu device
 * IMU: struct joint
 * returns: 0 if succesfull*/
int complementary_filter(const struct device *dev, struct joint *IMU) {

  struct sensor_value accel[3];
  struct sensor_value gyro[3];

  uint64_t current_time = k_uptime_get();

  float dt = (current_time - IMU->prev_time) / 1000.0;

  int rc = sensor_sample_fetch(dev);

  if (rc == 0)
    rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
  if (rc == 0)
    rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);

  IMU->prev_time = current_time;
  IMU->accel[0] = sensor_value_to_float(&accel[0]);
  IMU->accel[1] = sensor_value_to_float(&accel[1]);
  IMU->accel[2] = sensor_value_to_float(&accel[2]);
  IMU->gyro[0] = sensor_value_to_float(&gyro[0]) - IMU->gyro_offset[0];
  IMU->gyro[1] = sensor_value_to_float(&gyro[1]) - IMU->gyro_offset[1];
  IMU->gyro[2] = sensor_value_to_float(&gyro[2]) - IMU->gyro_offset[2];

  if (rc == 0) {

    float pitch_acc = (atan2f(-1 * IMU->accel[0], sqrtf(powf(IMU->accel[1], 2) +
                                                      powf(IMU->accel[2], 2))));
    float roll_acc = (atan2f(-1 * IMU->accel[1], sqrtf(powf(IMU->accel[2], 2) +
                                                     powf(IMU->accel[0], 2))));
    IMU->pitch =
        TAU * (IMU->pitch + (IMU->gyro[1]) * (dt)) + (1 - TAU) * pitch_acc;
    IMU->roll =
        TAU * (IMU->roll + (IMU->gyro[2]) * (dt)) + (1 - TAU) * roll_acc;
  } else {
    return 1;
  }
  return 0;
}

/* proportional feedback update for stepper motor
 * param:
 * target_angel - angle of target position to reach
 * current_angel - angel of current position
 * returns: dir to move stepper */
enum StepperDirection update_proportional(float target_angel,
                                          float current_angel) {
  float error = fabsf(target_angel - current_angel);
  if (error <= 0.1f)
    return STOP_PULSE;
  if (target_angel > current_angel)
    return HIGH_PULSE;
  else // target_angel<current_angel
    return LOW_PULSE;
}

/* compute ypr using magnetometer + imu data
 * param:
 * mag_dev - magnetometer device
 * imu_dev - imu device
 * data - pointer to struct joint
 * returns: 0 if successfull*/
int madgwick_filter(const struct device *const mag_dev,
                    const struct device *const imu_dev, struct joint *data,
                    uint64_t dt) {

  if (dt <= 0.0f || dt > 0.1f) {
    k_msleep(1);
  }

  struct sensor_value Mag[3];
  struct sensor_value Accel[3];
  struct sensor_value Gyro[3];

  float accel[3], gyro[3], mag[3];

  int rc_mag = sensor_sample_fetch(mag_dev);
  int rc_imu = sensor_sample_fetch(imu_dev);
  if (rc_mag == 0 && rc_imu == 0) {
    sensor_channel_get(mag_dev, SENSOR_CHAN_MAGN_XYZ, Mag);
    sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, Gyro);
    sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_XYZ, Accel);
    for (int i = 1; i <= 3; i++) {
      accel[i] = sensor_value_to_float(&Accel[i]);
      gyro[i] = sensor_value_to_float(&Gyro[i]);
      mag[i] = sensor_value_to_float(&Mag[i]);
    }
    imu_filter(accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], mag[0],
               mag[1], mag[2], dt);

    eulerAngles(q_est, &data->roll, &data->pitch, &data->yaw);

    return 0;
  }
  return 1;
}

/* calibrate magnetometer
 * param:
 * dev - magnetometer device
 * data - pointer to struct joint
 * returns: 0 if successfull*/
int calibrate_magnetometer(const struct device *dev, struct joint *data) {
  struct sensor_value mag[3];
  int rc = 0;

  double mag_max[3] = {-DBL_MAX, -DBL_MAX, -DBL_MAX};
  double mag_min[3] = {DBL_MAX, DBL_MAX, DBL_MAX};

  for (int i = 1; i < 2000; i++) {
    rc = sensor_sample_fetch(dev);
    if (rc == 0) {
      rc = sensor_channel_get(dev, SENSOR_CHAN_MAGN_XYZ, mag);
      for (int i = 0; i < 3; i++) {
        double val = sensor_value_to_double(&mag[i]);
        if (val > mag_max[i]) {
          mag_max[i] = val;
        }
        if (val < mag_min[i]) {
          mag_min[i] = val;
        }
      }
    } else
      break;
    k_sleep(K_MSEC(1));
  }
  if (rc == 0) {
    for (int i = 0; i < 3; i++) {
      data->mag_offset[i] = (mag_max[i] + mag_min[i]) / 2.0;
    }
  } else
    return 1;

  return 0;
}
