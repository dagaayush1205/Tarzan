#include <kyvernitis/lib/kyvernitis.h>
#include <float.h>
#include <Tarzan/lib/arm.h>
#include <math.h>

/* wrapper around gpio pin set dt which returns updated position
 * param:
 * motor - dt spec for stepper motor
 * dir - clockwise or anti-clockwise
 * pos - stepper position*/
int Stepper_motor_write(const struct stepper_motor *motor, int dir, int pos) {
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
int calibration(const struct device *dev, struct joint *IMU) {
  struct sensor_value accel[3];
  struct sensor_value gyro[3];
  double true_gyro = 0;
  int rc;
  for (int i = 0; i < 1000; i++) {

    rc = sensor_sample_fetch(dev);

    if (rc == 0)
      rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);

    if (rc == 0)
      rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);

    for (int i = 0; i < 3; i++)
      IMU->gyro_offset[i] += (sensor_value_to_double(&gyro[i]) - true_gyro);
    k_sleep(K_MSEC(1));
  }
  if (rc == 0) {
    for (int i = 0; i < 3; i++)
      IMU->gyro_offset[i] = IMU->gyro_offset[i] / 1000.0;
  } else
    return 1;

  return 0;
}

int calibrationlsm(const struct device *dev, struct joint *IMU) {
  //struct sensor_value mag[3];
  struct sensor_value accel[3];
  struct sensor_value gyro[3];
  double true_gyro = 0;
  int rc;
  for (int i = 0; i < 1000; i++) {
    rc = sensor_sample_fetch(dev);
    if (rc == 0)
      rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
    if (rc == 0)
      rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
    for (int i = 0; i < 3; i++)
      IMU->gyro_offset[i] += (sensor_value_to_double(&gyro[i]) - true_gyro);
    k_sleep(K_MSEC(1));
  }
  if (rc == 0) {
    for (int i = 0; i < 3; i++)
      IMU->gyro_offset[i] = IMU->gyro_offset[i] / 1000.0;
  } else
    return 1;
  return 0;
}

//RECHECK THIS FUNCTION
int calibrate_magnetometer(const struct device *dev, struct joint *imu_data, int32_t duration_ms)
{
    struct sensor_value mag[3];
    int rc = 0;
    int64_t start_time = k_uptime_get();

    double mag_max[3] = {-DBL_MAX, -DBL_MAX, -DBL_MAX};
    double mag_min[3] = {DBL_MAX, DBL_MAX, DBL_MAX};

    imu_data->mag_offset[0] = 0.0;
    imu_data->mag_offset[1] = 0.0;
    imu_data->mag_offset[2] = 0.0;

    printk("Starting magnetometer calibration...\n");
    printk("Please rotate the rover slowly for %d seconds.\n", duration_ms / 1000);
    printk("For best results, spin it in 1-2 full circles AND tilt it up/down.\n");
    while (k_uptime_get() - start_time < duration_ms) {
        rc = sensor_sample_fetch(dev);
        if (rc != 0) {
            printk("Error: Failed to fetch mag sample. rc = %d\n", rc);
            return rc;
        }
        rc = sensor_channel_get(dev, SENSOR_CHAN_MAGN_XYZ, mag);
        if (rc != 0) {
            printk("Error: Failed to get magnetometer data. rc = %d\n", rc);
            return rc;
        }
        for (int i = 0; i < 3; i++) {
            double val = sensor_value_to_double(&mag[i]);
            if (val > mag_max[i]) {
                mag_max[i] = val;
            }
            if (val < mag_min[i]) {
                mag_min[i] = val;
            }
        }
        k_sleep(K_MSEC(50)); 
    }

    for (int i = 0; i < 3; i++) {
        imu_data->mag_offset[i] = (mag_max[i] + mag_min[i]) / 2.0;
    }

    printk("Magnetometer calibration complete.\n");
    printk("Offsets: X=%.4f, Y=%.4f, Z=%.4f\n",
           imu_data->mag_offset[0],
           imu_data->mag_offset[1],
           imu_data->mag_offset[2]);

    return 0;
}


/* complementary filter to compute pitch & roll
 * param:
 * dev: imu device
 * IMU: struct joint
 * returns: 0 if succesfull*/
int process_pitch_roll(const struct imu_data *data, struct joint *IMU) {

  double a[3], g[3];

  uint64_t current_time = k_uptime_get();

  double dt = (current_time - IMU->prev_time) / 1000.0;

    IMU->prev_time = current_time;
    a[0] = data->accel[0];
    a[1] = data->accel[1]; 
    a[2] = data->accel[2];
    g[0] = data->gyro[0] - data->gyro_offset[0];
    g[1] = data->gyro[1] - data->gyro_offset[1];
    g[2] = data->gyro[2] - data->gyro_offset[2];

    double pitch_acc = (atan2(-1 * a[0], sqrt(pow(a[1], 2) + pow(a[2], 2))));
    double roll_acc = (atan2(-1 * a[1], sqrt(pow(a[2], 2) + pow(a[0], 2))));
    IMU->pitch = TAU * (IMU->pitch + (g[1]) * (dt)) + (1 - TAU) * pitch_acc;
    IMU->roll = TAU * (IMU->roll + (g[2]) * (dt)) + (1 - TAU) * roll_acc;
  return 0;
}

/* proportional feedback update for stepper motor
 * param:
 * target_angel - angle of target position to reach
 * current_angel - angel of current position
 * returns: dir to move stepper */
enum StepperDirection update_proportional(double target_angel,
                                          double current_angel) {
  double error = fabs(target_angel - current_angel);
  if (error <= 0.1)
    return STOP_PULSE;
  // float diff = target_angel - current_angel;
  if (target_angel > current_angel)
    return HIGH_PULSE;
  else // target_angel<current_angel
    return LOW_PULSE;
}

/* compute yaw using magnetometer data
 * param:
 * dev - magnetometer device
 * MAG - pointer to struct joint
 * returns: 0 if successfull*/
int process_yaw(const struct imu_data *data, struct joint *MAG) {

  MAG->yaw = atan2(data->mag[0], data->mag[1]) - M_PI / 2;
  if (MAG->yaw < 0)
    MAG->yaw += 2 * M_PI;
  if (MAG->yaw > 2 * M_PI)
    MAG->yaw -= 2 * M_PI;
  return 0;

}
