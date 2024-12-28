#include <kyvernitis/lib/kyvernitis.h>

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

/* complementary filter to compute pitch & roll
 * param:
 * dev: imu device
 * IMU: struct joint
 * returns: 0 if succesfull*/
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
/* magwick filter to compute yaw
 * param:
 * mm_dev - magnetometer device
 * imu_dev - imu device
 * data - pointer to struct joint
 * returns: 0 if successfull*/
int process_yaw(const struct device *mm_dev, const struct device *imu_dev,
                struct joint *data) {

  struct sensor_value accel[3], gyro[3], mag[3];

  int mm_rc = sensor_sample_fetch(mm_dev);
  int imu_rc = sensor_sample_fetch(imu_dev);

  if (imu_rc == 0)
    imu_rc = sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
  if (imu_rc == 0)
    imu_rc = sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_XYZ, gyro);
  if (mm_rc == 0)
    mm_rc = sensor_channel_get(mm_dev, SENSOR_CHAN_MAGN_XYZ, mag);

  else {
    return 1;
  }

  return 0;
}

struct quaternion q_est = {1, 0, 0, 0};
struct quaternion quat_mult(struct quaternion L, struct quaternion R) {
  struct quaternion product;
  product.q1 = (L.q1 * R.q1) - (L.q2 * R.q2) - (L.q3 * R.q3) - (L.q4 * R.q4);
  product.q2 = (L.q1 * R.q2) + (L.q2 * R.q1) + (L.q3 * R.q4) - (L.q4 * R.q3);
  product.q3 = (L.q1 * R.q3) - (L.q2 * R.q4) + (L.q3 * R.q1) + (L.q4 * R.q2);
  product.q4 = (L.q1 * R.q4) + (L.q2 * R.q3) - (L.q3 * R.q2) + (L.q4 * R.q1);

  return product;
}
void imu_filter(float ax, float ay, float az, float gx, float gy, float gz,
                float mx, float my, float mz) {
  struct quaternion q_est_prev = q_est;
  struct quaternion q_est_dot = {0};
  struct quaternion q_a = {0, ax, ay, az};
  struct quaternion q_m = {0, mx, my, mz};
  float F_g[3] = {0};
  float J_g[3][4] = {0};
  float F_m[3] = {0};
  float J_m[3][4] = {0};
  struct quaternion gradient = {0};
  struct quaternion q_w;
  q_w.q1 = 0;
  q_w.q2 = gx;
  q_w.q3 = gy;
  q_w.q4 = gz;

  quat_scaler(&q_w, 0.5);
  q_w = quat_mult(q_est_prev, q_w);
  // normalise data
  quat_Normalization(&q_a);
  quat_Normalization(&q_m);

  // function for gravity
  F_g[0] = 2 * (q_est_prev.q2 * q_est_prev.q4 - q_est_prev.q1 * q_est_prev.q3) -
           q_a.q2;
  F_g[1] = 2 * (q_est_prev.q1 * q_est_prev.q2 + q_est_prev.q3 * q_est_prev.q4) -
           q_a.q3;
  F_g[2] = 2 * (0.5 - q_est_prev.q2 * q_est_prev.q2 -
                q_est_prev.q3 * q_est_prev.q3) -
           q_a.q4;

  // function for magnetometer
  F_m[0] = 2 * (q_est_prev.q2 * q_est_prev.q4 - q_est_prev.q1 * q_est_prev.q3) -
           q_m.q2;
  F_m[1] = 2 * (q_est_prev.q1 * q_est_prev.q2 + q_est_prev.q3 * q_est_prev.q4) -
           q_m.q3;
  F_m[2] = 2 * (0.5 - q_est_prev.q2 * q_est_prev.q2 -
                q_est_prev.q3 * q_est_prev.q3) -
           q_m.q4;

  // jacobian for gravity
  J_g[0][0] = -2 * q_est_prev.q3;
  J_g[0][1] = 2 * q_est_prev.q4;
  J_g[0][2] = -2 * q_est_prev.q1;
  J_g[0][3] = 2 * q_est_prev.q2;

  J_g[1][0] = 2 * q_est_prev.q2;
  J_g[1][1] = 2 * q_est_prev.q1;
  J_g[1][2] = 2 * q_est_prev.q4;
  J_g[1][3] = 2 * q_est_prev.q3;

  J_g[2][0] = 0;
  J_g[2][1] = -4 * q_est_prev.q2;
  J_g[2][2] = -4 * q_est_prev.q3;
  J_g[2][3] = 0;

  // jacobian for magnetometer
  J_m[0][0] = -2 * q_est_prev.q3;
  J_m[0][1] = 2 * q_est_prev.q4;
  J_m[0][2] = -2 * q_est_prev.q1;
  J_m[0][3] = 2 * q_est_prev.q2;

  J_m[1][0] = 2 * q_est_prev.q2;
  J_m[1][1] = 2 * q_est_prev.q1;
  J_m[1][2] = 2 * q_est_prev.q4;
  J_m[1][3] = 2 * q_est_prev.q3;

  J_m[2][0] = 0;
  J_m[2][1] = -4 * q_est_prev.q2;
  J_m[2][2] = -4 * q_est_prev.q3;
  J_m[2][3] = 0;

  // gradient
  gradient.q1 = J_g[0][0] * F_g[0] + J_g[1][0] * F_g[1] + J_g[2][0] * F_g[2] +
                J_m[0][0] * F_m[0] + J_m[1][0] * F_m[1] + J_m[2][0] * F_m[2];
  gradient.q2 = J_g[0][1] * F_g[0] + J_g[1][1] * F_g[1] + J_g[2][1] * F_g[2] +
                J_m[0][1] * F_m[0] + J_m[1][1] * F_m[1] + J_m[2][1] * F_m[2];
  gradient.q3 = J_g[0][2] * F_g[0] + J_g[1][2] * F_g[1] + J_g[2][2] * F_g[2] +
                J_m[0][2] * F_m[0] + J_m[1][2] * F_m[1] + J_m[2][2] * F_m[2];
  gradient.q4 = J_g[0][3] * F_g[0] + J_g[1][3] * F_g[1] + J_g[2][3] * F_g[2] +
                J_m[0][3] * F_m[0] + J_m[1][3] * F_m[1] + J_m[2][3] * F_m[2];

  quat_Normalization(&gradient);
  // intergrating the gyro data
  quat_scalar(&q_w, 0.5);
  q_w = quat_mult(q_est_prev, q_w);

  // update
  quat_scalar(&gradient, BETA);
  quat_sub(&q_est_dot, q_w, gradient);
  quat_scalar(&q_est_dot, DELTA_T);
  quat_add(&q_est, q_est_prev, q_est_dot);
  quat_Normalization(&q_est);
}
// we should move this to process imu
void eulerAngles(struct quaternion q, float *roll, float *pitch, float *yaw) {

  *yaw = atan2f((2 * q.q2 * q.q3 - 2 * q.q1 * q.q4),
                (2 * q.q1 * q.q1 + 2 * q.q2 * q.q2 - 1)); // equation (7)
  *pitch = -asinf(2 * q.q2 * q.q4 + 2 * q.q1 * q.q3);     // equatino (8)
  *roll = atan2f((2 * q.q3 * q.q4 - 2 * q.q1 * q.q2),
                 (2 * q.q1 * q.q1 + 2 * q.q4 * q.q4 - 1));

  *yaw *= (180.0 / M_PI);
  *pitch *= (180.0 / M_PI);
  *roll *= (180.0 / M_PI);
}
