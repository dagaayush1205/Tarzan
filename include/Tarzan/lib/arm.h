#ifndef ARM_H
#define ARM_H

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#define M_PI 3.14159265358979323846f
#define TAU 0.90f

struct stepper {
  const struct gpio_dt_spec dir;
  const struct gpio_dt_spec step;
};

enum StepperDirection {
  LOW_PULSE = 0,
  HIGH_PULSE = 1,
  STOP_PULSE = 2,
};

extern struct quaternion q_est;

/* store imu related data */
struct joint {
  float accel[3];
  float gyro[3];
  float mag[3];
  float pitch;
  float roll;
  float yaw;
  float gyro_offset[3];
  float mag_offset[3];
  uint64_t prev_time;
};
/* inverse message */
struct inverse_msg {
  float turn_table;
  float first_link;
  float second_link;
  float pitch;
  float roll;
  float x;
  float y;
  float z;
};

/* imu data */ 
struct imu_data { 
  enum {
        IMU_OK = 0,
        IMU_NOT_READY,
        IMU_CALIBRATION_FAILED,
        IMU_FETCH_FAILED
  } error_code;
  double accel[3]; 
  double gyro[3];
  double mag[3];
  double gyro_offset[3];
  double accel_offset[3];
  uint64_t prev_time;
  double pitch;
  double roll;
};

/* imu data msg */
struct imu_msg {
  struct imu_data baseLink;
  struct imu_data firstLink;
  struct imu_data secondLink;
  struct imu_data differential;
};

int Stepper_motor_write(const struct stepper *motor, int dir, int pos);

int complementary_filter(const struct device *dev, struct joint *data);

int calibrate_gyro(const struct device *dev, struct joint *data);

int madgwick_filter(const struct device *const mag_dev,
                    const struct device *const imu_dev, struct joint *data);

int calibrate_magnetometer(const struct device *dev, struct joint *data);

enum StepperDirection update_proportional(float target_angle,
                                          float current_angel);
#endif
