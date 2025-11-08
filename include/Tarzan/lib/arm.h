#pragma once
#include <math.h>
#include <stdint.h>
#include <zephyr/device.h>

#define M_PI 3.14159265358979323846
#define TAU 0.90
#define DELTA_T 0.01 // 100Hz sampling rate
#define GYRO_MEAN_ERROR 0
#define BETA                                                                   \
  0.5 // the paper this code referred to to used sqrt(3.0f/4.0f) *
      // GYRO_MEAN_ERROR, it need to be adjusted by hit n trial

enum StepperDirection {
  LOW_PULSE = 0,
  HIGH_PULSE = 1,
  STOP_PULSE = 2,
};

extern struct quaternion q_est;
/* store imu related data */
struct joint {
  double pitch;
  double roll;
  double yaw;
  uint64_t prev_time;
  double gyro_offset[3];
  double mag_offset[3];
};
/* inverse message */
struct inverse_msg {
  double turn_table;
  double first_link;
  double second_link;
  double pitch;
  double roll;
  double x;
  double y;
  double z;
};
/* imu data */ 
struct imu_data { 
  double accel[3]; 
  double gyro[3];
  double mag[3];
  double gyro_offset[3];
};
/* imu data msg */ 
struct imu_msg {
  struct imu_data baseLink; 
  struct imu_data firstLink; 
  struct imu_data secondLink; 
  struct imu_data differential;
}; 
int Stepper_motor_write(const struct stepper_motor *motor, int dir, int pos);

int process_pitch_roll(const struct imu_data *data, struct joint *IMU);

int process_yaw(const struct imu_data *data, struct joint *MAG);

int calibration(const struct device *dev, struct joint *IMU);

int calibrationlsm(const struct device *dev, struct joint *IMU);

int calibrate_magnetometer(const struct device *dev, struct joint *IMU, int32_t duration_ms);

enum StepperDirection update_proportional(double target_angle,
                                          double current_angel);

