#pragma once
#include <stdint.h>
#include <zephyr/device.h>
#define HIGH_PULSE 1
#define LOW_PULSE 0
#define M_PI 3.14159265358979323846
#define TAU 0.90

/* store imu related data */
struct joint {
  double accel[3];
  double gyro[3];
  double pitch;
  double roll;
  uint64_t prev_time;
  double gyro_offset[3];
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
int Stepper_motor_write(const struct stepper_motor *motor, int dir, int pos);

void process_mpu6050(const struct device *dev, struct joint *IMU);

int calibration(const struct device *dev, struct joint *IMU);

int update_proportional(float target_angle, float current_angel);
