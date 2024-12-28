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
  double accel[3];
  double gyro[3];
  double mag[3];
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
struct quaternion {
  float q1;
  float q2;
  float q3;
  float q4;
};
int Stepper_motor_write(const struct stepper_motor *motor, int dir, int pos);

int process_pitch_roll(const struct device *dev, struct joint *IMU);

int calibration(const struct device *dev, struct joint *IMU);

enum StepperDirection update_proportional(double target_angle,
                                          double current_angel);

struct quaternion quat_mult(struct quaternion q_L, struct quaternion q_R);
struct quaternion quat_mult(struct quaternion q_L, struct quaternion q_R);

static inline void quat_scalar(struct quaternion *q, float scalar) {
  q->q1 *= scalar;
  q->q2 *= scalar;
  q->q3 *= scalar;
  q->q4 *= scalar;
}

static inline void quat_add(struct quaternion *Sum, struct quaternion L,
                            struct quaternion R) {
  Sum->q1 = L.q1 + R.q1;
  Sum->q2 = L.q2 + R.q2;
  Sum->q3 = L.q3 + R.q3;
  Sum->q4 = L.q4 + R.q4;
}

static inline void quat_sub(struct quaternion *Sum, struct quaternion L,
                            struct quaternion R) {
  Sum->q1 = L.q1 - R.q1;
  Sum->q2 = L.q2 - R.q2;
  Sum->q3 = L.q3 - R.q3;
  Sum->q4 = L.q4 - R.q4;
}

static inline struct quaternion quat_conjugate(struct quaternion q) {
  q.q2 = -q.q2;
  q.q3 = -q.q3;
  q.q4 = -q.q4;
  return q;
}

static inline float quat_Norm(struct quaternion q) {
  return sqrt(q.q1 * q.q1 + q.q2 * q.q2 + q.q3 * q.q3 + q.q4 * q.q4);
}

static inline void quat_Normalization(struct quaternion *q) {
  float norm = quat_Norm(*q);
  q->q1 /= norm;
  q->q2 /= norm;
  q->q3 /= norm;
  q->q4 /= norm;
}
void imu_filter(float ax, float ay, float az, float gx, float gy, float gz,
                float mx, float my, float mz);

void eulerAngles(struct quaternion q, float *roll, float *pitch, float *yaw);
