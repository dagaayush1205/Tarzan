#ifndef MADGWICK_FILTER_H
#define MADGWICK_FILTER_H

#include <math.h>

#define PI 3.14159265358979f
#define BETA 0.3f // Start with this value. Try 0.5 if it still drifts.

struct quaternion {
  float q1;
  float q2;
  float q3;
  float q4;
};

// global variables
extern struct quaternion q_est;

// Multiply two quaternions and return a copy of the result, prod = L * R
struct quaternion quat_mult(struct quaternion q_L, struct quaternion q_R);

// Multiply a reference of a quaternion by a scalar, q = s*q
static inline void quat_scalar(struct quaternion *q, float scalar) {
  q->q1 *= scalar;
  q->q2 *= scalar;
  q->q3 *= scalar;
  q->q4 *= scalar;
}

// Adds two quaternions together and the sum is the pointer to another
// quaternion, Sum = L + R
static inline void quat_add(struct quaternion *Sum, struct quaternion L,
                            struct quaternion R) {
  Sum->q1 = L.q1 + R.q1;
  Sum->q2 = L.q2 + R.q2;
  Sum->q3 = L.q3 + R.q3;
  Sum->q4 = L.q4 + R.q4;
}

// Subtracts two quaternions together and the sum is the pointer to another
// quaternion, sum = L - R
static inline void quat_sub(struct quaternion *Sum, struct quaternion L,
                            struct quaternion R) {
  Sum->q1 = L.q1 - R.q1;
  Sum->q2 = L.q2 - R.q2;
  Sum->q3 = L.q3 - R.q3;
  Sum->q4 = L.q4 - R.q4;
}

// the conjugate of a quaternion is it's imaginary component sign changed q* =
// [s, -v] if q = [s, v]
static inline struct quaternion quat_conjugate(struct quaternion q) {
  q.q2 = -q.q2;
  q.q3 = -q.q3;
  q.q4 = -q.q4;
  return q;
}

static inline float quat_Norm(struct quaternion q) {
  return sqrtf(q.q1 * q.q1 + q.q2 * q.q2 + q.q3 * q.q3 + q.q4 * q.q4);
}

// Normalizes pointer q by calling quat_Norm(q),
static inline void quat_Normalization(struct quaternion *q) {
  float norm = quat_Norm(*q);
  if (norm == 0.0f)
    return; // handle divide by zero
  q->q1 /= norm;
  q->q2 /= norm;
  q->q3 /= norm;
  q->q4 /= norm;
}

// This is the correct 9-DoF function prototype
void imu_filter(float ax, float ay, float az, float gx, float gy, float gz,
                float mx, float my, float mz, float dt);

void eulerAngles(struct quaternion q, float *roll, float *pitch, float *yaw);

#endif /* MADGWICK_FILTER_H */
