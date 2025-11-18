#ifndef LQR_H
#define LQR_H

#include <math.h>
#include <stdint.h>

static const float Iz =  1500.0f;
static const float a = 0.1f;    // damping term
static const float q1 = 100.0f; // wt yaw error
static const float q2 = 1.0f;   // wt yaw rate
static const float r = 1.0f;    // wt control effort

struct lqr{
  float K[2]; // k1 k2
  float yaw; 
  float yaw_rate;
  float desired_yaw;
};

static inline float wrap_pi(float angle) {
  const float PI = 3.14159265359f;
  while (angle > PI)
    angle -= 2.0f * PI;
  while (angle < -PI)
    angle += 2.0f * PI;
  return angle;
}

static inline void init_lqr_gains(struct lqr* lqr_ctx) {

  float b = 1.0f / Iz;
  float p22 = sqrtf(q2 / r);
  float term = (a - b * p22);
  float disc = term * term + 4.0f * q1;

  if (disc < 0.0f)
    disc = 0.0f;
  float p12 = (p22 * b - a + sqrtf(disc)) / 2.0f;

  lqr_ctx->K[0] = p12 / r;
  lqr_ctx->K[1] = p22 / r;
}

static inline float lqr_yaw_correction(struct lqr* lqr_ctx) {

  float err = wrap_pi(lqr_ctx->yaw - lqr_ctx->desired_yaw);

  float u = -((lqr_ctx->K[0] * err) + (lqr_ctx->K[1] * lqr_ctx->yaw_rate));

  return u;
}
#endif
