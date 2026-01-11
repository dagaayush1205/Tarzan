#ifndef DRIVE_H
#define DRIVE_H

#include <stdint.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>

#include <Tarzan/lib/jerk_limiter.h>
#include <Tarzan/lib/lqr.h>

/* macro for linear interpolation */
#define LINEAR_INTERPOLATION(x, in_min, in_max, out_min, out_max)              \
  (((x) - (in_min)) * ((out_max) - (out_min)) / ((in_max) - (in_min)) +        \
   (out_min))

struct pwm_motor {
  const struct pwm_dt_spec dev_spec;
  const uint32_t min_pulse;
  const uint32_t max_pulse;
  int channel;
};

struct DiffDriveTwist {
  float linear_x;
  float angular_z;
};

struct DiffDriveConfig {
  float wheel_separation;
  int wheels_per_side; // actually 2, but both are controlled by 1 signal
  float wheel_radius;
  int64_t command_timeout_seconds;
  float wheel_separation_multiplier;
  float left_wheel_radius_multiplier;
  float right_wheel_radius_multiplier;
};

struct DiffDriveCtrl {
  struct jerk_limiter_t linear_limiter;
  struct jerk_limiter_t angular_limiter;
};

struct DiffDriveCtx {
  struct DiffDriveConfig drive_config;
  struct DiffDriveCtrl drive_control;
  bool limiter_switch;
  int64_t previous_update_timestamp;
  int (*velocity_callback)(const float *velocity_buffer, int buffer_len,
                           int wheels_per_side);
};

int pwm_motor_write(const struct pwm_motor *motor, uint32_t pulse_width);

struct DiffDriveCtx *
drive_init(struct DiffDriveConfig *config, bool jerk_limiter_switch,
           int (*velocity_callback)(const float *velocity_buffer,
                                    int buffer_len, int wheels_per_side));

int diffdrive_update(struct DiffDriveCtx *ctx, struct DiffDriveTwist command);
#endif
