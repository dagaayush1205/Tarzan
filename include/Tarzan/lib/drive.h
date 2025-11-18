#ifndef DRIVE_H
#define DRIVE_H

#include <stdint.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>

#include <Tarzan/lib/jerk_limiter.h>
#include <Tarzan/lib/lqr.h>
#include <Tarzan/lib/scurve_planner.h>

struct pwm_motor {
  const struct pwm_dt_spec dev_spec;
  const uint32_t min_pulse;
  const uint32_t max_pulse;
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
  uint8_t update_type;
};

struct DiffDriveCtrl {
  struct jerk_limiter_t linear_limiter;
  struct jerk_limiter_t angular_limiter;
  struct lqr yaw_error;
  bool autonomous;
};

struct DiffDriveCtx {
  struct DiffDriveConfig drive_config;
  struct DiffDriveCtrl drive_control;
  int64_t previous_update_timestamp;
  int (*velocity_callback)(const float *velocity_buffer, int buffer_len,
                           int wheels_per_side);
};

int pwm_motor_write(const struct pwm_motor *motor, uint32_t pulse_width);

uint32_t velocity_pwm_interpolation(float velocity, float *vel_range,
                                    uint32_t *pwm_range);

float sbus_velocity_interpolation(uint16_t channel, float *velocity_range,
                                  uint16_t *channel_range);

uint32_t sbus_pwm_interpolation(uint16_t channel, uint32_t *pwm_range,
                                uint16_t *channel_range);

struct DiffDriveCtx *
drive_init(struct DiffDriveConfig *config,
           int (*velocity_callback)(const float *velocity_buffer,
                                    int buffer_len, int wheels_per_side));

int diffdrive_update(struct DiffDriveCtx *ctx, struct DiffDriveTwist command,
                     float dt_sec, float yaw, float yaw_rate);
#endif
