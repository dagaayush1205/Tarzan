#include "zephyr/kernel.h"
#include <stdint.h>
#include <stdlib.h>

#include <Tarzan/lib/drive.h>

#define LINEAR_V_MAX 1.5f
#define LINEAR_A_MAX 0.8f
#define LINEAR_J_MAX 1.5f
#define ANGULAR_V_MAX 2.0f
#define ANGULAR_A_MAX 5.0f
#define ANGULAR_J_MAX 8.0f

/* Wrapper around pwm_set_pulse_dt to ensure that pulse_width
remains under max-min ranges
params:
motor-> pwm pin to write
pulse_width-> the pwm value to write*/
int pwm_motor_write(const struct pwm_motor *motor, uint32_t pulse_width) {
  // wrapper around pwm_set_pulse_dt to ensure that pulse_width
  // remains under max-min range
  if (pulse_width <= motor->min_pulse)
    pulse_width = motor->min_pulse;
  if (pulse_width >= motor->max_pulse)
    pulse_width = motor->max_pulse;

  int ret = pwm_set_pulse_dt(&(motor->dev_spec), pulse_width);
  return ret;
}

/*initialize the diffdirve context variable
 * param:
 * config-> ptr to the config to init
 * velocity_callback-> func to write the pwm val to motors
 **/
struct DiffDriveCtx *
drive_init(struct DiffDriveConfig *config, bool jerk_limiter_switch,
           int (*velocity_callback)(const float *velocity_buffer,
                                    int buffer_len, int wheels_per_side)) {
  struct DiffDriveCtx *ctx =
      (struct DiffDriveCtx *)malloc(sizeof(struct DiffDriveCtx));

  if (!ctx) {
    return NULL; // Return NULL if memory allocation fails
  }

  memcpy(&ctx->drive_config, config, sizeof(ctx->drive_config));

  ctx->velocity_callback = velocity_callback;

  ctx->previous_update_timestamp = k_uptime_get();

  ctx->limiter_switch = jerk_limiter_switch;

  // Initialize the jerk limiter
  if (jerk_limiter_switch) {
    jerk_limiter_init(&ctx->drive_control.linear_limiter, 0.0f, 0.0f,
                      LINEAR_V_MAX, LINEAR_A_MAX, LINEAR_J_MAX);
    jerk_limiter_init(&ctx->drive_control.angular_limiter, 0.0f, 0.0f,
                      ANGULAR_V_MAX, ANGULAR_A_MAX, ANGULAR_J_MAX);
  }

  return ctx;
}

/*differential kinematics func
 * param:
 * ctx-> differential drive context var
 * command-> linear and angular command
 * dt_sec-> time since last update*/
int diffdrive_update(struct DiffDriveCtx *ctx, struct DiffDriveTwist command) {

  int ret = 0;
  float linear_command;
  float angular_command;

  int64_t dt = k_uptime_delta(&ctx->previous_update_timestamp);

  // Get time since last update
  if (dt / 1000 > ctx->drive_config.command_timeout_seconds) {
    command.linear_x = 0.0;
    command.angular_z = 0.0;
  }

  if (ctx->limiter_switch) {
    linear_command = jerk_limiter_step(&ctx->drive_control.linear_limiter,
                                       command.linear_x, dt);
    angular_command = jerk_limiter_step(&ctx->drive_control.angular_limiter,
                                        command.angular_z, dt);
  } else {
    linear_command = command.linear_x;
    angular_command = command.angular_z;
  }

  const float wheel_separation = ctx->drive_config.wheel_separation_multiplier *
                                 ctx->drive_config.wheel_separation;

  const float left_wheel_radius =
      ctx->drive_config.left_wheel_radius_multiplier *
      ctx->drive_config.wheel_radius;

  const float right_wheel_radius =
      ctx->drive_config.right_wheel_radius_multiplier *
      ctx->drive_config.wheel_radius;

  const int feedback_buffer_size = ctx->drive_config.wheels_per_side * 2;

  const float velocity_left =
      (linear_command - angular_command * wheel_separation / 2.0f) /
      left_wheel_radius;

  const float velocity_right =
      (linear_command + angular_command * wheel_separation / 2.0f) /
      right_wheel_radius;

  float *velocity_buffer =
      (float *)malloc(sizeof(float) * feedback_buffer_size);

  for (int i = 0; i < ctx->drive_config.wheels_per_side; i++) {
    velocity_buffer[i] = velocity_left;
    velocity_buffer[ctx->drive_config.wheels_per_side + i] = velocity_right;
  }

  if (ctx->velocity_callback(velocity_buffer, feedback_buffer_size,
                             ctx->drive_config.wheels_per_side))
    ret = 1;

  free(velocity_buffer);

  return ret;
}
