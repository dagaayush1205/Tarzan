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

/* Velocity to PWM interpolation
 * params:
 * velocity-> input velocity
 * vel_range-> input velocity range
 * pwm_range-> output pwm range*/
uint32_t velocity_pwm_interpolation(float velocity, float *vel_range,
                                    uint32_t *pwm_range) {
  if (velocity > vel_range[1]) {
    return pwm_range[1];
  }

  if (velocity < vel_range[0]) {
    return pwm_range[0];
  }

  if (abs((int)velocity * 100) == 0) {
    return (uint32_t)((pwm_range[0] + pwm_range[1]) / 2);
  }

  float dvel = vel_range[1] - vel_range[0];
  float dpwm = pwm_range[1] - pwm_range[0];

  uint32_t pwm_interp =
      pwm_range[0] + (dpwm / dvel) * (velocity - vel_range[0]);

  return pwm_interp;
}

/* interpolates sbus channel value to velocity
 *param :
 *channel - sbus channel
 *velocity_range - velocity range for interpolation
 *channel_range - sbus channel range */
float sbus_velocity_interpolation(uint16_t channel, float *velocity_range,
                                  uint16_t *channel_range) {

  if (channel > channel_range[1])
    return velocity_range[1];

  if (channel < channel_range[0])
    return velocity_range[0];

  if (channel < 1050 && channel > 950)                  // deadzone
    return (velocity_range[0] + velocity_range[1]) / 2; // neutral

  float dchannel = channel_range[1] - channel_range[0];
  float dvel = velocity_range[1] - velocity_range[0];

  float vel_interp =
      velocity_range[0] + (dvel / dchannel) * (channel - channel_range[0]);

  return vel_interp;
}

/* interpolates sbus channel value to pwm
 * param :
 * channel - sbus channel value
 * pwm_range - pwm range for interpolation
 * channel_range - sbus channel range */
uint32_t sbus_pwm_interpolation(uint16_t channel, uint32_t *pwm_range,
                                uint16_t *channel_range) {

  if (channel > channel_range[1])
    return pwm_range[2];

  if (channel < channel_range[0])
    return pwm_range[0];

  if (channel < 1050 && channel > 950) // deadzone
    return 1500000;                    // neutral

  float dchannel = channel_range[1] - channel_range[0];
  float dpwm = pwm_range[1] - pwm_range[0];

  uint32_t pwm_interp =
      pwm_range[0] + (dpwm / dchannel) * (channel - channel_range[0]);

  return pwm_interp;
}

/*initialize the diffdirve context variable
 * param:
 * config-> ptr to the config to init
 * velocity_callback-> func to write the pwm val to motors
 **/
struct DiffDriveCtx *
drive_init(struct DiffDriveConfig *config,
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

  // Initialize the jerk limiter
  jerk_limiter_init(&ctx->drive_control.linear_limiter, 0.0f, 0.0f,
                    LINEAR_V_MAX, LINEAR_A_MAX, LINEAR_J_MAX);
  jerk_limiter_init(&ctx->drive_control.angular_limiter, 0.0f, 0.0f,
                    ANGULAR_V_MAX, ANGULAR_A_MAX, ANGULAR_J_MAX);

  // Initialize lqr gains
  init_lqr_gains(&ctx->drive_control.yaw_error);

  // set autonomous flag
  ctx->drive_control.autonomous = false;

  return ctx;
}

/*differential kinematics func
 * param:
 * ctx-> differential drive context var
 * command-> linear and angular command
 * dt_sec-> time since last update*/
int diffdrive_update(struct DiffDriveCtx *ctx, struct DiffDriveTwist command,
                     float dt_sec, float yaw, float yaw_rate) {
  int ret = 0;
  float linear_command;
  float angular_command;

  // Get time since last update
  if (k_uptime_delta(&ctx->previous_update_timestamp) / 1000 >
      ctx->drive_config.command_timeout_seconds) {
    command.linear_x = 0.0;
    command.angular_z = 0.0;
  }

  // MANUAL MODE
  if (ctx->drive_control.autonomous) {
    linear_command = command.linear_x;
    angular_command = command.angular_z;
  } else {
    linear_command = jerk_limiter_step(&ctx->drive_control.linear_limiter,
                                       command.linear_x, dt_sec);
    angular_command = jerk_limiter_step(&ctx->drive_control.angular_limiter,
                                        command.angular_z, dt_sec);
  }

  if (ctx->drive_control.yaw_error.yaw_correction) {
    if (angular_command != 0.0f) {
      ctx->drive_control.yaw_error.desired_yaw = yaw;
    } else if (linear_command != 0.0f) {
      ctx->drive_control.yaw_error.yaw = yaw;
      ctx->drive_control.yaw_error.yaw_rate = yaw_rate;
      angular_command = lqr_yaw_correction(&ctx->drive_control.yaw_error);
    }
  }

  if(angular_command!=0.0f){
	  ctx->drive_control.yaw_error.desired_yaw = yaw;
  }
  else if(linear_command!=0.0f){
	  ctx->drive_control.yaw_error.yaw = yaw;
	  ctx->drive_control.yaw_error.yaw_rate = yaw_rate;
	  angular_command = lqr_yaw_correction(&ctx->drive_control.yaw_error);
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
