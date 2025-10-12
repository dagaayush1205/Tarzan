#include <stdint.h>

#include <kyvernitis/lib/kyvernitis.h>

#include <Tarzan/lib/drive.h>

#include <Tarzan/lib/scurve_planner.h>

#include <Tarzan/lib/jerk_limiter.h>

/* interpolates sbus channel value to velocity
 *param :
 *channel - sbus channel
 *velocity_range - velocity range for interpolation
 *channel_range - sbus channel range */

typedef enum {
    DRIVE_MODE_TELEOP,
    DRIVE_MODE_AUTO
} drive_mode_t;

#define LINEAR_V_MAX  1.5f  
#define LINEAR_A_MAX  1.0f   
#define LINEAR_J_MAX  2.5f  
#define ANGULAR_V_MAX 2.0f   
#define ANGULAR_A_MAX 1.5f  
#define ANGULAR_J_MAX 3.0f   

void diffdrive_init(struct DiffDrive *drive) {
    // Set default operating mode and states
    drive->mode = DRIVE_MODE_TELEOP;
    drive->is_auto_move_active = false;
    drive->move_timer = 0.0f;

    // Initialize the teleop jerk limiters with your rover's physical limits
    jerk_limiter_init(&drive->linear_limiter,0.0f,0.0f,LINEAR_V_MAX,LINEAR_A_MAX,LINEAR_J_MAX);

    jerk_limiter_init(&drive->angular_limiter,0.0f,0.0f,ANGULAR_V_MAX,ANGULAR_A_MAX,ANGULAR_J_MAX);
}

void set_drive_mode(struct DiffDrive *drive, drive_mode_t new_mode) {
    drive->mode = new_mode;
    drive->is_auto_move_active = false;
}

void start_auto_move(struct DiffDrive *drive, float linear_distance, float angular_distance) {
    //set to auto if not
    drive->mode = DRIVE_MODE_AUTO;
    //define the constraints for the S-curve planner using your constants
    scurve_constraints_t linear_limits = {LINEAR_V_MAX, LINEAR_A_MAX, LINEAR_J_MAX};
    scurve_constraints_t angular_limits = {ANGULAR_V_MAX, ANGULAR_A_MAX, ANGULAR_J_MAX};

    // Plan the motion profiles for a simple point-to-point move (start and end velocity are 0)
    scurve_plan_profile(&drive->linear_profile, &linear_limits, linear_distance, 0.0f, 0.0f);
    scurve_plan_profile(&drive->angular_profile, &angular_limits, angular_distance, 0.0f, 0.0f);
    // Reset the timer and activate the move
    drive->move_timer = 0.0f;
    drive->is_auto_move_active = true;
}



//EXISTING FUNCS
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

/* float sbus_velocity_interpolation(uint16_t channel, float *velocity_range, uint16_t *channel_range) {
  if (channel < 950 || channel > 2050) { // Gradual scaling beyond deadlock
    float factor = (channel < 950) ? (950.0 - channel) / 100.0 : (channel - 2050.0) / 100.0;
    return velocity_range[0] + (velocity_range[1] - velocity_range[0]) * factor;
  }

  if (channel >= 950 && channel <= 1050) // Deadzone check
    return (velocity_range[0] + velocity_range[1]) / 2; // neutral

  float dchannel = channel_range[1] - channel_range[0];
  float dvel = velocity_range[1] - velocity_range[0];
  
  float vel_interp = velocity_range[0] + (dvel / dchannel) * (channel - channel_range[0]);
  return vel_interp;
}


 /*
uint32_t sbus_pwm_interpolation(uint16_t channel, uint32_t *pwm_range, uint16_t *channel_range) {
  if (channel < 950 || channel > 2050) { // Gradual scaling beyond deadlock
    float factor = (channel < 950) ? (950.0 - channel) / 100.0 : (channel - 2050.0) / 100.0;
    return pwm_range[0] + (pwm_range[1] - pwm_range[0]) * factor;
  }

  if (channel >= 950 && channel <= 1050) // Deadzone check
    return 1500000; // neutral

  float dchannel = channel_range[1] - channel_range[0];
  float dpwm = pwm_range[1] - pwm_range[0];
  
  uint32_t pwm_interp = pwm_range[0] + (dpwm / dchannel) * (channel - channel_range[0]);
  return pwm_interp;
}
*/



int diffdrive_update(struct DiffDrive *drive, struct DiffDriveTwist command)
{
	int ret = 0;
	// Get time since last update
	if (k_uptime_delta(&drive->previous_update_timestamp) / 1000 >
	    drive->config.command_timeout_seconds) {
		command.linear_x = 0.0;
		command.angular_z = 0.0;
	}

  const float dt_sec = k_uptime_delta(&drive->previous_update_timestamp) / 1000.0f;


  //NEW CODE ADDED
    float linear_command;
    float angular_command;

    switch (drive->mode)
    {
        case DRIVE_MODE_TELEOP:
          //MANUAL MODE- we are taking the joystick command
            linear_command = jerk_limiter_step(&drive->linear_limiter, command.linear_x, dt_sec);
            angular_command = jerk_limiter_step(&drive->angular_limiter, command.angular_z, dt_sec);
            break;
        case DRIVE_MODE_AUTO:
            //AUTO MODE- we are using scurve
            if (drive->is_auto_move_active) {
                drive->move_timer += dt_sec;

                linear_command = scurve_evaluate_velocity(&drive->linear_profile, drive->move_timer);
                angular_command = scurve_evaluate_velocity(&drive->angular_profile, drive->move_timer);

                // Stop the move when the longer profile is complete
                float total_move_time = fmaxf(drive->linear_profile.T, drive->angular_profile.T);
                if (drive->move_timer >= total_move_time) {
                    drive->is_auto_move_active = false;
                }
            } else {
                // If no auto move is active, command zero velocity
                linear_command = 0.0f;
                angular_command = 0.0f;
            }
            break;
    }


	const float wheel_separation =
		drive->config.wheel_separation_multiplier * drive->config.wheel_separation;
	const float left_wheel_radius =
		drive->config.left_wheel_radius_multiplier * drive->config.wheel_radius;
	const float right_wheel_radius =
		drive->config.right_wheel_radius_multiplier * drive->config.wheel_radius;

	float linear_command = command.linear_x;
	float angular_command = command.angular_z;

	const float velocity_left =
		(linear_command - angular_command * wheel_separation / 2.0) / left_wheel_radius;
	const float velocity_right =
		(linear_command + angular_command * wheel_separation / 2.0) / right_wheel_radius;

	float *velocity_buffer = (float *)malloc(sizeof(float) * feedback_buffer_size);
	for (int i = 0; i < drive->config.wheels_per_side; i++) {
		velocity_buffer[i] = velocity_left;
		velocity_buffer[drive->config.wheels_per_side + i] = velocity_right;
	}
	if (drive->velocity_callback(velocity_buffer, feedback_buffer_size,
				     drive->config.wheels_per_side)) {
		// ERROR: Something went wrong writing the velocities
		ret = 3;
	}
	free(velocity_buffer);
	return ret;
}

