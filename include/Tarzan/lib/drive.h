#include <stdint.h>

/*struct DiffDriveTwist {
	float linear_x;
	float angular_z;
};*/

/*struct DiffDriveConfig { 
	float wheel_separation;
	float wheel_separation_multiplier;
	float wheel_radius;
	float wheels_per_side;
	int64_t command_timeout_seconds;
	float left_wheel_radius_multiplier;
	float right_wheel_radius_multiplier;
	uint8_t update_type;
};*/

float sbus_velocity_interpolation(uint16_t channel,float *velocity_range, int *channel_range);

uint32_t linear_actuator_pwm_interpolation(uint16_t linear_actuator_movement , uint32_t *pwm_range);  

//int diffdrive_update(struct DiffDrive *drive, struct DiffDriveTwist command, int64_t time_taken_by_last_update_seconds);
