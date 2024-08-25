#include <Tarzan/lib/drive.h>

float sbus_velocity_interpolation(uint16_t channel,float *velocity_range,int *channel_range) {

	if (channel > channel_range[1])
		return velocity_range[1];

	if (channel < channel_range[0])
		return velocity_range[0];

	if (channel < 1005 && channel > 995) 
		return (velocity_range[0] + velocity_range[1]) / 2;

	float dchannel = channel_range[1] - channel_range[0];
	float dvel = velocity_range[1] - velocity_range[0];

	float  vel_interp = velocity_range[0] + (dvel / dchannel) * (channel - channel_range[0]);
	
	return vel_interp;
}

uint32_t one_hot_interpolation(uint16_t channel, uint32_t *pwm_range) {

	if (channel < 600) 
		return pwm_range[0];
	if (channel > 1400) 
		return pwm_range[2];
	
	return pwm_range[1];
}

/*
int diffdrive_update(struct DiffDrive *drive, struct DiffDriveTwist command,
		     int64_t time_taken_by_last_update_seconds)
{
	int ret = 0;
	// Get time since last update
	if (k_uptime_delta(&drive->previous_update_timestamp) / 1000 >
	    drive->config.command_timeout_seconds) {
		command.linear_x = 0.0;
		command.angular_z = 0.0;
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

	const int feedback_buffer_size = drive ->config.wheels_per_side*2;

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
}*/
