struct DiffDriveTwist TIMEOUT_CMD = {
	.angular_z = 0,
	.linear_x = 0,
};


struct DiffDriveConfig drive_config = { 
	.wheel_separation,
	.wheel_separation_multiplier,
	.wheel_radius,
	.wheels_per_side,
	.command_timeout_seconds,
	.left_wheel_radius_multiplier,
	.right_wheel_radius_multiplier,
	.update_type,
};

float sbus_velocity_interpolation(uint16_t channel_input,float *velocity_range);

int feedback_callback(float *feedback_buffer, int buffer_len, int wheels_per_side);

int velocity_callback(const float *velocity_buffer, int buffer_len, int wheels_per_side);

uint32_t linear_actuator_pwm_interpolation(uint16_t linear_actuator_movement , uint32_t *pwm_range);

int linear_actuator_write(int i, int dir);
