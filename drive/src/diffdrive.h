int diffdrive_update(struct DiffDrive *drive, struct DiffDriveTwist command,
		     int64_t time_taken_by_last_update_seconds)
{
	int ret = 0;

	// Timeout check
	if (k_uptime_delta(&drive->previous_update_timestamp) / 1000 >
	    drive->config.command_timeout_seconds) {
		command.linear_x = 0.0f;
		command.angular_z = 0.0f;
	}

	const float wheel_separation =
		drive->config.wheel_separation_multiplier * drive->config.wheel_separation;
	const float left_wheel_radius =
		drive->config.left_wheel_radius_multiplier * drive->config.wheel_radius;
	const float right_wheel_radius =
		drive->config.right_wheel_radius_multiplier * drive->config.wheel_radius;

	float linear_command = command.linear_x;
	float angular_command = command.angular_z;

	// === Smooth Turning Logic for Static Wheels ===
	const float MAX_TURN_SCALE = 0.5f;  // Scale down linear velocity during sharp turns
	const float ANGULAR_TURN_THRESHOLD = 1.0f; // radians per sec

	if (fabs(angular_command) > ANGULAR_TURN_THRESHOLD) {
		linear_command *= MAX_TURN_SCALE;  // Reduce forward/backward speed
	}

	// Compute individual wheel velocities
	const float velocity_left =
		(linear_command - angular_command * wheel_separation / 2.0f) / left_wheel_radius;
	const float velocity_right =
		(linear_command + angular_command * wheel_separation / 2.0f) / right_wheel_radius;

	const int wheel_count = drive->config.wheels_per_side * 2;
	float *velocity_buffer = (float *)malloc(sizeof(float) * wheel_count);
	if (!velocity_buffer) {
		return 1;
	}

	for (int i = 0; i < drive->config.wheels_per_side; i++) {
		velocity_buffer[i] = velocity_left;
		velocity_buffer[drive->config.wheels_per_side + i] = velocity_right;
	}

	if (drive->velocity_callback(velocity_buffer, wheel_count, drive->config.wheels_per_side)) {
		ret = 1;
	}

	free(velocity_buffer);
	return ret;
}

