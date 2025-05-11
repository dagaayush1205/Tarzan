int diff_kine(struct DiffDrive *drive, struct DiffDriveTwist command,
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
	float left_feedback_mean = 0.0;
	float right_feedback_mean = 0.0;




  //----------------------------
	// Smooth the input commands
	const float alpha = 0.1;

// here it takes 90% of previous command and 10% of new command alpha-kept as smoothing factor  

	linear_command = alpha * command.linear_x + (1.0f - alpha) * drive->last_linear_command;
	angular_command = alpha * command.angular_z + (1.0f - alpha) * drive->last_angular_command;

	drive->last_linear_command = linear_command;
	drive->last_angular_command = angular_command;


  //---------------------------





	const int feedback_buffer_size = drive->config.wheels_per_side * 2;
	float *feedback = malloc(sizeof(float) * feedback_buffer_size);
	if (drive->feedback_callback(feedback, feedback_buffer_size,
				     drive->config.wheels_per_side)) {
		// ERROR: Something went wrong while getting feedback
		ret = 1;
	}
	for (int i = 0; i < drive->config.wheels_per_side; i++) {
		const float left_feedback = feedback[i];
		const float right_feedback = feedback[drive->config.wheels_per_side + i];

		if (isnan(left_feedback) || isnan(right_feedback)) {
			// ERROR: One of the wheels gives invalid feedback
			ret = 1;
		}

		left_feedback_mean += left_feedback;
		right_feedback_mean += right_feedback;
	}
	free(feedback);
	left_feedback_mean /= drive->config.wheels_per_side;
	right_feedback_mean /= drive->config.wheels_per_side;

	if (drive->config.update_type == POSITION_FEEDBACK) {
		diffdrive_odometry_update(drive->odometry, left_feedback_mean, right_feedback_mean,
					  drive->previous_update_timestamp);
	} else {
		diffdrive_odometry_update_from_velocity(drive->odometry,
							left_feedback_mean * left_wheel_radius *
								time_taken_by_last_update_seconds,
							right_feedback_mean * right_wheel_radius *
								time_taken_by_last_update_seconds,
							drive->previous_update_timestamp);
	}




  //--------------------------------
	const float velocity_left =
		(linear_command - angular_command * wheel_separation / 2.0f) / left_wheel_radius;
	const float velocity_right =
		(linear_command + angular_command * wheel_separation / 2.0f) / right_wheel_radius;
   
// stoping the angular to remove jerk

const float max_angular_z = 1.0f; // rad/s
if (angular_command > max_angular_z)
    angular_command = max_angular_z;
else if (angular_command < -max_angular_z)
    angular_command = -max_angular_z;
  
  //-------------------------------







	float *velocity_buffer = (float *)malloc(sizeof(float) * feedback_buffer_size);
	for (int i = 0; i < drive->config.wheels_per_side; i++) {
		velocity_buffer[i] = velocity_left;
		velocity_buffer[drive->config.wheels_per_side + i] = velocity_right;
	}
	if (drive->velocity_callback(velocity_buffer, feedback_buffer_size,
				     drive->config.wheels_per_side)) {
		// ERROR: Something went wrong writing the velocities
		ret = 1;
	}
	free(velocity_buffer);
	return ret;
}

