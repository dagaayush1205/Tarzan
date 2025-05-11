int diffdrive_update(struct DiffDrive *drive, struct DiffDriveTwist command,
                     int64_t current_time)
{
    int ret = 0;

    int64_t dt_us = current_time - drive->previous_update_timestamp;
    float dt_sec = dt_us / 1e6f;

    if (dt_sec <= 0.0f) {
        return 1;  // Avoid division by zero or negative time
    }

    // Timeout check
    if (dt_sec > drive->config.command_timeout_seconds) {
        command.linear_x = 0.0f;
        command.angular_z = 0.0f;
    }

    drive->previous_update_timestamp = current_time;

    // Adjusted wheel parameters
    const float wheel_separation = drive->config.wheel_separation_multiplier * drive->config.wheel_separation;
    const float left_wheel_radius = drive->config.left_wheel_radius_multiplier * drive->config.wheel_radius;
    const float right_wheel_radius = drive->config.right_wheel_radius_multiplier * drive->config.wheel_radius;

    // Get feedback
    const int buffer_len = drive->config.wheels_per_side * 2;
    float *feedback = malloc(sizeof(float) * buffer_len);
    if (!feedback || drive->feedback_callback(feedback, buffer_len, drive->config.wheels_per_side)) {
        free(feedback);
        return 1;
    }

    float left_feedback_mean = 0.0f, right_feedback_mean = 0.0f;
    for (int i = 0; i < drive->config.wheels_per_side; i++) {
        float left_val = feedback[i];
        float right_val = feedback[drive->config.wheels_per_side + i];

        if (isnan(left_val) || isnan(right_val)) {
            free(feedback);
            return 1;
        }

        left_feedback_mean += left_val;
        right_feedback_mean += right_val;
    }
    left_feedback_mean /= drive->config.wheels_per_side;
    right_feedback_mean /= drive->config.wheels_per_side;
    free(feedback);

    // Update odometry
    if (drive->config.update_type == POSITION_FEEDBACK) {
        diffdrive_odometry_update(drive->odometry, left_feedback_mean, right_feedback_mean, current_time);
    } else {
        float left_vel = left_feedback_mean * left_wheel_radius;
        float right_vel = right_feedback_mean * right_wheel_radius;
        diffdrive_odometry_update_from_velocity(drive->odometry, left_vel, right_vel, current_time);
    }

    // Target wheel velocities
    float target_velocity_left = (command.linear_x - (command.angular_z * wheel_separation / 2.0f)) / left_wheel_radius;
    float target_velocity_right = (command.linear_x + (command.angular_z * wheel_separation / 2.0f)) / right_wheel_radius;

    // Smooth transition based on max delta
    float max_delta = drive->config.max_velocity_delta; // in rad/s
    float delta_limit = max_delta * dt_sec;

    // Smooth left wheel
    float delta_left = target_velocity_left - drive->prev_velocity_left;
    if (fabsf(delta_left) > delta_limit) {
        delta_left = delta_limit * (delta_left > 0 ? 1 : -1);
    }
    float smooth_velocity_left = drive->prev_velocity_left + delta_left;

    // Smooth right wheel
    float delta_right = target_velocity_right - drive->prev_velocity_right;
    if (fabsf(delta_right) > delta_limit) {
        delta_right = delta_limit * (delta_right > 0 ? 1 : -1);
    }
    float smooth_velocity_right = drive->prev_velocity_right + delta_right;

    // Store smoothed velocities
    drive->prev_velocity_left = smooth_velocity_left;
    drive->prev_velocity_right = smooth_velocity_right;

    // Send to motors
    float *velocity_buffer = malloc(sizeof(float) * buffer_len);
    if (!velocity_buffer) return 1;

    for (int i = 0; i < drive->config.wheels_per_side; i++) {
        velocity_buffer[i] = smooth_velocity_left;
        velocity_buffer[drive->config.wheels_per_side + i] = smooth_velocity_right;
    }

    if (drive->velocity_callback(velocity_buffer, buffer_len, drive->config.wheels_per_side)) {
        ret = 1;
    }

    free(velocity_buffer);
    return ret;
}

