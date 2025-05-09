#ifndef DIFFDRIVE_UPDATE_H
#define DIFFDRIVE_UPDATE_H

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

struct DiffDriveConfig {
	float wheel_separation;
	float wheel_radius;
	float wheel_separation_multiplier;
	float left_wheel_radius_multiplier;
	float right_wheel_radius_multiplier;
	int wheels_per_side;
	int command_timeout_seconds;
	int update_type;
};

struct DiffDriveTwist {
	float linear_x;
	float angular_z;
};

struct DiffDriveOdometry;

struct DiffDrive {
	struct DiffDriveConfig config;
	int64_t previous_update_timestamp;
	struct DiffDriveOdometry *odometry;
	int (*feedback_callback)(float *feedback, int buffer_size, int wheels_per_side);
	int (*velocity_callback)(float *velocity, int buffer_size, int wheels_per_side);
};

#define POSITION_FEEDBACK 0

int k_uptime_delta(int64_t *timestamp);
void diffdrive_odometry_update(struct DiffDriveOdometry *odom, float left, float right, int64_t timestamp);
void diffdrive_odometry_update_from_velocity(struct DiffDriveOdometry *odom, float left, float right, int64_t timestamp);

static inline int diffdrive_update(struct DiffDrive *drive, struct DiffDriveTwist command,
                                   int64_t time_taken_by_last_update_seconds)
{
	int ret = 0;
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

	const int feedback_buffer_size = drive->config.wheels_per_side * 2;
	float *feedback = malloc(sizeof(float) * feedback_buffer_size);
	if (drive->feedback_callback(feedback, feedback_buffer_size,
	                             drive->config.wheels_per_side)) {
		ret = 1;
	}

	const float alpha = 0.2f;  // smoothing factor
	for (int i = 0; i < drive->config.wheels_per_side; i++) {
		const float left_feedback = feedback[i];
		const float right_feedback = feedback[drive->config.wheels_per_side + i];

		if (isnan(left_feedback) || isnan(right_feedback)) {
			ret = 1;
		}

		left_feedback_mean = (i == 0) ? left_feedback : alpha * left_feedback + (1.0f - alpha) * left_feedback_mean;
		right_feedback_mean = (i == 0) ? right_feedback : alpha * right_feedback + (1.0f - alpha) * right_feedback_mean;
	}
	free(feedback);

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
		ret = 1;
	}
	free(velocity_buffer);
	return ret;
}

#endif // DIFFDRIVE_UPDATE_H

