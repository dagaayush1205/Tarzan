#ifndef DIFF_KINE_H
#define DIFF_KINE_H

#include <stdint.h>

// Forward declarations of structs
struct DiffDrive;

struct DiffDriveTwist {
	float linear_x;
	float angular_z;
};

struct DiffDrive *diffdrive_init(struct DiffDriveConfig *config,
		     int (*feedback_callback)(float *feedback_buffer, int buffer_len,
					      int wheels_per_side),
		     int (*velocity_callback)(const float *velocity_buffer, int buffer_len,
					      int wheels_per_side));

// Main kinematics function
int diff_kine(struct DiffDrive *drive, struct DiffDriveTwist command,
              int64_t time_taken_by_last_update_seconds);

struct DiffDriveStatus diffdrive_status(struct DiffDrive *drive);

#endif // DIFF_KINE_H

