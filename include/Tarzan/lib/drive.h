#include <stdint.h>
#include <kyvernitis/lib/kyvernitis.h>
#include <Tarzan/lib/jerk_limiter.h>
#include <Tarzan/lib/scurve_planner.h>

struct DiffDriveCtrl {
    jerk_limiter_t linear_limiter;
    jerk_limiter_t angular_limiter;
    scurve_profile_t linear_profile;
    scurve_profile_t angular_profile;
    float move_timer;
};

struct DiffDriveCtx {
   struct DiffDriveConfig drive_config; 
   struct DiffDriveCtrl drive_control;
   int64_t previous_update_timestamp;
   int (*velocity_callback)(const float *velocity_buffer, int buffer_len, int wheels_per_side);
};

enum msg_type { AUTONOMOUS, INVERSE, IMU };

float sbus_velocity_interpolation(uint16_t channel, float *velocity_range,
                                  uint16_t *channel_range);

uint32_t sbus_pwm_interpolation(uint16_t channel, uint32_t *pwm_range,
                                uint16_t *channel_range);

struct DiffDriveCtx* drive_init(struct DiffDriveConfig *config, int (*velocity_callback)(const float *velocity_buffer, int buffer_len, int wheels_per_side));

int diffdrive_kine(struct DiffDriveCtx* ctx, struct DiffDriveTwist command, enum msg_type mode);


