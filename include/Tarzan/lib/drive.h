#include <stdint.h>

enum msg_type { AUTONOMOUS, INVERSE , MANUAL};

struct DiffDriveMotion {
    struct DiffDriveConfig config;
    int64_t previous_update_timestamp;
    bool (*velocity_callback)(float*, int, int);
    float wheel_separation;
    float wheel_separation_multiplier;
    float wheel_radius;
    int wheels_per_side;
    int command_timeout_seconds;
    float left_wheel_radius_multiplier;
    float right_wheel_radius_multiplier;
    //for motion control
    msg_type mode;
    jerk_limiter_t linear_limiter;
    jerk_limiter_t angular_limiter;
    scurve_profile_t linear_profile;
    scurve_profile_t angular_profile;
    float move_timer;
    bool is_auto_move_active;
};


float sbus_velocity_interpolation(uint16_t channel, float *velocity_range,
                                  uint16_t *channel_range);

uint32_t sbus_pwm_interpolation(uint16_t channel, uint32_t *pwm_range,
                                uint16_t *channel_range);


