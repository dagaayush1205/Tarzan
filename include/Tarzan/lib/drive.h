#include <stdint.h>

enum msg_type { AUTONOMOUS, INVERSE };

float sbus_velocity_interpolation(uint16_t channel, float *velocity_range,
                                  uint16_t *channel_range);

uint32_t sbus_pwm_interpolation(uint16_t channel, uint32_t *pwm_range,
                                uint16_t *channel_range);
