#include <stdint.h>

float sbus_velocity_interpolation(uint16_t channel, float *velocity_range,
                                  uint16_t *channel_range);

uint32_t sbus_pwm_interpolation(uint16_t channel, uint32_t *pwm_range,
                                uint16_t *channel_range);

int Stepper_motor_write(const struct stepper_motor *motor, uint16_t ch,
                        int pos);
