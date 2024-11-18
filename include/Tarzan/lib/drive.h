#include <stdint.h>


float sbus_velocity_interpolation(uint16_t channel, float *velocity_range,
                                  int *channel_range);

uint32_t one_hot_interpolation(uint16_t channel, uint32_t *pwm_range,
                               uint16_t *channel_range);

int Stepper_motor_write(const struct stepper_motor *motor, uint16_t ch,
                        int pos);
