#ifndef SMC_TEST_H_
#define SMC_TEST_H_

/* float sbus_velocity_interpolation(uint16_t channel, float *velocity_range, */
/*                                   uint16_t *channel_range); */
/* uint32_t sbus_pwm_interpolation(uint16_t channel, uint32_t *pwm_range, */
/*                                 uint16_t *channel_range);  */
float lqr_yaw_correction(float yaw_actual);

#endif // SMC_TEST_H_
