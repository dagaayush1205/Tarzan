#include <stdint.h>

#include <kyvernitis/lib/kyvernitis.h>

#include <Tarzan/lib/drive.h>

// interpolates sbus channel value to velocity
// param :
// channel - sbus channel
// velocity_range - velocity range for interpolation
// channel_range - sbus channel range
float sbus_velocity_interpolation(uint16_t channel, float *velocity_range,
                                  uint16_t *channel_range) {

  if (channel > channel_range[1])
    return velocity_range[1];

  if (channel < channel_range[0])
    return velocity_range[0];

  if (channel < 1050 && channel > 950) // deadzone
    return (velocity_range[0] + velocity_range[1]) / 2;

  float dchannel = channel_range[1] - channel_range[0];
  float dvel = velocity_range[1] - velocity_range[0];

  float vel_interp =
      velocity_range[0] + (dvel / dchannel) * (channel - channel_range[0]);

  return vel_interp;
}

// interpolates sbus channel value to pwm
// param :
// channel - sbus channel value
// pwm_range - pwm range for interpolation
// channel_range - sbus channel range
uint32_t sbus_pwm_interpolation(uint16_t channel, uint32_t *pwm_range,
                                uint16_t *channel_range) {

  if (channel > channel_range[1])
    return pwm_range[2];

  if (channel < channel_range[0])
    return pwm_range[0];

  if (channel < 1005 && channel > 995)
    return (pwm_range[0] + pwm_range[2]) / 2;

  float dchannel = channel_range[1] - channel_range[0];
  float dpwm = pwm_range[1] - pwm_range[2];

  uint32_t pwm_interp =
      pwm_range[0] + (dpwm / dchannel) * (channel - channel_range[0]);

  return pwm_interp;
}
