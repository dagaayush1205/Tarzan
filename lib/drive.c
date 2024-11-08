#include <Tarzan/lib/drive.h>

// interpolates sbus channel value to velocity
// param :
// channel - sbus channel
// velocity_range - velocity range for interpolation
// channel_range - sbus channel range
float sbus_velocity_interpolation(uint16_t channel, float *velocity_range,
                                  int *channel_range) {

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

//// wrapper around gpio_set_dt
//// moves stepper motor forward or backward by 1 step
//// param :
//// motor - DT spec for stepper motor
//// ch - sbus channel value
//// returns :
//// ret = 0 if succesfull
//// ret = 1 if unsuccesfull
////int Stepper_motor_write(const struct stepper_motor *motor, uint16_t channel,
/// uint16_t *channel_range) {
//
//	int ret = 0;
//	if(channel > channel_range[1]) {
//		gpio_pin_set_dt(&(motor->dir), 1);
//		pos +=1; //clockwise
//	}
//	else {
//		gpio_pin_set_dt(&(motor->dir), 0);
//		pos -=1;
//	}	//anticlockwise
//	switch(pos & 0x03) {
//		case 0: ret+=gpio_pin_set_dt(&(motor->step),0);
////(0b10&(1<<0))?1:0); 			break; 		case 1:
/// ret+=gpio_pin_set_dt(&(motor->step),1);
////(0b11&(1<<0))?1:0); 			break; 		case 2:
/// ret+=gpio_pin_set_dt(&(motor->step),1);
////(0b01&(1<<0))?1:0); 			break; 		case 3:
/// ret+=gpio_pin_set_dt(&(motor->step),0);
////(0b00&(1<<0))?1:0); 			break;
//	}
//	return ret;
//}
//
//// returns the number of ticks(delay) required to to achieve input speed (in
/// rpm) / 1 tick = 1 microsecond / pararm: / speed - speed in rpm
// void setSpeed(float speed){
//	if(speed = 0.0)
//		stepInterval = 0.0;
//	else
//		stepInterval = 1 / (3 * PULSE_PER_REV * speed)
//			       *MINUTES_TO_MICRO;
// }
