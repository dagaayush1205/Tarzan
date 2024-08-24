#include <kyvernitis/lib/kyvernitis.h>
#include <R2-D2/lib/drive.h>

float sbus_velocity_interpolation(uint16_t channel_input,float *velocity_range)
{

	if (channel_input > channel_range[1])
	{
		return velocity_range[1];
	}

	if (channel_input < channel_range[0])
	{
		return velocity_range[0];
	}

	if (channel_input < 1005 && channel_input > 995) 
	{
		return (velocity_range[0] + velocity_range[1]) / 2;
	}
	float dchannel = channel_range[1] - channel_range[0];
	float dvel = velocity_range[1] - velocity_range[0];

	float  vel_interp = velocity_range[0] + (dvel / dchannel) * (channel_input - channel_range[0]);
	return vel_interp;
}

/*
int feedback_callback(float *feedback_buffer, int buffer_len, int wheels_per_side)
{
	return 0;
}
*/

int velocity_callback(const float *velocity_buffer, int buffer_len, int wheels_per_side)
{
	if (buffer_len < wheels_per_side * 2) 
	{
		return 1;
	}

	const int i = 0;
	if (pwm_motor_write(&(motor[i]), velocity_pwm_interpolation(*(velocity_buffer + i), wheel_velocity_range, pwm_range))) 
	{
		printk("Drive: Unable to write pwm pulse to Left : %d", i);
		return 1;
	}
	if (pwm_motor_write(&(motor[i + 1]), velocity_pwm_interpolation(*(velocity_buffer + wheels_per_side + i), wheel_velocity_range, pwm_range))) 
	{
		printk("Drive: Unable to write pwm pulse to Right : %d", i);
		return 1;
	}
	return 0;
}


uint32_t linear_actuator_pwm_interpolation(uint16_t linear_actuator_movement , uint32_t *pwm_range)
{
	if (linear_actuator_movement > 1400) 
	{
		return pwm_range[1];
	}

	if (linear_actuator_movement < 600) 
	{
		return pwm_range[0];
	}
	
		return 1500000;

}
int linear_actuator_write(int i, int dir){
	if(pwm_motor_write(&(motor[i]), linear_actuator_pwm_interpolation(dir, pwm_range)))
	{
		printk("Linear Actuator: Unable to write at linear actuator %d", i);
		return 1;
	}
}
