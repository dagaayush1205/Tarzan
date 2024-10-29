#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <kyvernitis/lib/kyvernitis.h>
#include <stdio.h>
#include <string.h>
#include "sbus_parse.c"
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

#define PULSE_PER_REV 6400
#define MINUTES_TO_MICRO 60000000
LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

static const struct device *const uart_dev = DEVICE_DT_GET(DT_ALIAS(mother_uart)); // data from SBUS
static const struct device *const uart_debug = DEVICE_DT_GET(DT_ALIAS(debug_uart)); //debugger

/* DT spec for pwm motors */
#define PWM_MOTOR_SETUP(pwm_dev_id)                                                                \
	{.dev_spec = PWM_DT_SPEC_GET(pwm_dev_id),                                                  \
	 .min_pulse = DT_PROP(pwm_dev_id, min_pulse),                                              \
	 .max_pulse = DT_PROP(pwm_dev_id, max_pulse)},

struct pwm_motor motor[13] = {DT_FOREACH_CHILD(DT_PATH(pwmmotors), PWM_MOTOR_SETUP)};

const struct stepper_motor stepper[3] = {
	{
		.dir = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor1), dir_gpios),
		.step = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor1), step_gpios)
	},
	{
		.dir = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor2), dir_gpios), 
		.step = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor2), step_gpios)
	},
	{
		.dir = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor3), dir_gpios), 
		.step = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor3), step_gpios)
	}
}; 
K_MSGQ_DEFINE(uart_msgq, sizeof(uint8_t), 250, 1);

//struct mother_msg msg;
float linear_velocity_range[] = {-1.5, 1.5};
float angular_velocity_range[]= {-5.5,5.5};
float wheel_velocity_range[] = {-10.0, 10.0};
uint32_t pwm_range[] = {1120000, 1880000};
float la_speed_range[] = {-127.0, 127.0};
float angle_range[] = {-270, 270};
uint16_t channel_range[]={0, 940, 2047};
uint16_t* ch;

/* Global ticks for encoders */
//int64_t ticks_fr, ticks_fl;
int pos1, pos2, pos3; 
uint64_t time, last_time; 
float stepInterval;

struct DiffDriveTwist TIMEOUT_CMD = {
	.angular_z = 0,
	.linear_x = 0,
};

void setSpeed(float speed){
	if(speed = 0.0) 
		stepInterval = 0.0;
	else 
		stepInterval = (1/3*PULSE_PER_REV*speed)*MINUTES_TO_MICRO;
}

int Stepper_motor_write(const struct stepper_motor *motor, uint16_t ch,int pos) { 
	
	if(ch > channel_range[1]) {
		gpio_pin_set_dt(&(motor->dir), 1); 
		pos +=1; //clockwise
	}
	else { 
		gpio_pin_set_dt(&(motor->dir), 0); 
		pos -=1;
	}	//anticlockwise 
	switch(pos & 0x03) { 
		case 0: gpio_pin_set_dt(&(motor->step),0);	//(0b10&(1<<0))?1:0); 
			break; 
		case 1: gpio_pin_set_dt(&(motor->step),1);	//(0b11&(1<<0))?1:0);  
			break; 
		case 2: gpio_pin_set_dt(&(motor->step),1);	//(0b01&(1<<0))?1:0);  
			break; 
		case 3: gpio_pin_set_dt(&(motor->step),0);	//(0b00&(1<<0))?1:0); 
			break;
	}
	return pos;
}
int arm_joints(int motor, uint16_t ch, int pos) {

	setSpeed(60);
	// Stepper Motor Forward 
	time = k_uptime_ticks(); 
	if((time-last_time)>=stepInterval){  
		pos = Stepper_motor_write(&stepper[motor], ch, pos);  
	}	
		last_time = time; 
	
	return pos;
}
int sbus_parsing() {
	uint8_t packet[25],packet_pos=0,start = 0x0F, end = 0x00, message=0;

	k_msgq_get(&uart_msgq, &message, K_MSEC(4));

	if(message == start) 
	{
		for(packet_pos = 0; packet_pos < 25; packet_pos++) 
		{
			packet[packet_pos] = message;
			k_msgq_get(&uart_msgq, &message,K_MSEC(4));
		}
		LOG_INF("%o %o %o",packet[0],packet[23], packet[24]);
		ch = parse_buffer(packet);
		return 1;
	} 
	else 
	{
		return 0;
	}
}

void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t c;
	
	if (!uart_irq_update(uart_dev)) 
	{
		return;
	}

	if (!uart_irq_rx_ready(uart_dev)) 
	{
		return;
	}

	while (uart_fifo_read(uart_dev, &c, 1) == 1) 
	{
		k_msgq_put(&uart_msgq, &c, K_NO_WAIT); // put message from UART to queue
	}
}

float one_hot_interpolation(uint16_t channel_input) {

	if (channel_input > channel_range[1])
	{
		return pwm_range[1];
	}

	if (channel_input < channel_range[0])
	{
		return pwm_range[0];
	}

	if (channel_input < 1005 && channel_input > 995) 
	{
		return (pwm_range[0] + pwm_range[1]) / 2;
	}
	float dchannel = channel_range[1] - channel_range[0];
	float dpwm = pwm_range[1] - pwm_range[0];

	uint32_t pwm_interp = pwm_range[0] + (dpwm / dchannel) * (channel_input - channel_range[0]);

	return pwm_interp;
}

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

int feedback_callback(float *feedback_buffer, int buffer_len, int wheels_per_side)
{
	return 0;
}

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

int arm_joints_write(int i, uint16_t ch){
	if(pwm_motor_write(&(motor[i]), one_hot_interpolation(ch)))
	{
		printk("Linear Actuator: Unable to write at linear actuator %d", i);
		return 1;
	}
}

//void arm_joints(int motor, uint16_t ch, int pos) {
//
//	setSpeed(2);
//	// Stepper Motor Forward 
//	if((ch>channel_range[1])) {
//		time = k_uptime_ticks(); 
//		if((time-last_time)>=i){  
//			Stepper_motor_write(&stepper[motor], 1, pos))  
//		}
//			last_time = time; 
//	}
//	// Stepper Motor Backward
//	else { 
//		if(Stepper_motor_write(&stepper[motor], 2)) {
//				printk("Unable to write motor command to Stepper %d", stepper[motor]);
//				return 0; 
//		}
//	}
//}


int main(){
	int err,i,flag=0;
	uint64_t drive_timestamp = 0;
	uint64_t time_last_drive_update = 0;

	struct DiffDriveConfig drive_config = { 
		.wheel_separation = 0.77f,
		.wheel_separation_multiplier = 1,
		.wheel_radius = 0.15f,
		.wheels_per_side = 2,
		.command_timeout_seconds = 2,
		.left_wheel_radius_multiplier = 1,
		.right_wheel_radius_multiplier = 1,
		.update_type = POSITION_FEEDBACK,
	};
// 	Angular and linear velocity
	struct DiffDriveTwist cmd = {
            .angular_z = 0,
            .linear_x = 0,
	};

	if (!device_is_ready(uart_dev)) 
	{
		LOG_ERR("UART device not ready");
	}

	for (size_t i = 0U; i < ARRAY_SIZE(motor); i++) 
	{
		if (!pwm_is_ready_dt(&(motor[i].dev_spec))) 
		{
			LOG_ERR("PWM: Motor %s is not ready", motor[i].dev_spec.dev->name);
		}
	}
// 	Interrupt
	err = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
	
	if(err<0)
	{
		if (err == -ENOTSUP) 
		{
			printk("Interrupt-driven UART API support not enabled");
		} 
		else if (err == -ENOSYS) 
		{
			printk("UART device does not support interrupt-driven API");
		}
		else 
		{
			printk("Error setting UART callback: %d", err);
		}
	}


	uart_irq_rx_enable(uart_dev);
	
	struct DiffDrive *drive = diffdrive_init(&drive_config, feedback_callback, velocity_callback);	

	for (size_t i = 0U; i < ARRAY_SIZE(motor); i++) 
	{
		if (pwm_motor_write(&(motor[i]), 1500000)) 
		{
			printk("Unable to write pwm pulse to PWM Motor : %d", i);
		}
	}

	printk("Initialization completed successfully!");
	
	while(true)
	{
// 		parse uart data return 1 if start bit not found
		flag=sbus_parsing();
		if(flag == 0)
		{
			continue;
		}
		else 
		{
        drive_timestamp = k_uptime_get();
        
	for(int i = 0 ; i < 16 ; i++)
        {
          printk("%d \t", ch[i]);
	}
        printk("\n");

        pos1 = arm_joints(0, ch[4], pos1); // turn-table
        pos2 = arm_joints(1, ch[5], pos2); // Line 1(turn-table)
        pos3 = arm_joints(2, ch[6], pos3); // Link 2

        arm_joints_write(7, ch[7]); // ABox




      if(ch[8] > 300)
      {
        cmd.angular_z = sbus_velocity_interpolation(ch[0], angular_velocity_range);
        cmd.linear_x = sbus_velocity_interpolation(ch[1], linear_velocity_range);

        err = diffdrive_update(drive, cmd, time_last_drive_update);
        
        linear_actuator_write(2, ch[2]);
        linear_actuator_write(3, ch[3]);

      }
      else{
        arm_joints_write(8, ch[0]); // Y of YPR
        arm_joints_write(9, ch[1]); // P of YPR
        
        arm_joints_write(10, ch[2]); //Gripper1
        arm_joints_write(13, ch[3]); //Gripper2
        arm_joints_write(14, ch[8]); //R of YPR

      }
  		time_last_drive_update = k_uptime_get() - drive_timestamp;
    }

	}
}
