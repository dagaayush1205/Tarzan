
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

static const struct device *const uart_dev = DEVICE_DT_GET(DT_ALIAS(mother_uart)); // data from SBUS
static const struct device *const uart_debug = DEVICE_DT_GET(DT_ALIAS(debug_uart)); //debugger

/* DT spec for encoders */
// const struct device *const encoder_fr = DEVICE_DT_GET(DT_ALIAS(en_fr));
// const struct device *const encoder_fl = DEVICE_DT_GET(DT_ALIAS(en_fl));

/* DT spec for pwm motors */
#define PWM_MOTOR_SETUP(pwm_dev_id)                                                                \
	{.dev_spec = PWM_DT_SPEC_GET(pwm_dev_id),                                                  \
	 .min_pulse = DT_PROP(pwm_dev_id, min_pulse),                                              \
	 .max_pulse = DT_PROP(pwm_dev_id, max_pulse)},

struct pwm_motor motor[13] = {DT_FOREACH_CHILD(DT_PATH(pwmmotors), PWM_MOTOR_SETUP)};

K_MSGQ_DEFINE(uart_msgq, sizeof(uint8_t), 250, 1);

//struct mother_msg msg;
uint8_t message;
float linear_velocity_range[] = {-1.5, 1.5};
float angular_velocity_range[]= {-5.5,5.5};
float wheel_velocity_range[] = {-10.0, 10.0};
uint32_t pwm_range[] = {1120000, 1880000};
float la_speed_range[] = {-127.0, 127.0};
float angle_range[] = {-270, 270};
uint16_t channel_range[]={0,2047};
uint16_t* ch;

/* Global ticks for encoders */
//int64_t ticks_fr, ticks_fl;

struct DiffDriveTwist TIMEOUT_CMD = {
	.angular_z = 0,
	.linear_x = 0,
};


int sbus_parsing() {
	uint8_t packet[25],packet_pos=0,start = 0x0F, end = 0x00;

	k_msgq_get(&uart_msgq, &message,K_MSEC(4));

	if(message == start) 
	{
		for(packet_pos = 0; packet_pos < 25; packet_pos++) 
		{
			packet[packet_pos] = message;
			k_msgq_get(&uart_msgq, &message,K_MSEC(4));
		}
		// check if the last byte was 0x00
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
        	k_msgq_put(&uart_msgq, &c, K_NO_WAIT);
	}
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

	struct DiffDriveTwist cmd = {
            .angular_z = 0,
            .linear_x = 0,
	};

	if (!device_is_ready(uart_dev)) 
	{
		printk( "UART device not ready");
	}

	for (size_t i = 0U; i < ARRAY_SIZE(motor); i++) 
	{
		if (!pwm_is_ready_dt(&(motor[i].dev_spec))) 
		{
			printk( "PWM: Motor %s is not ready", motor[i].dev_spec.dev->name);
		}
	}

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
		flag=sbus_parsing();
		if(flag == 0)
		{
			printk("Error:Could not find start bit\n"); 
			continue;
		}
		 else 
		 {
			 for(i = 0 ; i < 15 ; i++)
			 {
				//	cmd.linear_x = sbus_velocity_interpolation(ch[i],linear_velocity_range);
					printk("%d: %d \t",i,ch[i]);
		 	}
			printk("\n");
		 }

		drive_timestamp = k_uptime_get();
		err = diffdrive_update(drive, cmd, time_last_drive_update);
		time_last_drive_update = k_uptime_get() - drive_timestamp;
	}
}
