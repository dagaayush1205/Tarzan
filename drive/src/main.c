#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>

#include <kyvernitis/lib/kyvernitis.h>

#include <R2-D2/lib/sbus.h>
#include <R2-D2/lib/drive.h>

#include <stdio.h>


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

// creating mssg queue to store data
K_MSGQ_DEFINE(uart_msgq, sizeof(uint8_t), 250, 1);

//struct mother_msg msg;
float linear_velocity_range[] = {-1.5, 1.5};
float angular_velocity_range[]= {-5.5,5.5};
float wheel_velocity_range[] = {-10.0, 10.0};
uint32_t pwm_range[] = {1120000, 1880000};
float la_speed_range[] = {-127.0, 127.0};
float angle_range[] = {-270, 270};
uint16_t channel_range[]={0,2047};

/* Global ticks for encoders */
//int64_t ticks_fr, ticks_fl;

struct DiffDriveTwist TIMEOUT_CMD = {
	.angular_z = 0,
	.linear_x = 0,
};

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

	// Angular and linear velocity
	struct DiffDriveTwist cmd = {
            .angular_z = 0,
            .linear_x = 0,
	};

	// device ready chceks
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

	// Interrupt to get sbus data
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

	// enable uart device for communication
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
// 			angular velocity interpolation
			cmd.angular_z = sbus_velocity_interpolation(ch[0],angular_velocity_range);
//			printk("%2d: %5d    %0.2f   ",1,ch[0], cmd.angular_z);
	
// 			linear velocity interpolation
			cmd.linear_x = sbus_velocity_interpolation(ch[1],linear_velocity_range);
//			printk("%2d: %5d    %0.2f   ",2,ch[1],cmd.linear_x);
				

// 			linear actuators print
//			printk("%2d: %5d    ",3,ch[2]);
//			printk("%2d: %5d    ",4,ch[3]);
//
//
// 			arm joint print
//			printk("%2d: %5d    ",5,ch[4]);
//			printk("%2d: %5d    ",6,ch[5]);
//
//			turntable print
//			printk("%2d: %5d    ",7,ch[6]);
//
//			printk("\n");
		}
		
		drive_timestamp = k_uptime_get();

// 		drive write
		err = diffdrive_update(drive, cmd, time_last_drive_update); 

// 		linear actuators interpolate and write
		linear_actuator_write(2,ch[2]); 			     		
		linear_actuator_write(3,ch[3]);

// 		arm joint interpolate and write		
		linear_actuator_write(4,ch[4]);
		linear_actuator_write(5,ch[5]);

		time_last_drive_update = k_uptime_get() - drive_timestamp;
	}
}
