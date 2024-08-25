#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
//#include <zephyr/logging/log.h>

#include <kyvernitis/lib/kyvernitis.h>

#include <Tarzan/lib/sbus.h>
#include <Tarzan/lib/drive.h>

static const struct device *const uart_dev = DEVICE_DT_GET(DT_ALIAS(mother_uart)); // data from SBUS
static const struct device *const uart_debug = DEVICE_DT_GET(DT_ALIAS(debug_uart)); //debugger

/* DT spec for pwm motors */
#define PWM_MOTOR_SETUP(pwm_dev_id) {                            \
	.dev_spec = PWM_DT_SPEC_GET(pwm_dev_id),                 \
	 .min_pulse = DT_PROP(pwm_dev_id, min_pulse),            \
	 .max_pulse = DT_PROP(pwm_dev_id, max_pulse)},


struct pwm_motor motor[13] = {DT_FOREACH_CHILD(DT_PATH(pwmmotors), PWM_MOTOR_SETUP)};

// creating mssg queue to store data
K_MSGQ_DEFINE(uart_msgq, sizeof(uint8_t), 250, 1);

// ranges used in interpolation;
float linear_velocity_range[] = {-1.5, 1.5};
float angular_velocity_range[]= {-5.5,5.5};
float wheel_velocity_range[] = {-10.0, 10.0};
uint32_t pwm_range[] = {1120000, 1500000, 1880000};
// float la_speed_range[] = {-127.0, 127.0};
// float angle_range[] = {-270, 270};
uint16_t channel_range[]= {0,2047};

uint16_t *ch;

void serial_cb(const struct device *dev, void *user_data) {
	uint8_t c;
	
	if (!uart_irq_update(uart_dev)) 
		return;
	if (!uart_irq_rx_ready(uart_dev)) 
		return;

	while (uart_fifo_read(uart_dev, &c, 1) == 1) 
		k_msgq_put(&uart_msgq, &c, K_NO_WAIT); // put message from UART to queue
}

// creating sbus packets from msg queue
int create_sbus_packet() {

	uint8_t packet[25],	// to store sbus packet
		start = 0x0F,  // start byte 
		message=0,   // to store a byte from queue
		i;

	k_msgq_get(&uart_msgq, &message,K_MSEC(4));

	if(message == start) {
		for(i = 0; i < 25; i++) {
			packet[i] = message;
			k_msgq_get(&uart_msgq, &message,K_MSEC(4));
		}
		ch = parse_buffer(packet);
		return 1;
	}

	else 
		return 0;
}

int velocity_callback(const float *velocity_buffer, int buffer_len, int wheels_per_side) {
	
	if (buffer_len < wheels_per_side * 2) 
		return 1;
	
	if (pwm_motor_write(&(motor[0]), velocity_pwm_interpolation(*(velocity_buffer), wheel_velocity_range, pwm_range))) {
//		LOG_ERR("Drive: Unable to write pwm pulse to Left");
		return 1;
	}
	if (pwm_motor_write(&(motor[1]), velocity_pwm_interpolation(*(velocity_buffer + wheels_per_side), wheel_velocity_range, pwm_range))) {
//		LOG_ERR("Drive: Unable to write pwm pulse to Right");
		return 1; 
	} 
	return 0;
}

int actuator_callback(int i, uint8_t channel){
	if(pwm_motor_write(&(motor[i]), one_hot_interpolation(channel, pwm_range))) { 
//		LOG_ERR("Linear Actuator: Unable to write at linear actuator %d", i); 
		return 1; 
	}
	return 0;
}

int feedback_callback() {
	return 0;
}

int main() {

	int err,flag=0;
	uint64_t drive_timestamp = 0;
	uint64_t time_last_drive_update = 0;


	// Angular and linear velocity
	struct DiffDriveTwist cmd = {
		.angular_z = 0,
		.linear_x = 0,
	};

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

	// device ready chceks
	if (!device_is_ready(uart_dev)) 
	{
//		LOG_ERR("UART device not ready");
	}

	for (size_t i = 0U; i < ARRAY_SIZE(motor); i++) 
	{
		if (!pwm_is_ready_dt(&(motor[i].dev_spec))) 
		{
//			LOG_ERR( "PWM: Motor %s is not ready", motor[i].dev_spec.dev->name);
		}
	}

	// Interrupt to get sbus data
	err = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
	
	if(err<0)
	{
		if (err == -ENOTSUP) 
		{
//			LOG_ERR("Interrupt-driven UART API support not enabled");
		} 
		else if (err == -ENOSYS) 
		{
//			LOG_ERR("UART device does not support interrupt-driven API");
		}
		else 
		{
//			LOG_ERR("Error setting UART callback: %d", err);
		}
	}

	// enable uart device for communication
	uart_irq_rx_enable(uart_dev);
	
	struct DiffDrive *drive = diffdrive_init(feedback_callback(),&drive_config, velocity_callback);	

	for (size_t i = 0U; i < ARRAY_SIZE(motor); i++) 
	{
		if (pwm_motor_write(&(motor[i]), 1500000)) 
		{
//			LOG_ERR("Unable to write pwm pulse to PWM Motor : %d", i);
		}
	}

//	LOG_ERR("Initialization completed successfully!");
	
	while(true)
	{
		// call to create sbus packet
		flag = create_sbus_packet();
	
		// check if sbus packet found
		if(flag == 0)
		continue;

		// angular velocity interpolation
		cmd.angular_z = sbus_velocity_interpolation(ch[0],angular_velocity_range, channel_range);

		// linear velocity interpolation
		cmd.linear_x = sbus_velocity_interpolation(ch[1],linear_velocity_range, channel_range);
		
		drive_timestamp = k_uptime_get();

		// drive write
		err = diffdrive_update(drive, cmd, time_last_drive_update); 

		// linear actuators interpolate and write
		actuator_callback(2,ch[2]); 			     		
		actuator_callback(3,ch[3]);

		// arm joint interpolate and write		
		actuator_callback(4,ch[4]);
		actuator_callback(5,ch[5]);

		time_last_drive_update = k_uptime_get() - drive_timestamp;
	}
}
