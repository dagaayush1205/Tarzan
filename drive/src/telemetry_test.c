#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h> 

#include <kyvernitis/lib/kyvernitis.h>

#include <Tarzan/lib/sbus.h>
#include <Tarzan/lib/drive.h>

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

static const struct device *const uart_dev = DEVICE_DT_GET(DT_ALIAS(mother_uart)); // data from SBUS
static const struct device *const uart_debug = DEVICE_DT_GET(DT_CHOSEN(zephyr_console)); //debugger

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

int create_sbus_packet() {
	uint8_t packet[25],
		packet_pos=0,
		start = 0x0F;

	k_msgq_get(&uart_msgq, &message,K_MSEC(4));

	if(message == start) {
		for(packet_pos = 0; packet_pos < 25; packet_pos++) {
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

void serial_cb(const struct device *dev, void *user_data) {
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

//void telemetry_callback(const struct device *dev, void *user_data) { 
//	ARG_UNUSED(user_data); 
//
//	while(uart_irq_update(dev) && uart_irq_is_pending(dev)) {
//		if(uart_irq_tx_ready(dev)) { 
//			uart_fifo_fill(dev, ch, 16*sizeof(uint16_t));
//		}
//	}

int main() {

	int err,i,flag=0;

	if(!device_is_ready(uart_debug)) { 
		LOG_ERR("CDC ACM device is not ready"); 
	}

	if (!device_is_ready(uart_dev)) {
		LOG_ERR( "UART device not ready");
	}

	err = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
	
	if(err<0) {
		if (err == -ENOTSUP) {
			LOG_ERR("Interrupt-driven UART API support not enabled");
		} 
		else if (err == -ENOSYS) {
			LOG_ERR("UART device does not support interrupt-driven API");
		}
		else {
			LOG_ERR("Error setting UART callback: %d", err);
		}
}

	uart_irq_rx_enable(uart_dev);
	
	LOG_ERR("Initialization completed successfully!");
	
	while(true) {
		flag=create_sbus_packet();
		if(flag == 0)
			continue;
		 else {
			  LOG_INF("%hhx",ch[i]);	
			 // uart_irq_callback_set(dev, interrupt_handler);
		 	}
		 }
}
