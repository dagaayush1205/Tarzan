#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

#include <kyvernitis/lib/kyvernitis.h>

#include <Tarzan/lib/sbus.h>
#include <Tarzan/lib/drive.h>


static const struct device *const uart_dev = DEVICE_DT_GET(DT_ALIAS(mother_uart)); // data from SBUS
static const struct device *const uart_debug = DEVICE_DT_GET(DT_ALIAS(debug_uart)); //debugger

// DT spec for stepper 
const struct stepper_motor stepper[3] = {
	{
		.dir = GPIO_DT_SPEC_GET(DT_ALIAS(stepper-motor1), dir_gpios),
		.step = GPIO_DT_SPEC_GET(DT_ALIAS(stepper-motor1), step_gpios)
	},
	{
		.dir = GPIO_DT_SPEC_GET(DT_ALIAS(stepper-motor2), dir_gpios), 
		.step = GPIO_DT_SPEC_GET(DT_ALIAS(stepper-motor2), step_gpios)
	},
	{
		.dir = GPIO_DT_SPEC_GET(DT_ALIAS(stepper-motor3), dir_gpios), 
		.step = GPIO_DT_SPEC_GET(DT_ALIAS(stepper-motor3), step_gpios)
	}
}; 

// creating mssg queue to store data
K_MSGQ_DEFINE(uart_msgq, sizeof(uint8_t), 250, 1);

// ranges used in interpolation;
float linear_velocity_range[] = {-1.5, 1.5};
float angular_velocity_range[]= {-5.5,5.5};
float wheel_velocity_range[] = {-10.0, 10.0};
uint32_t pwm_range[] = {1120000, 1500000, 1880000};
uint16_t channel_range[]= {0,950, 2047};

uint16_t *ch;
uint8_t packet[25];

void serial_cb(const struct device *dev, void *user_data) {
	ARG_UNUSED(user_data);
	uint8_t start= 0x0F; 
	uint8_t c;

	if (!uart_irq_update(uart_dev)) 
		return;
	if (!uart_irq_rx_ready(uart_dev)) 
		return;

	while (uart_fifo_read(uart_dev, &c, 1) == 1) {
		if(c != start) continue; 
		packet[0] = c; 
		if(uart_fifo_read(uart_dev, packet+1, 24) == 24)  
			k_msgq_put(&uart_msgq, packet, K_NO_WAIT);
	}
}

int arm_joints(int motor, uint8_t ch) {

	if((ch>channel_range[1])) { 
		if(stepper_motor_write(&stepper[motor], 1)) { 
				printk("Unable to write motor command to Stepper %d", stepper[motor]); 
				return 0; 
		}
	}
	else { 
		if(stepper_motor_write(&stepper[motor], 2)) {
				printk("Unable to write motor command to Stepper %d", stepper[motor]);
				return 0; 
		}
	}
}

int main() {

	int err;

	// device ready chceks
	if (!device_is_ready(uart_dev)) {
		printk("UART device not ready");
	}

	// Interrupt to get sbus data
	err = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
	
	if(err<0) {
		if (err == -ENOTSUP) 
			printk("Interrupt-driven UART API support not enabled");
		 
		else if (err == -ENOSYS) 
			printk("UART device does not support interrupt-driven API");
		
		else 
			printk("Error setting UART callback: %d", err);
		
	}
	
	for(size_t i = 0U; i < 3; i++) { 
		if(!gpio_is_ready_dt(&stepper[i].dir)) {
			printk("Stepper Motor %d: Dir %d is not ready\n", i, stepper[i].dir.pin);
			return 0; 
		}
		if(!gpio_is_ready_dt(&stepper[i].step)) {
			printk("Stepper Motor %d: Dir %d is not ready\n", i, stepper[i].step.pin);
			return 0; 
		}
	}
	
	for (size_t i = 0U; i < 3; i++) {
		if(gpio_pin_configure_dt(&(stepper[i].dir), GPIO_OUTPUT_INACTIVE)) {
			printk("Error: Stepper motor %d: Dir %d not configured", i,
							stepper[i].dir.pin);
			return 0;
		}
		if(gpio_pin_configure_dt(&(stepper[i].step), GPIO_OUTPUT_INACTIVE)) {
			printk("Error: Stepper motor %d: Dir %d not configured", i,
							stepper[i].step.pin);
			return 0;
		}	
	}
	// enable uart device for communication
	uart_irq_rx_enable(uart_dev);
	
	printk("Initialization completed successfully!");
	
	while(true)
	{
		k_msgq_get(&uart_msgq, &packet, K_MSEC(20));
		
		ch = parse_buffer(packet); 

		// printing channels 
		for(int i=0; i<16; i++)
			printk("%d\t", ch[i]);
		printk("\n"); 

		// first link 
		arm_joints(0, ch[6]);

		// second link 
		arm_joints(1, ch[7]);
		
		// turn table 
		arm_joints(2, ch[8]);
	}
}
