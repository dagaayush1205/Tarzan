#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

#include <kyvernitis/lib/kyvernitis.h>

#include "sbus_parse.c"
//#include <Tarzan/lib/sbus.h>
//#include <Tarzan/lib/drive.h>

LOG_MODULE_REGISTER(stepper_test, CONFIG_LOG_DEFAULT_LEVEL);
static const struct device *const uart_dev = DEVICE_DT_GET(DT_ALIAS(mother_uart)); // data from SBUS
static const struct device *const uart_debug = DEVICE_DT_GET(DT_ALIAS(debug_uart)); //debugger

// DT spec for stepper 
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

// creating mssg queue to store data
K_MSGQ_DEFINE(uart_msgq, sizeof(uint8_t), 250, 1);

// ranges used in interpolation;
uint16_t channel_range[]= {0,950, 2047};

uint16_t *ch;
uint8_t packet[25];
int pos[3] ={ 0};
uint64_t time[3], last_time[3];
float stepInterval;

int sbus_parsing() {
  uint8_t packet[25], packet_pos = 0, start = 0x0f, message = 0;

  k_msgq_get(&uart_msgq, &message, K_MSEC(4));

  if (message == start) {
    for (packet_pos = 0; packet_pos < 25; packet_pos++) {
      packet[packet_pos] = message;
      k_msgq_get(&uart_msgq, &message, K_MSEC(4));
    }
    ch = parse_buffer(packet);
    return 1;
  } else {
    return 0;
  }
}

void serial_cb(const struct device *dev, void *user_data) {
  uint8_t c;

  if (!uart_irq_update(uart_dev)) {
    return;
  }

  if (!uart_irq_rx_ready(uart_dev)) {
    return;
  }

  while (uart_fifo_read(uart_dev, &c, 1) == 1) {
    k_msgq_put(&uart_msgq, &c, K_NO_WAIT); // put message from UART to queue
  }
}
void setSpeed(float speed){
	if(speed == 0.0) 
		stepInterval = 0.0;
	else 
		stepInterval = (60000000.0/(6400.0*speed*3.0));
}

int Stepper_motor_write(const struct stepper_motor *motor, uint16_t cmd, int pos) { 
	
  if (ch > channel_range[1]) {
    gpio_pin_set_dt(&(motor->dir), 1);
    pos += 1; // clockwise
  } else {
    gpio_pin_set_dt(&(motor->dir), 0);
    pos -= 1;
  } // anticlockwise
  switch (pos & 0x03) {
  case 0:
    gpio_pin_set_dt(&(motor->step), 0);
    break;
  case 1:
    gpio_pin_set_dt(&(motor->step), 1);
    break;
  case 2:
    gpio_pin_set_dt(&(motor->step), 1);
    break;
  case 3:
    gpio_pin_set_dt(&(motor->step), 0);
    break;
  }
  return pos;
} 

void arm_joints() {
//  setSpeed(500000.0);
  // Stepper Motor Forward
  for(int i=0;i<3;i++){
  time[i] = k_uptime_ticks();
  if ((time[i] - last_time[i]) >= 25) {
    pos[i] = Stepper_motor_write(&stepper[i], ch, pos[i]);
  last_time[i] = time[i];
  	}
  }
}

int main() {

	int err;
	uint16_t c = 2000;

  if (!device_is_ready(uart_dev)) {
    LOG_ERR("UART device not ready");
  }

  err = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);

  if (err < 0) {
    if (err == -ENOTSUP) {
      printk("Interrupt-driven UART API support not enabled");
    } else if (err == -ENOSYS) {
      printk("UART device does not support interrupt-driven API");
    } else {
      printk("Error setting UART callback: %d", err);
    }
  }

  if (err < 0) {
    if (err == -ENOTSUP)
      printk("Interrupt-driven UART API support not enabled");

    else if (err == -ENOSYS)
      printk("UART device does not support interrupt-driven API");

    else
      printk("Error setting UART callback: %d", err);
  }

  uart_irq_rx_enable(uart_dev);

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

	LOG_INF("Initialization completed successfully!");
	
	while(true)
	{
		k_msgq_get(&uart_msgq, &packet, K_NO_WAIT);
		
		ch = parse_buffer(packet); 

		// first link 
		arm_joints();
	//	arm_joints(1, c);
	//	arm_joints(2, c);

	}
}
