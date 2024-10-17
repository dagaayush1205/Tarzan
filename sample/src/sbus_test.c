#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

#include <Tarzan/lib/sbus.h>

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

// creating mssg queue to store data
K_MSGQ_DEFINE(uart_msgq, sizeof(uint8_t), 250, 1);

uint8_t packet[25]; // to store sbus packet

static const struct device *const uart_dev = DEVICE_DT_GET(DT_ALIAS(mother_uart));

void serial_cb(const struct device *dev, void *user_data) {
	ARG_UNUSED(user_data);
	uint8_t c, start = 0x0F;
	
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

int main() {

	int err;
	uint16_t *ch; // to store parsed channels 

	// device ready chceks
	if (!device_is_ready(uart_dev)) {
		LOG_ERR("UART device not ready");
	}

	// Interrupt to get sbus data
	err = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
	
	if(err<0) {
		if (err == -ENOTSUP) 
			LOG_ERR("Interrupt-driven UART API support not enabled");
		 
		else if (err == -ENOSYS) 
			LOG_ERR("UART device does not support interrupt-driven API");
		
		else 
			LOG_ERR("Error setting UART callback: %d", err);
		
	}

	// enable uart device for communication
	uart_irq_rx_enable(uart_dev);
	
	LOG_INF("Initialization completed successfully!");
	
	while(true)
	{
		k_msgq_get(&uart_msgq, &packet, K_MSEC(20));
		
		ch = parse_buffer(packet); 

		// printing channels 
		for(int i=0; i<16; i++)
			printk("%d\t", ch[i]);
		printk("\n"); 
	}
}
