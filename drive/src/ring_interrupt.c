#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h> 
#include <kyvernitis/lib/kyvernitis.h>

#include <Tarzan/lib/sbus.h>

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

static const struct device *const uart_dev = DEVICE_DT_GET(DT_ALIAS(mother_uart)); // data from SBUS
static const struct device *const uart_debug = DEVICE_DT_GET(DT_ALIAS(debug_uart)); //debugger

uint8_t ring_buffer[25];

struct ring_buf ringbuf;

uint16_t *ch;

void serial_cb(const struct device *dev, void *user_data) {
	
	ARG_UNUSED(user_data);
	while(uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		uint8_t c[25];
		int recv_len, rb_len;
		size_t len = MIN(ring_buf_space_get(&ringbuf), sizeof(c)); 

		recv_len = uart_fifo_read(dev, c, len);
		if(recv_len < 0) {
			LOG_ERR("Failed to read UART FIFO");
			recv_len = 0; 
		} 

		rb_len = ring_buf_put(&ringbuf, c, recv_len);
		if(rb_len < recv_len) {
			LOG_ERR("Drop %u bytes", recv_len - rb_len);
		}
	}
}
//	uint8_t c;
//	
//	if (!uart_irq_update(uart_dev)) 
//		return;
//	if (!uart_irq_rx_ready(uart_dev)) 
//		return;
//
//	while (uart_fifo_read(uart_dev, &c, 1) == 1) 
//		k_msgq_put(&uart_msgq, &c, K_NO_WAIT); // put message from UART to queue

// creating sbus packets from msg queue
int create_sbus_packet() {

	uint8_t packet[25],	// to store sbus packet
		start = 0x0F,  // start byte 
		message=0,   // to store a byte from queue
		i;

	ring_buf_put(&ringbuf, &message, sizeof(uint8_t));

	if(message == start) {
		for(i = 0; i < 25; i++) {
			packet[i] = message;
			k_msgq_get(&ringbuf, &message,K_MSEC(4));
		}
		ch = parse_buffer(packet);
		return 1;
	}

	else 
		return 0;
}

int main() {

	int err,flag=0;

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
	
	while(true) {
		// call to create sbus packet
		flag = create_sbus_packet();
	
		// check if sbus packet found
		if(flag == 0) {
			printk("Corrupt Packet");
			continue;
		}
		else { 
			for(int i=0; i<16;i++) {
				printk("%d \t", ch[i]);
			}
		}
			printk("\n");
	}
}
