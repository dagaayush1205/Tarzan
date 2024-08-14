#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <kyvernitis/lib/kyvernitis.h>
#include <stdio.h>
#include <string.h>

static const struct device *const uart_dev = DEVICE_DT_GET(DT_ALIAS(mother_uart));
#define UART_MSG_SIZE (sizeof(uint8_t)*25)
K_MSGQ_DEFINE(uart_msgq, sizeof(uint8_t)*25, 10, 1);
struct mother_msg msg;
	//void recv_str(const struct device *uart, char *str)
	//{
	//        char *head = str;
	//        char c;
	//
	//        while (!uart_poll_in(uart, &c)) {
	//                *head++ = c;
	//        }
	//        *head = '\0';
	//
	//        printk("Device %s received: \"%s\"\n", uart->name, str);
	//}
	//
	//
void serial_cb(const struct device *dev, void *user_data){
	uint8_t c;
	uint8_t data[25];
	int buf_pos=0;
	if (!uart_irq_update(uart_dev)) {
		return;
	}

	if (!uart_irq_rx_ready(uart_dev)) {
		return;
	}
	while (uart_fifo_read(uart_dev, &c, 1) == 1) {
//		data[buf_pos]=c;
//		buf_pos++;
//	}
//	buf_pos=0;
        k_msgq_put(&uart_msgq, &c, K_NO_WAIT);
//	printk("print c:%hhx\n",c);
	}
}
int main(){
	//char recv_buf[64];
	int err;
	printk("I am live");
	err = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
	if(err<0)
	{
		printk("Error");
	}
	uart_irq_rx_enable(uart_dev);
	
	while(true){

		printk("Message: %x \n",k_msgq_get(&uart_msgq, &msg,K_FOREVER)); 
//		k_sleep(K_MSEC(100));	
		
//		while(true)
//		{
//			recv_str(uart_dev, recv_buf);
//		}
		}
}
