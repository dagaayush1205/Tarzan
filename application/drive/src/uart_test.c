#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <kyvernitis/lib/kyvernitis.h>
#include <stdio.h>
#include <string.h>
#include "sbus_parse.c"

static const struct device *const uart_dev = DEVICE_DT_GET(DT_ALIAS(mother_uart));
//#define UART_MSG_SIZE (sizeof(uint8_t)*25)
K_MSGQ_DEFINE(uart_msgq, sizeof(uint8_t)*25, 10, 1);
//struct mother_msg msg;
uint8_t message;
float vel_range[] = {-10, 10};
//uint32_t pwm_range[] = {1120000, 1880000};
float la_speed_range[] = {-127.0, 127.0};
//uint32_t pid_pwm_range[] = {1300000, 1700000};
float angle_range[] = {-270, 270};

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
uint16_t *sbus_parsing() {
	uint8_t packet[25],packet_pos=0,start = 0x0F, end = 0x00;	
	uint16_t ch[16];
	if(message == start) {
	for(packet_pos = 0; packet_pos < 25; packet_pos++) {
		k_msgq_get(&uart_msgq, &message,K_MSEC(4));
		packet[packet_pos] = message;
		}
	}

	*ch = parse_buffer(packet);

	if(packet[24] == end)
		return ch;
	
	else 
		return -1;
}

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
uint16_t velocity_interpolation(uint16_t val)
{
	val-=1024;
	val=(val/1024)*vel_range[1];
        return val;
}

int main(){
	uint16_t channels[16];
	int err,i;
	printk("I am alive");
	err = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
	if(err<0)
	{
		printk("Error");
	}
	uart_irq_rx_enable(uart_dev);
	
	while(true){
		*channels=sbus_parsing();
		 if(channels == -1)
			 continue; 
		 else {
			 for(i=0;i<16;i++)
			 {
				 channels[i]=velocity_interpolation(channels[i]);
				 printk("channel: \n%d",channels[i]);
			 }
		 }

//		printk("Message: %hhx \n", message); 
//		k_sleep(K_MSEC(100));	
		
//		while(true)
//		{
//			recv_str(uart_dev, recv_buf);
//		}
		
		
		
	}
}

