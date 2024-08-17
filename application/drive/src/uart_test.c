#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <kyvernitis/lib/kyvernitis.h>
#include <stdio.h>
#include <string.h>
#include "sbus_parse.c"

static const struct device *const uart_dev = DEVICE_DT_GET(DT_ALIAS(mother_uart));
static const struct device *const uart_debug = DEVICE_DT_GET(DT_ALIAS(debug_uart));
//#define UART_MSG_SIZE (sizeof(uint8_t)*25)
K_MSGQ_DEFINE(uart_msgq, sizeof(uint8_t), 250, 1);
//struct mother_msg msg;
uint8_t message;
float linear_velocity_range[] = {-1.5, 1.5};
float angular_velocity_range[]= {-5.5,5.5};
//uint32_t pwm_range[] = {1120000, 1880000};
float la_speed_range[] = {-127.0, 127.0};
//uint32_t pid_pwm_range[] = {1300000, 1700000};
float angle_range[] = {-270, 270};
uint16_t channel_range[]={0,2047};
uint16_t* ch;

int sbus_parsing() {
	uint8_t packet[25],packet_pos=0,start = 0x0F, end = 0x00;

	k_msgq_get(&uart_msgq, &message,K_MSEC(4));

	if(message == start) {
		for(packet_pos = 0; packet_pos < 25; packet_pos++) {
			packet[packet_pos] = message;
			k_msgq_get(&uart_msgq, &message,K_MSEC(4));
		}
		// check if the last byte was 0x00
		ch = parse_buffer(packet);
		return 1;
	} else {
		return 0;
	}

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
        k_msgq_put(&uart_msgq, &c, K_NO_WAIT);
	}
}
float sbus_velocity_interpolation(uint16_t channel_input,float *velocity_range)
{
	if (channel_input > channel_range[1]) {
		return velocity_range[1];
	}

	if (channel_input < channel_range[0]) {
		return velocity_range[0];
	}

	if (channel_input < 1005 && channel_input > 995) {
		return (velocity_range[0] + velocity_range[1]) / 2;
	}
	else{
	float dchannel = channel_range[1] - channel_range[0];
	float dvel = velocity_range[1] - velocity_range[0];

	float  vel_interp = velocity_range[0] + (dvel / dchannel) * (channel_input - channel_range[0]);
	return vel_interp;}
}

int main(){
	int err,i,flag=0;

	err = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
	if(err<0)
	{
		printk("Error");
	}
	uart_irq_rx_enable(uart_dev);
	
	while(true){
		flag=sbus_parsing();
		if(flag == 0)
			 continue; 
		 else {
			 for(i=0;i<16;i++)
			 {
				 if(i==2)
					printk("%d: %d \t %0.2f \t",i,ch[i], sbus_velocity_interpolation(ch[i],linear_velocity_range));
				  if(i==0)
					printk("%d: %d \t %0.2f \t",i,ch[i], sbus_velocity_interpolation(ch[i],angular_velocity_range));
		//		 else
		//		 	printk("%d: %d \t",i,ch[i]);
			 }
			 printk("\n");
		 }
	}
}

