#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <kyvernitis/lib/kyvernitis.h>
#include <stdio.h>
#include <string.h>

static const struct device *const uart_dev = DEVICE_DT_GET(DT_ALIAS(mother_uart));
static const struct device *const uart_debug = DEVICE_DT_GET(DT_ALIAS(debug_uart));
//#define UART_MSG_SIZE (sizeof(uint8_t)*25)
//K_MSGQ_DEFINE(uart_msgq, sizeof(uint8_t), 100, 1);
////struct mother_msg msg;
//char message;
//	//void recv_str(const struct device *uart, char *str)
//	//{
//	//        char *head = str;
//	//        char c;
//	//
//	//        while (!uart_poll_in(uart, &c)) {
//	//                *head++ = c;
//	//        }
//	//        *head = '\0';
//	//
//	//        printk("Device %s received: \"%s\"\n", uart->name, str);
//	//}
//	//
//	//
//void serial_cb(const struct device *dev, void *user_data){
//	uint8_t c;
//	int buf_pos=0;
//	if (!uart_irq_update(uart_dev)) {
//		return;
//	}
//
//	if (!uart_irq_rx_ready(uart_dev)) {
//		return;
//	}
//	while (uart_fifo_read(uart_dev, &c, 1) == 1) {
//        	k_msgq_put(&uart_msgq, &c, K_MSEC(4));
//	//	printk("print c:%hhx\n",c);
//	}
//}
//int main(){
//	int err;
//	printk("I am alive");
//	if (!device_is_ready(uart_dev)) 
//	{
//		printk( "UART device not ready");
//	}
//	err = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
//	if(err<0)
//	{
//		printk("Error setting irq on uart_dev\n");
//	}
//	uart_irq_rx_enable(uart_dev);
//	
//	while(true){
//		k_msgq_get(&uart_msgq, &message,K_MSEC(4));
//		printk("%c ", message);
//		printk("\n");
//		}
//}
static const struct device *const uart_gps = DEVICE_DT_GET(DT_ALIAS(test_uart));
//#define UART_MSG_SIZE (sizeof(uint8_t)*25)                                                                     
#define K_PIPE_DEFINE(ubxpipe,30,0);
struct modem_ubx ubx;
struct modem_ubx_config config;

K_MSGQ_DEFINE(uart_msgq, sizeof(uint8_t), 100, 1);
K_MSGQ_DEFINE(uart_gps_msgq, sizeof(char)*30, 1000, 1);
char gpsmessage;                                                                                                    
void serial_cb(const struct device *dev, void *user_data){                                                       
	char c;                                                                                                     
	int buf_pos=0;                                                                                                 
	if (!uart_irq_update(uart_gps)) {                                                                              
		return;                                                                                                      
	}                                                                                                              
                                                                                                                 
	if (!uart_irq_rx_ready(uart_gps)) {
		return;
	}
	while (uart_fifo_read(uart_gps, &c, 1) == 1) {                                                                 
	  if (gpsmessage == 0xb5)
      k_msgq_get(&uart_gps_msgq, &gpsmessage,K_MSEC(4));
      if( gpsmessage== 0x62)
      for(int i=0 ; i<30 ; i++)
      {
        	k_msgq_put(&uart_gps_msgq, &c, K_MSEC(4));                                                                 
      }
	//	printk("print c:%hhx\n",c);                                                                                
	}                                                                                                              
}                                                                                                                
int main(){                                                                                                      
	int err;                                                                                                       
	printk("I am alive");                                                                                          
	if (!device_is_ready(uart_gps))                                                                                
	{                                                                                                              
		printk( "UART device not ready");                                                                            
	}                                                                                                              
	err = uart_irq_callback_user_data_set(uart_gps, serial_cb, NULL);                                              
	if(err<0)                                                                                                      
	{                                                                                                              
		printk("Error setting irq on uart_dev\n");                                                                   
	}                                                                                                              
	uart_irq_rx_enable(uart_gps);                                                                                  
	                                                                                                               
modem_ubx_init (*ubx, const struct modem_ubx_config *config );
modem_ubx_attach(*ubx, *ubxpipe);
char temp;
	while(true){                                                                                                   
		k_msgq_get(&uart_gps_msgq, &gpsmessage,K_MSEC(4));                                                                  
		//printk("%x ", gpsmessage);
k_pipe_put(*ubxpipe, &gpsmessage, 1, *temp, 1,K_MSEC(4));
  }
}                                                                                                                
