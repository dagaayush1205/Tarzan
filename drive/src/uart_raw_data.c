#include <kyvernitis/lib/kyvernitis.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

static const struct device *const uart_dev =
    DEVICE_DT_GET(DT_ALIAS(mother_uart));
static const struct device *const uart_debug =
    DEVICE_DT_GET(DT_ALIAS(debug_uart));
static const struct device *const uart_gps = DEVICE_DT_GET(DT_ALIAS(gps_uart));

K_MSGQ_DEFINE(uart_msgq, sizeof(uint8_t), 100, 1);
K_MSGQ_DEFINE(uart_gps_msgq, sizeof(char) * 30, 1000, 1);
char gpsmessage;
void serial_cb(const struct device *dev, void *user_data) {
  char c;
  int buf_pos = 0;
  if (!uart_irq_update(uart_gps)) {
    return;
  }

  if (!uart_irq_rx_ready(uart_gps)) {
    return;
  }
  while (uart_fifo_read(uart_gps, &c, 1) == 1) {
    if (gpsmessage == 0xb5)
      k_msgq_get(&uart_gps_msgq, &gpsmessage, K_MSEC(4));
    if (gpsmessage == 0x62)
      for (int i = 0; i < 30; i++) {
        k_msgq_put(&uart_gps_msgq, &c, K_MSEC(4));
      }
    printk("%hhx ", c);
  }
}
int main() {
  int err;
  printk("I am alive");
  if (!device_is_ready(uart_gps)) {
    printk("UART device not ready");
  }
  err = uart_irq_callback_user_data_set(uart_gps, serial_cb, NULL);
  if (err < 0) {
    printk("Error setting irq on uart_dev\n");
  }
  uart_irq_rx_enable(uart_gps);

  char temp;
  while (true) {
    k_msgq_get(&uart_gps_msgq, &gpsmessage, K_MSEC(4));
    // printk("%x ", gpsmessage);
  }
}
