#include "zephyr/sys/printk.h"
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

#include <stdint.h>

#include <kyvernitis/lib/kyvernitis.h>

#include <Tarzan/lib/drive.h>
#include <Tarzan/lib/sbus.h>

#define STACK_SIZE 512
#define PRIORITY 2

/* workq dedicated thread */
K_THREAD_STACK_DEFINE(stack_area, STACK_SIZE);
struct k_work_q work_q;
static const struct device *const uart_dev =
    DEVICE_DT_GET(DT_ALIAS(mother_uart)); // data from SBUS

// creating mssg queue to store data
K_MSGQ_DEFINE(uart_msgq, 25 * sizeof(uint8_t), 10, 1);

uint16_t channel[16], packet[25];
int read;

void sbus_parsing(struct k_work *sbus_work) {
  uint8_t packet[25] = {0};
  k_msgq_get(&uart_msgq, &packet, K_MSEC(4));
  parse_buffer(packet, channel);
  for (int i = 0; i < 10; i++)
    printk("%u\t", channel[i]);
  printk("\n");
}
K_WORK_DEFINE(work, sbus_parsing);
void serial_cb(const struct device *dev, void *user_data) {
  ARG_UNUSED(user_data);
  uint8_t c, packet[25];
  int value;
  if (!uart_irq_update(uart_dev))
    return;
  if (!uart_irq_rx_ready(uart_dev))
    return;
  while (read < 25 && uart_fifo_read(uart_dev, &c, 1)) {
    if (read == 0 && c != 0x0f)
      continue;
    packet[read++] = c;
  }
  if (read == 25) {
    k_msgq_put(&uart_msgq, &packet, K_NO_WAIT);
    k_work_submit(&work);
    read = 0;
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

  if (err < 0) {
    if (err == -ENOTSUP)
      printk("Interrupt-driven UART API support not enabled");

    else if (err == -ENOSYS)
      printk("UART device does not support interrupt-driven API");

    else
      printk("Error setting UART callback: %d", err);
  }

  // enable uart device for communication
  uart_irq_rx_enable(uart_dev);

  printk("Initialization completed successfully!");
}
