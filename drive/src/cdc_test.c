#include <canscribe/lib/canscribe.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/usb/usb_device.h>

const struct device *const dev = DEVICE_DT_GET(DT_ALIAS(latte_panda));
K_MSGQ_DEFINE(msgq, sizeof(uint8_t), 74, 1);
struct msg {
  double x;
  double y;
  double z;
  double a;
  double b;
  double c;
  double d;
  double e;
  double f;
} data = {1, 2, 3, 4, 5, 6, 7, 8, 9}, mssg;
int pos;

uint8_t rx_buf[sizeof(struct msg) + 2];
uint8_t tx_buf[sizeof(struct msg) + 2];
void serial_cb(const struct device *uart_dev, void *user_data) {
  ARG_UNUSED(user_data);
  uint8_t c;
  if (!uart_irq_update(uart_dev)) {
    return;
  }
  if (!uart_irq_rx_ready(uart_dev)) {
    return;
  }
  while (uart_fifo_read(uart_dev, &c, 1) == 1) {
    if (c == 0x00 && pos > 0) {
      rx_buf[pos] = 0;
      if (pos != (sizeof(struct msg) + 2 - 1)) {
        pos = 0;
        continue;
      }
      deserialize(rx_buf, (uint8_t *)&mssg, sizeof(mssg));
      printk("received\n");
      pos = 0;
    } else if (pos < sizeof(rx_buf)) {
      rx_buf[pos++] = c;
    }
    if (pos > sizeof(struct msg) + 2) {
      pos = 0;
      continue;
    }
  }
}
int main() {
  uint32_t dtr = 0;

  if (!device_is_ready(dev)) {
    printk("Uart device not ready");
  }
  int err = uart_irq_callback_user_data_set(dev, serial_cb, NULL);
  if (err < 0) {
    if (err == -ENOTSUP)
      printk("Interrupt-driver UART API support not enabled");
    else if (err == -ENOSYS)
      printk("UART device does not support interrupt-driven API");
    else
      printk("Error setting UART callback");
  }
  if (usb_enable(NULL)) {
    return 0;
  }
  while (!dtr) {
    uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
    printk("error\n");
    k_sleep(K_MSEC(100));
  }
  uart_irq_rx_enable(dev);
  while (true) {
    serialize(tx_buf, (uint8_t *)&data, sizeof(struct msg));
    tx_buf[sizeof(struct msg) + 1] = 0x00;
    for (int i = 0; i < sizeof(struct msg) + 2; i++) {
      uart_poll_out(dev, tx_buf[i]);
    }
  }
}
