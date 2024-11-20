#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>

const struct device *const dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

int main() {
  uint32_t dtr = 0;
  if (usb_enable(NULL)) {
    return 0;
  }

  while (!dtr) {
    uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
    k_sleep(K_MSEC(100));
  }
  printk("Nucleo says Hello");
}
