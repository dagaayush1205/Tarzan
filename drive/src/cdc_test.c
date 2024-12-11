#include <canscribe/lib/canscribe.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>

// const struct device *const dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
const struct device *const dev = DEVICE_DT_GET(DT_ALIAS(latte_panda));

int main() {
  uint32_t dtr = 0;
  struct msg {
    int x;
    int y;
    int z;
    int a;
    int b;
    int c;
    int d;
    int cc;
  } data = {0, 1, 2, 4, 5, 6, 7};
  uint8_t buf[sizeof(data) + 2];
  if (usb_enable(NULL)) {
    return 0;
  }

  while (!dtr) {
    uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
    k_sleep(K_MSEC(100));
  }
  while (true) {
    serialize(buf, (uint8_t *)&data, sizeof(struct msg));
    for (int i = 0; i < sizeof(struct msg); i++)
      uart_poll_out(dev, buf[i]);
    k_sleep(K_SECONDS(1));
  }
}
