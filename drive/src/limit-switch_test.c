#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <kyvernitis/lib/kyvernitis.h>

#include <Tarzan/lib/drive.h>
#include <Tarzan/lib/sbus.h>

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

static const struct device *const uart_debug =
    DEVICE_DT_GET(DT_ALIAS(debug_uart)); // debugger

static const struct gpio_dt_spec gripper =
    GPIO_DT_SPEC_GET_BY_IDX(DT_ALIAS(limit_switch), gpios, 1);
int main() {

  //  for (int i = 0; i < 9; i++) {
  if (!gpio_is_ready_dt(&gripper)) {
    LOG_ERR("gpio %d is not ready\n", 1);
    return 0;
  }
  //}
  // for (int i = 0; i < 9; i++) {
  if (!gpio_pin_configure_dt(&gripper, GPIO_INPUT)) {
    LOG_ERR("Error: gpio %d is not configured\n", 1);
    return 0;
  }
  //}
  LOG_INF("Initialization completed successfully!\n");

  while (true) {
    if (gpio_port_get(&gripper, 0) == 0)
      LOG_INF("limit switch hit");
  }
}
