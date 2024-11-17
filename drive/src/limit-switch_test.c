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

static const struct gpio_dt_spec gripper =
    GPIO_DT_SPEC_GET(DT_ALIAS(limit_switch), gpios);
/*int main() {

  //  for (int i = 0; i < 9; i++) {
  if (!gpio_is_ready_dt(&gripper)) {
    printk("gpio %d is not ready\n", 1);
    return 0;
  }
  //}
  // for (int i = 0; i < 9; i++) {
  if (gpio_pin_configure_dt(&gripper, GPIO_INPUT)) {
    printk("Error: gpio %d is not configured\n", 1);
    return 0;
  }
  //}
  printk("Initialization completed successfully!\n");

  while (true) {
    if (gpio_pin_get_dt(&gripper) == 0)
      printk("limit switch hit\n");
  }
}*/


int main(void)
{
	int err;
  printk("This is branch: %s\n",GIT_BRANCH_NAME);
	if (!gpio_is_ready_dt(&gripper)) {
		printf("The load switch pin GPIO port is not ready.\n");
		return 0;
	}
	err = gpio_pin_configure_dt(&gripper, GPIO_INPUT);
	if (err != 0) {
		printf("Configuring GPIO pin failed: %d\n", err);
		return 0;
	}
  printk("Initialization completed\n");
  return 0;

}
