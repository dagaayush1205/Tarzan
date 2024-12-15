#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <Tarzan/lib/arm.h>

const struct device *const base = DEVICE_DT_GET(DT_ALIAS(mm_turn_table));
const struct device *const rover = DEVICE_DT_GET(DT_ALIAS(mm_rover));
void process_bmm150(const struct device *dev) {
  struct sensor_value mag[3];
  int rc = sensor_sample_fetch(dev);

  if (rc == 0)
    rc = sensor_channel_get(dev, SENSOR_CHAN_MAGN_XYZ, mag);

  printf("%f %f %f\t", sensor_value_to_double(&mag[0]),
         sensor_value_to_double(&mag[1]), sensor_value_to_double(&mag[2]));
}
int main() {
  /*Device checks*/
  if (!device_is_ready(base)) {
    printk("Device %s is not ready\n", base->name);
    return 0;
  }
  printk("ready1\n");
  if (!device_is_ready(rover)) {
    printk("Device %s is not ready\n", base->name);
    return 0;
  }
  printk("ready2\n");

  printk("Initialization completed successfully!\n");

  while (true) {
    process_bmm150(base);
    process_bmm150(rover);
    printk("\n");
    k_sleep(K_MSEC(20));
  }
}
