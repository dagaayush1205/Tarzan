#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <Tarzan/lib/arm.h>
// const struct device *const base = DEVICE_DT_GET(DT_ALIAS(mm_turn_table));
const struct device *const rover = DEVICE_DT_GET(DT_ALIAS(mm_rover));
float getCompassDegree(struct sensor_value x , struct sensor_value y)
{
  float compass = 0.0;
  // compass = atan2((float)x.val1 + x.val2/1000000.0, (float)y.val1 + y.val2/1000000.0);
  compass = atan2(sensor_value_to_float(&x), sensor_value_to_float(&y))-M_PI/2;
  if (compass < 0) {
    compass += 2 * M_PI;
  }
  if (compass > 2 * M_PI) {
     compass -= 2 * M_PI;
  }
  return compass * 180 / M_PI;
}
void process_bmm150(const struct device *dev) {
  struct sensor_value mag[3];
  int rc = sensor_sample_fetch(dev);

  if (rc == 0)
    rc = sensor_channel_get(dev, SENSOR_CHAN_MAGN_XYZ, mag);

  getCompassDegree(mag[0],mag[1]);
  printf("%f",getCompassDegree(mag[0],mag[1]));
}
int main() {
  /*Device checks*/
  // if (!device_is_ready(base)) {
  //   printk("Device %s is not ready\n", base->name);
  // }
  // else printk("device ready");

  if (!device_is_ready(rover)) {
    printk("Device %s is not ready\n", rover->name);
    return 0;
  }
  else printk("device ready");

  printk("Initialization completed successfully!\n");

  while (true) {
    // process_bmm150(base);
    process_bmm150(rover);
    printk("\n");
    k_sleep(K_MSEC(20));
  }
}
