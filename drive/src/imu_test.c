#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

const struct device *const lower = DEVICE_DT_GET(DT_ALIAS(imu_lower_joint));
const struct device *const upper = DEVICE_DT_GET(DT_ALIAS(imu_upper_joint));
const struct device *const base = DEVICE_DT_GET(DT_ALIAS(imu_turn_table));
const struct device *const end = DEVICE_DT_GET(DT_ALIAS(imu_pr));
/* Causing buffer overflow
static const char *now_str(void) {
  static char buf[16]; // ....HH:MM:SS.MMM
  uint32_t now = k_uptime_get_32();
  unsigned int ms = now % MSEC_PER_SEC;
  unsigned int s;
  unsigned int min;
  unsigned int h;

  now /= MSEC_PER_SEC;
  s = now % 60U;
  now /= 60U;
  min = now % 60U;
  now /= 60U;
  h = now;

  sprintf(buf, sizeof(buf), "%u:%02u:%02u.%03u", h, min, s, ms);
  return buf;
}*/

static int process_mpu6050(const struct device *dev, int n) {
  struct sensor_value accel[3];
  struct sensor_value gyro[3];

  int rc = sensor_sample_fetch(dev);

  if (rc == 0)
    rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
  if (rc == 0)
    rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);

  if (rc == 0) {
    printk("%d accel %f %f %f m/s/s\t"
           " gyro %f %f %f rad/s\n",
           n, sensor_value_to_double(&accel[0]),
           sensor_value_to_double(&accel[1]), sensor_value_to_double(&accel[2]),
           sensor_value_to_double(&gyro[0]), sensor_value_to_double(&gyro[1]),
           sensor_value_to_double(&gyro[2]));

  } else
    printk("sample fetch/get failed: %d\n", rc);

  return rc;
}

int main() {

  if (!device_is_ready(lower)) {
    printk("Device %s is not ready\n", lower->name);
    return 0;
  }

  if (!device_is_ready(upper)) {
    printk("Device %s is not ready\n", upper->name);
    return 0;
  }

  if (!device_is_ready(base)) {
    printk("Device %s is not ready\n", lower->name);
    return 0;
  }

  if (!device_is_ready(end)) {
    printk("Device %s is not ready\n", upper->name);
    return 0;
  }

  printk("Initialization completed successfully!");

  while (true) {
    process_mpu6050(lower, 1);
    process_mpu6050(upper, 2);
    process_mpu6050(base, 3);
    process_mpu6050(end, 4);
    k_sleep(K_MSEC(20));
  }
}
