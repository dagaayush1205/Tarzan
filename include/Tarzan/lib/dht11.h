#ifndef DHT11_SENSOR_H
#define DHT11_SENSOR_H
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>

int read_sensor_values(struct gpio_dt_spec device, int arr[5]);
int32_t read_adc_mv(const struct device *adc_dev,
                           const struct adc_channel_cfg *cfg, uint16_t vref_mv,
                           uint8_t resolution);

#endif
