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

LOG_MODULE_REGISTER(stepper_test, CONFIG_LOG_DEFAULT_LEVEL);
static const struct device *const uart_dev =
    DEVICE_DT_GET(DT_ALIAS(mother_uart)); // data from SBUS
static const struct device *const uart_debug =
    DEVICE_DT_GET(DT_ALIAS(debug_uart)); // debugger

// DT spec for stepper
const struct stepper_motor stepper[3] = {
    {.dir = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor1), dir_gpios),
     .step = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor1), step_gpios)},
    {.dir = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor2), dir_gpios),
     .step = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor2), step_gpios)},
    {.dir = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor3), dir_gpios),
     .step = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor3), step_gpios)}};

// creating mssg queue to store data
K_MSGQ_DEFINE(uart_msgq, sizeof(uint8_t), 250, 1);

// ranges used in interpolation;
uint16_t channel_range[] = {0, 950, 2047};

uint16_t *ch;
uint8_t packet[25];
int pos[3] = {0};
uint64_t time, last_time = 0.0;
float stepInterval;
void serial_cb(const struct device *dev, void *user_data) {
  ARG_UNUSED(user_data);
  uint8_t start = 0x0F;
  uint8_t c;

  if (!uart_irq_update(uart_dev))
    return;
  if (!uart_irq_rx_ready(uart_dev))
    return;

  while (uart_fifo_read(uart_dev, &c, 1) == 1) {
    if (c != start)
      continue;
    packet[0] = c;
    if (uart_fifo_read(uart_dev, packet + 1, 24) == 24)
      k_msgq_put(&uart_msgq, packet, K_NO_WAIT);
  }
}

void setSpeed(float speed) {
  if (speed = 0.0)
    stepInterval = 0.0;
  else
    stepInterval = (1000000 / speed);
}

int Stepper_motor_write(const struct stepper_motor *motor, uint16_t cmd,
                        int pos) {

  if (abs(cmd - channel_range[1]) < 200) {
    // gpio_pin_set_dt(&(motor->step),0);
    return pos;
  }

  if (cmd > channel_range[1]) {
    gpio_pin_set_dt(&(motor->dir), 1); // clockwise
    pos += 1;
  } else {
    gpio_pin_set_dt(&(motor->dir), 0); // anti-clockwise
    pos -= 1;
  }
  switch (pos & 0x03) {
  case 0:
    gpio_pin_set_dt(&(motor->step), 0);
    break;
  case 1:
    gpio_pin_set_dt(&(motor->step), 1);
    break;
  case 2:
    gpio_pin_set_dt(&(motor->step), 1);
    break;
  case 3:
    gpio_pin_set_dt(&(motor->step), 0);
    break;
  }
  return pos;
}

void arm_joints(struct k_work *work) {
  uint16_t cmd[2] = {ch[0], ch[1]};
  // printk("%4d %4d\n",cmd[0],cmd[1]);
  // setSpeed(500000.0);
  for (int i = 0; i < 2; i++) {
    //	 time[i] = k_uptime_ticks();
    //	 if ((time[i] - last_time[i]) >= 1) {
    pos[i] = Stepper_motor_write(&stepper[i], cmd[i], pos[i]);
    //		 last_time[i] = time[i];
    //	}
  }
}
K_WORK_DEFINE(my_work, arm_joints);

void my_timer_handler(struct k_timer *dummy) { k_work_submit(&my_work); }
K_TIMER_DEFINE(my_timer, my_timer_handler, NULL);

int main() {

  int err;
  if (!device_is_ready(uart_dev)) {
    LOG_ERR("UART device not ready");
  }

  err = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);

  if (err < 0) {
    if (err == -ENOTSUP) {
      printk("Interrupt-driven UART API support not enabled");
    } else if (err == -ENOSYS) {
      printk("UART device does not support interrupt-driven API");
    } else {
      printk("Error setting UART callback: %d", err);
    }
  }

  if (err < 0) {
    if (err == -ENOTSUP)
      printk("Interrupt-driven UART API support not enabled");

    else if (err == -ENOSYS)
      printk("UART device does not support interrupt-driven API");

    else
      printk("Error setting UART callback: %d", err);
  }

  uart_irq_rx_enable(uart_dev);

  for (size_t i = 0U; i < 3; i++) {
    if (!gpio_is_ready_dt(&stepper[i].dir)) {
      printk("Stepper Motor %d: Dir %d is not ready\n", i, stepper[i].dir.pin);
      return 0;
    }
    if (!gpio_is_ready_dt(&stepper[i].step)) {
      printk("Stepper Motor %d: Dir %d is not ready\n", i, stepper[i].step.pin);
      return 0;
    }
  }

  for (size_t i = 0U; i < 3; i++) {
    if (gpio_pin_configure_dt(&(stepper[i].dir), GPIO_OUTPUT_INACTIVE)) {
      printk("Error: Stepper motor %d: Dir %d not configured", i,
             stepper[i].dir.pin);
      return 0;
    }
    if (gpio_pin_configure_dt(&(stepper[i].step), GPIO_OUTPUT_INACTIVE)) {
      printk("Error: Stepper motor %d: Dir %d not configured", i,
             stepper[i].step.pin);
      return 0;
    }
  }

  LOG_INF("Initialization completed successfully!");

  k_timer_start(&my_timer, K_USEC(60), K_USEC(15));
  printk("Initialization completed successfully!");
  while (true) {
    k_msgq_get(&uart_msgq, &packet, K_MSEC(20));
    ch = parse_buffer(packet);
    for (int i = 0; i < 16; i++)
      printk("%d\t", ch[0]);
    printk("\n");
  }
}
