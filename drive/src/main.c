#include "zephyr/kernel/thread_stack.h"
#include "zephyr/sys/printk.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
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

#define STACK_SIZE 512
#define PRIORITY 5

/* workq dedicated thread */
K_THREAD_STACK_DEFINE(stack_area, STACK_SIZE);

/* msgq to store sbus data */
K_MSGQ_DEFINE(uart_msgq, 25 * sizeof(uint8_t), 10, 1);

/* msgq poll event */
struct k_poll_event msgq_poll = K_POLL_EVENT_STATIC_INITIALIZER(
    K_POLL_TYPE_MSGQ_DATA_AVAILABLE, K_POLL_MODE_NOTIFY_ONLY, &uart_msgq, 0);

/* sbus uart */
static const struct device *const uart_dev =
    DEVICE_DT_GET(DT_ALIAS(mother_uart));

/* DT spec for pwm motors */
#define PWM_MOTOR_SETUP(pwm_dev_id)                                            \
  {.dev_spec = PWM_DT_SPEC_GET(pwm_dev_id),                                    \
   .min_pulse = DT_PROP(pwm_dev_id, min_pulse),                                \
   .max_pulse = DT_PROP(pwm_dev_id, max_pulse)},

struct pwm_motor motor[10] = {
    DT_FOREACH_CHILD(DT_PATH(pwmmotors), PWM_MOTOR_SETUP)};

/* DT spec for stepper */
const struct stepper_motor stepper[3] = {
    {.dir = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor1), dir_gpios),
     .step = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor1), step_gpios)},
    {.dir = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor2), dir_gpios),
     .step = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor2), step_gpios)},
    {.dir = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor3), dir_gpios),
     .step = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor3), step_gpios)}};

/* diffrential drive configs */
struct DiffDriveConfig drive_config = {
    .wheel_separation = 0.77f,
    .wheel_separation_multiplier = 1,
    .wheel_radius = 0.15f,
    .wheels_per_side = 2,
    .command_timeout_seconds = 2,
    .left_wheel_radius_multiplier = 1,
    .right_wheel_radius_multiplier = 1,
    .update_type = POSITION_FEEDBACK,
};

/* angular and linear velocity */
struct DiffDriveTwist cmd = {
    .angular_z = 0,
    .linear_x = 0,
};

struct DiffDrive *drive =
    diffdrive_init(&drive_config, feedback_callback, velocity_callback);

uint64_t time_last_drive_update = 0;
uint64_t last_time = 0;
float linear_velocity_range[] = {-1.5, 1.5};
float angular_velocity_range[] = {-5.5, 5.5};
float wheel_velocity_range[] = {-10.0, 10.0};
uint32_t pwm_range[] = {1120000, 1880000};
uint16_t channel_range[] = {172, 1811};
int pos[2] = {0};
uint16_t *ch;             // to store sbus channels
uint8_t packet[25] = {0}; // to store sbus packet

/* interrupt to fill sbus data in queue */
void serial_cb(const struct device *dev, void *user_data) {
  ARG_UNUSED(user_data);
  uint8_t c, start = 0x0F;

  if (!uart_irq_update(uart_dev))
    return;
  if (!uart_irq_rx_ready(uart_dev))
    return;

  while (uart_fifo_read(uart_dev, &c, 1) == 1) {
    k_msgq_put(&uart_msgq, &c, K_NO_WAIT);
  }
}
/* work handler to form sbus packet and return sbus channels */
void sbus_parsing(struct k_work *sbus) {
  uint8_t start = 0x0f, packet[25] = {0}, message;
  k_msgq_get(&uart_msgq, &message, K_MSEC(4));
  if (message == start) {
    packet[0] = message;
    for (int i = 1; i < 26; i++) {
      k_msgq_get(&uart_msgq, &message, K_MSEC(4));
      packet[i] == message;
    }
    // parse sbus packet
    parse_buffer(packet, channel);
  }
}
K_WORK_DEFINE(parse_sbus, sbus);

/* work handler to write to motors */
void drive_write(struct k_work *actuator) {
  // drive motor write
  int drive_timestamp = k_uptime_get();
  cmd.angular_z =
      sbus_velocity_interpolation(ch[0], angular_velocity_range, channel_range);
  cmd.linear_x =
      sbus_velocity_interpolation(ch[1], linear_velocity_range, channel_range);
  diffdrive_update(drive, cmd, time_last_drive_update);
  time_last_drive_update = k_uptime_get() - drive_timestamp;
  // linear actuator write
  if (pwm_motor_write(&(motor[2]),
                      sbus_pwm_interpolation(ch, pwm_range, channel_range)))
    printk("Linear Actuator: Unable to write at linear actuator");
  if (pwm_motor_write(&(motor[3]),
                      sbus_pwm_interpolation(ch, pwm_range, channel_range)))
    printk("Linear Actuator: Unable to write at linear actuator");
  // gripper motor write
  if (pwm_motor_write(&(motor[4]),
                      sbus_pwm_interpolation(ch, pwm_range, channel_range)))
    printk("Gripper: Unable to write at gripper");
}
K_WORK_DEFINE(actuator_write, actuator);

/* work handler to write to arm stepper */
void Stepper_motor_write(struct k_work *stepper) {
  uint64_t step_interval = 50;
  uint16_t cmd[2] = {ch[4], ch[5]};
  uint64_t current_time = k_uptime_ticks();
  if (current_time - last_time >= 50) {
    for (int i = 0; i < 2; i++) {
      if (abs(cmd[i] - 992) < 200)
        continue; // deadzone
      if (cmd[i] > 1004) {
        gpio_pin_set_dt(&(stepper[i]->dir), 1);
        pos += 1; // clockwise
      } else {
        gpio_pin_set_dt(&(stepper[i]->dir), 0);
        pos -= 1; // anticlockwise
      }
      switch (pos & 0x03) {
      case 0:
        gpio_pin_set_dt(&(stepper[i]->step), 0);
        break;
      case 1:
        gpio_pin_set_dt(&(stepper[i]->step), 1);
        break;
      case 2:
        gpio_pin_set_dt(&(stepper[i]->step), 1);
        break;
      case 3:
        gpio_pin_set_dt(&(stepper[i]->step), 0);
        break;
      }
    }
  }
}
K_WORK_DEFINE(stepper_write, stepper);

int velocity_callback(const float *velocity_buffer, int buffer_len,
                      int wheels_per_side) {
  if (buffer_len < wheels_per_side * 2) {
    return 1;
  }

  const int i = 0;
  if (pwm_motor_write(&(motor[i]), velocity_pwm_interpolation(
                                       *(velocity_buffer + i),
                                       wheel_velocity_range, pwm_range))) {
    printk("Drive: Unable to write pwm pulse to Left : %d", i);
    return 1;
  }
  if (pwm_motor_write(
          &(motor[i + 1]),
          velocity_pwm_interpolation(*(velocity_buffer + wheels_per_side + i),
                                     wheel_velocity_range, pwm_range))) {
    printk("Drive: Unable to write pwm pulse to Right : %d", i);
    return 1;
  }
  return 0;
}

int feedback_callback(float *feedback_buffer, int buffer_len,
                      int wheels_per_side) {
  return 0;
}
int main() {

  printk("This is tarzan version %s\nFile: %s\n", GIT_BRANCH_NAME, __FILE__);

  int err, rc;
  uint64_t drive_timestamp = 0;
  uint64_t time_last_drive_update = 0;

  /* initializing work queue */
  struct k_work_q work_q;
  k_work_queue_init(&work_q);

  if (!device_is_ready(uart_dev)) {
    printk("UART device not ready\n");
  }

  err = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);

  if (err < 0) {
    if (err == -ENOTSUP) {
      printk("Interrupt-driven UART API support not enabled\n");
    } else if (err == -ENOSYS) {
      printk("UART device does not support interrupt-driven API\n");
    } else {
      printk("Error setting UART callback: %d\n", err);
    }
  }

  for (size_t i = 0U; i < ARRAY_SIZE(motor); i++) {
    if (!pwm_is_ready_dt(&(motor[i].dev_spec))) {
      printk("PWM: Motor %s is not ready\n", motor[i].dev_spec.dev->name);
    }
  }

  for (size_t i = 0U; i < ARRAY_SIZE(motor); i++) {
    if (pwm_motor_write(&(motor[i]), 1500000)) {
      printk("Unable to write pwm pulse to PWM Motor : %d\n", i);
    }
  }

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
      printk("Error: Stepper motor %d: Dir %d not configured\n", i,
             stepper[i].dir.pin);
      return 0;
    }
    if (gpio_pin_configure_dt(&(stepper[i].step), GPIO_OUTPUT_INACTIVE)) {
      printk("Error: Stepper motor %d: Dir %d not configured\n", i,
             stepper[i].step.pin);
      return 0;
    }
  }

  printk("Initialization completed successfully!\n");

  /* enable interrupt to receive sbus data */
  uart_irq_rx_enable(uart_dev);

  /* start running work queue */
  k_work_queue_start(work_q, stack_area, K_THREAD_STACK_SIZEOF(stack_area),
                     PRIORITY, NULL);

  while (true) {
    rc = k_poll(msgq_poll, 1, K_NO_WAIT);
    if (rc == 0) {
      k_work_submit_to_queue(work_q, &parse_sbus);
      k_work_submit_to_queue(work_q, &actuator_write);
      k_work_submit_to_queue(work_q, &stepper_write);
    }
  }
}
