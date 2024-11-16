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

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

/* DT spec for pwm motors */
#define PWM_MOTOR_SETUP(pwm_dev_id)                                            \
  {.dev_spec = PWM_DT_SPEC_GET(pwm_dev_id),                                    \
   .min_pulse = DT_PROP(pwm_dev_id, min_pulse),                                \
   .max_pulse = DT_PROP(pwm_dev_id, max_pulse)},

struct pwm_motor motor[21] = {
    DT_FOREACH_CHILD(DT_PATH(pwmmotors), PWM_MOTOR_SETUP)};

/* DT spec for stepper */
const struct stepper_motor stepper[3] = {
    {.dir = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor1), dir_gpios),
     .step = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor1), step_gpios)},
    {.dir = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor2), dir_gpios),
     .step = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor2), step_gpios)},
    {.dir = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor3), dir_gpios),
     .step = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor3), step_gpios)}};

// creating mssg queue to store data
K_MSGQ_DEFINE(uart_msgq, 25 * sizeof(uint8_t), 10, 1);

static const struct device *const uart_dev =
    DEVICE_DT_GET(DT_ALIAS(mother_uart)); // data from SBUS
static const struct device *const uart_debug =
    DEVICE_DT_GET(DT_ALIAS(debug_uart)); // debugger

float linear_velocity_range[] = {-1.5, 1.5};
float angular_velocity_range[] = {-5.5, 5.5};
float wheel_velocity_range[] = {-10.0, 10.0};
uint32_t pwm_range[] = {1120000, 1880000};
float la_speed_range[] = {-127.0, 127.0};
float angle_range[] = {-270, 270};
uint16_t channel_range[] = {172, 1811};
int pos[2] = {0};
uint16_t *ch;       // to store sbus channels
uint8_t packet[25]; // to store sbus packet

// to get serial data using uart
void serial_cb(const struct device *dev, void *user_data) {
  ARG_UNUSED(user_data);
  uint8_t c, start = 0x0F;

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

int Stepper_motor_write(const struct stepper_motor *motor, uint16_t ch,
                        int pos) {

  if (abs(ch - 992) < 200)
    return pos;

  if (ch > 1004) {
    gpio_pin_set_dt(&(motor->dir), 1);
    pos += 1; // clockwise
  } else {
    gpio_pin_set_dt(&(motor->dir), 0);
    pos -= 1;
  } // anticlockwise
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
  uint16_t cmd[2] = {ch[4], ch[5]};
  // int pos[2];
  for (int i = 0; i < 2; i++) {
    //    printk("stepper\n");
    pos[i] = Stepper_motor_write(&stepper[i], cmd[i], pos[i]);
    // last_time[i] = time[i];
  }
}
K_WORK_DEFINE(my_work, arm_joints);

void my_timer_handler(struct k_timer *dummy) { k_work_submit(&my_work); }
K_TIMER_DEFINE(my_timer, my_timer_handler, NULL);

int sbus_parsing() {
  uint8_t packet[25], packet_pos = 0, start = 0x0f, message = 0;

  k_msgq_get(&uart_msgq, &message, K_MSEC(4));

  if (message == start) {
    for (packet_pos = 0; packet_pos < 25; packet_pos++) {
      packet[packet_pos] = message;
      k_msgq_get(&uart_msgq, &message, K_MSEC(4));
      printk("Message was found");
    }
    ch = parse_buffer(packet);
    return 1;
  } else {
    return 0;
  }
}

void serial_cb(const struct device *dev, void *user_data) {
  uint8_t c;
  printk("Entered Serial Callback");
  if (!uart_irq_update(uart_dev)) {
    return;
  }

  if (!uart_irq_rx_ready(uart_dev)) {
    return;
  }

  while (uart_fifo_read(uart_dev, &c, 1) == 1) {
    k_msgq_put(&uart_msgq, &c, K_NO_WAIT); // put message from UART to queue
  }
}

float one_hot_interpolation(uint16_t channel_input) {

  if (channel_input > channel_range[1]) {
    return pwm_range[1];
  }

  if (channel_input < channel_range[0]) {
    return pwm_range[0];
  }

  if (channel_input < 1005 && channel_input > 995) {
    return (pwm_range[0] + pwm_range[1]) / 2;
  }
  float dchannel = channel_range[1] - channel_range[0];
  float dpwm = pwm_range[1] - pwm_range[0];

  uint32_t pwm_interp =
      pwm_range[0] + (dpwm / dchannel) * (channel_input - channel_range[0]);

  return pwm_interp;
}

float sbus_velocity_interpolation(uint16_t channel_input,
                                  float *velocity_range) {

  if (channel_input > channel_range[1]) {
    return velocity_range[1];
  }

  if (channel_input < channel_range[0]) {
    return velocity_range[0];
  }

  if (channel_input < 1005 && channel_input > 995) {
    return (velocity_range[0] + velocity_range[1]) / 2;
  }
  float dchannel = channel_range[1] - channel_range[0];
  float dvel = velocity_range[1] - velocity_range[0];

  float vel_interp = velocity_range[0] +
                     (dvel / dchannel) * (channel_input - channel_range[0]);
  return vel_interp;
}

>>>>>>> Stashed changes
int feedback_callback(float *feedback_buffer, int buffer_len,
                      int wheels_per_side) {
  return 0;
}

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

int actuator_write(int i, uint16_t ch) {
  if (pwm_motor_write(&(motor[i]),
                      sbus_pwm_interpolation(ch, pwm_range, channel_range))) {
    printk("Linear Actuator: Unable to write at linear actuator %d", i);
    return 1;
  }
}

int main() {
  printk("This is tarzan version %s\nFile: %s\n", GIT_BRANCH_NAME, __FILE__);
  int err, i, flag = 0;
  uint16_t neutral = 992;
  uint64_t drive_timestamp = 0;
  uint64_t time_last_drive_update = 0;

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
  // 	Angular and linear velocity
  struct DiffDriveTwist cmd = {
      .angular_z = 0,
      .linear_x = 0,
  };

  struct DiffDrive *drive =
      diffdrive_init(&drive_config, feedback_callback, velocity_callback);

  if (!device_is_ready(uart_dev)) {
    LOG_ERR("UART device not ready");
  }

  for (size_t i = 0U; i < ARRAY_SIZE(motor); i++) {
    if (!pwm_is_ready_dt(&(motor[i].dev_spec))) {
      LOG_ERR("PWM: Motor %s is not ready", motor[i].dev_spec.dev->name);
    }
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

  uart_irq_rx_enable(uart_dev);

  for (size_t i = 0U; i < ARRAY_SIZE(motor); i++) {
    if (pwm_motor_write(&(motor[i]), 1500000)) {
      printk("Unable to write pwm pulse to PWM Motor : %d", i);
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

  // timer for arm_joints
  k_timer_start(&my_timer, K_USEC(100), K_USEC(10));

  printk("Initialization completed successfully!\n");

  while (true) {

    k_msgq_get(&uart_msgq, &packet, K_MSEC(20));

    ch = parse_buffer(packet);

    drive_timestamp = k_uptime_get();

    for (int i = 0; i < 10; i++) {
      printk("%d \t", ch[i]);
    }
    printk("\n");

    actuator_write(7, ch[7]); // ABox

    if (ch[8] > 1000) {

      actuator_write(8, neutral); // Y of YPR
      actuator_write(9, neutral); // P of YPR

      actuator_write(12, neutral); // Gripper1
      actuator_write(13, neutral); // Gripper2
      actuator_write(14, neutral); // R of YPR
      cmd.angular_z = sbus_velocity_interpolation(ch[0], angular_velocity_range,
                                                  channel_range);
      cmd.linear_x = sbus_velocity_interpolation(ch[1], linear_velocity_range,
                                                 channel_range);

      err = diffdrive_update(drive, cmd, time_last_drive_update);

      actuator_write(2, ch[2]);
      actuator_write(3, ch[3]);
    } else {
      cmd.angular_z = sbus_velocity_interpolation(
          neutral, angular_velocity_range, channel_range);
      cmd.linear_x = sbus_velocity_interpolation(neutral, linear_velocity_range,
                                                 channel_range);

      err = diffdrive_update(drive, cmd, time_last_drive_update);

      actuator_write(2, neutral);
      actuator_write(3, neutral);

      actuator_write(8, ch[0]); // Y of YPR
      actuator_write(9, ch[1]); // P of YPR

      actuator_write(12, ch[2]); // Gripper1
      actuator_write(13, ch[3]); // Gripper2
      actuator_write(14, ch[9]); // R of YPR

      time_last_drive_update = k_uptime_get() - drive_timestamp;
    }
  }
}
