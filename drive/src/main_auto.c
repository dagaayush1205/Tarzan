#include "zephyr/toolchain.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/thread_stack.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/usb_device.h>

#include <kyvernitis/lib/kyvernitis.h>

#include <Tarzan/lib/arm.h>
#include <Tarzan/lib/cobs.h>
#include <Tarzan/lib/drive.h>
#include <Tarzan/lib/sbus.h>

/* latte panda uart */
static const struct device *const latte_panda_uart =
    DEVICE_DT_GET(DT_ALIAS(latte_panda_uart));
/* gps uart */
static const struct device *const gps_uart = DEVICE_DT_GET(DT_ALIAS(gps_uart));
/* DT spec for pwm motors */
#define PWM_MOTOR_SETUP(pwm_dev_id)                                            \
  {.dev_spec = PWM_DT_SPEC_GET(pwm_dev_id),                                    \
   .min_pulse = DT_PROP(pwm_dev_id, min_pulse),                                \
   .max_pulse = DT_PROP(pwm_dev_id, max_pulse)},
struct pwm_motor motor[10] = {
    DT_FOREACH_CHILD(DT_PATH(pwmmotors), PWM_MOTOR_SETUP)};

/* DT spec for leds */
static const struct gpio_dt_spec init_led =
    GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

static const struct gpio_dt_spec auto_led =
    GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
/* common msg struct for coms */
struct cmd_msg {
  struct DiffDriveTwist auto_cmd;
  uint32_t crc;
};
/* msg struct for com with base station */
struct base_station_msg {
  char gps_msg[100];
  uint16_t msg_status;
  uint32_t crc;
};

/* decleration of check_crc func*/
int check_crc(struct cmd_msg *);

/* defining cobs message queue */
K_MSGQ_DEFINE(msgq_rx, sizeof(struct cmd_msg), 100, 1);
/* defining gps message queue */
K_MSGQ_DEFINE(gps_msgq, sizeof(uint8_t) * 100, 10, 1);

/* struct for drive variables */
struct drive_arg {
  struct DiffDriveConfig drive_config;
  struct DiffDriveTwist cmd;
  struct DiffDrive *drive_init;
  uint64_t time_last_drive_update;
} drive;
/* struct for communication with latte panda*/
struct com_arg {
  struct cmd_msg msg_rx; // to store decoded mssg
  struct base_station_msg bs_msg_tx;
} com;

uint8_t gps_mssg[100];             // to store gps mssg
int gps_bytes_read;                // to store number of gps bytes read
uint16_t error_mssg_flag = 0x0000; // to store error status byte
const int AUTO_MSG_LEN = sizeof(struct cmd_msg) + 2; // len of inverse mssg
const int BS_MSG_LEN =
    sizeof(struct base_station_msg) + 2; // len of base station mssg
uint8_t rx_buf[sizeof(struct cmd_msg) + 2] = {0};
uint8_t bs_tx_buf[sizeof(struct base_station_msg) + 2] = {0};
int cobs_bytes_read; // to store number of cobs bytes read
/* range variables */
float linear_velocity_range[] = {-1.5, 1.5};
float angular_velocity_range[] = {-5.5, 5.5};
float wheel_velocity_range[] = {-10.0, 10.0};
uint32_t pwm_range[] = {1100000, 1900000};

/* interrupt to store gps data */
void gps_cb(const struct device *dev, void *user_data) {
  ARG_UNUSED(user_data);
  char c;
  if (!uart_irq_update(gps_uart)) {
    return;
  }

  if (!uart_irq_rx_ready(gps_uart)) {
    return;
  }
  while (gps_bytes_read < 100 && uart_fifo_read(gps_uart, &c, 1)) {
    if (gps_bytes_read == 0 && c != 0xb5)
      continue;
    if (gps_bytes_read == 1 && c != 0x62)
      continue;
    if (gps_bytes_read == 2 && c != 0x01)
      continue;
    if (gps_bytes_read == 3 && c != 0x07)
      continue;
    gps_mssg[gps_bytes_read++] = c;
  }
  if (gps_bytes_read == 100) {
    k_msgq_put(&gps_msgq, gps_mssg, K_NO_WAIT);
    gps_bytes_read = 0;
  }
}
void cobs_cb(const struct device *dev, void *user_data) {
  ARG_UNUSED(user_data);
  uint8_t c;
  struct cmd_msg rx_msg;
  if (!uart_irq_update(dev)) {
    return;
  }
  if (!uart_irq_rx_ready(dev)) {
    return;
  }
  while (uart_fifo_read(dev, &c, 1) == 1) {
    if (c == 0x00 && cobs_bytes_read > 0) {
      rx_buf[cobs_bytes_read++] = 0;
      if (cobs_bytes_read != AUTO_MSG_LEN) {
        cobs_bytes_read = 0;
        continue;
      }
      cobs_decode((void *)&(rx_msg), sizeof(rx_msg), rx_buf, AUTO_MSG_LEN - 1);
      k_msgq_put(&msgq_rx, &rx_msg, K_NO_WAIT);
      cobs_bytes_read = 0;
    } else if (cobs_bytes_read < sizeof(rx_buf)) {
      rx_buf[cobs_bytes_read++] = c;
    }
  }
}
/* work handler to send data to latte panda */
void latte_panda_tx_work_handler() {
  k_msgq_get(&gps_msgq, com.bs_msg_tx.gps_msg, K_MSEC(4));
  com.bs_msg_tx.msg_status = error_mssg_flag;
  com.bs_msg_tx.crc =
      crc32_ieee((uint8_t *)(&com.bs_msg_tx),
                 sizeof(struct base_station_msg) - sizeof(uint32_t));
  cobs_encode_result result =
      cobs_encode(bs_tx_buf, BS_MSG_LEN, (void *)&com.bs_msg_tx,
                  sizeof(struct base_station_msg));

  if (result.status != COBS_ENCODE_OK) {
    // printk("COBS Encoded Failed %d\n", result.status);
    return;
  }
  bs_tx_buf[BS_MSG_LEN - 1] = 0x00;
  for (int i = 0; i < BS_MSG_LEN; i++) {
    uart_poll_out(latte_panda_uart, bs_tx_buf[i]);
  }
  error_mssg_flag = 0x0000;
}
/* check if received cobs message is valid,
 * ret 0 if successfull */
int check_crc(struct cmd_msg *msg) {
  uint32_t valid_crc;
  valid_crc =
      crc32_ieee((uint8_t *)msg, sizeof(struct cmd_msg) - sizeof(msg->crc));
  if (valid_crc != msg->crc) {
    return 1;
  }
  return 0;
}

int velocity_callback(const float *velocity_buffer, int buffer_len,
                      int wheels_per_side) {
  if (buffer_len < wheels_per_side * 2) {
    return 1;
  }
  if (pwm_motor_write(&(motor[0]), velocity_pwm_interpolation(
                                       *(velocity_buffer), wheel_velocity_range,
                                       pwm_range))) {
    error_mssg_flag = error_mssg_flag | 0x0010;
    // printk("Drive: Unable to write pwm pulse to Left");
    return 1;
  }
  if (pwm_motor_write(&(motor[1]), velocity_pwm_interpolation(
                                       *(velocity_buffer + wheels_per_side + 1),
                                       wheel_velocity_range, pwm_range))) {
    error_mssg_flag = error_mssg_flag | 0x0020;
    // printk("Drive: Unable to write pwm pulse to Right");
    return 1;
  }
  return 0;
}

int feedback_callback(float *feedback_buffer, int buffer_len,
                      int wheels_per_side) {
  return 0;
}

void mssg_timer_handler(struct k_timer *mssg_timer_ptr) {
  latte_panda_tx_work_handler();
}
K_TIMER_DEFINE(mssg_timer, mssg_timer_handler, NULL);

int main() {

  printk("Tarzan version %s\nFile: %s\n", TARZAN_GIT_VERSION, __FILE__);

  int err;

  /* initializing drive configs */
  const struct DiffDriveConfig tmp_drive_config = {
      .wheel_separation = 0.77f,
      .wheel_separation_multiplier = 1,
      .wheel_radius = 0.15f,
      .wheels_per_side = 2,
      .command_timeout_seconds = 2,
      .left_wheel_radius_multiplier = 1,
      .right_wheel_radius_multiplier = 1,
      .update_type = POSITION_FEEDBACK,
  };
  drive.drive_config = tmp_drive_config;
  drive.drive_init = diffdrive_init(&(drive.drive_config), feedback_callback,
                                    velocity_callback);
  /* gps uart ready check */
  if (!device_is_ready(gps_uart)) {
    printk("GPS UART device not ready");
  }
  /* latte panda uart ready check */
  if (!device_is_ready(latte_panda_uart)) {
    printk("LATTE PANDA UART device not ready");
  }
  if (usb_enable(NULL)) {
    return 0;
  }
  /* set gps uart for interrupt */
  err = uart_irq_callback_user_data_set(gps_uart, gps_cb, NULL);
  if (err < 0) {
    if (err == -ENOTSUP) {
      printk("Interrupt-driven UART API support not enabled");
    } else if (err == -ENOSYS) {
      printk("UART device does not support interrupt-driven API");
    } else {
      printk("Error setting UART callback: %d", err);
    }
  }
  /* set latte panda uart for interrupt */
  err = uart_irq_callback_user_data_set(latte_panda_uart, cobs_cb, NULL);
  if (err < 0) {
    if (err == -ENOTSUP) {
      printk("Interrupt-driven UART API support not enabled");
    } else if (err == -ENOSYS) {
      printk("UART device does not support interrupt-driven API");
    } else {
      printk("Error setting UART callback: %d", err);
    }
  }
  /* pwm ready check */
  for (size_t i = 0U; i < ARRAY_SIZE(motor); i++) {
    if (!pwm_is_ready_dt(&(motor[i].dev_spec))) {
      printk("PWM: Motor %s is not ready", motor[i].dev_spec.dev->name);
    }
  }
  for (size_t i = 0U; i < ARRAY_SIZE(motor); i++) {
    if (pwm_motor_write(&(motor[i]), 1500000)) {
      printk("Unable to write pwm pulse to PWM Motor : %d\n", i);
    }
  }
  /* led ready checks */
  if (!gpio_is_ready_dt(&init_led)) {
    printk("Initialization led not ready\n");
  }
  if (gpio_pin_configure_dt(&init_led, GPIO_OUTPUT_INACTIVE) < 0) {
    printk("Intitialization led not configured\n");
  }
  if (!gpio_is_ready_dt(&auto_led)) {
    printk("Initialization led not ready\n");
  }
  if (gpio_pin_configure_dt(&auto_led, GPIO_OUTPUT_INACTIVE) < 0) {
    printk("Intitialization led not configured\n");
  }
  printk("Initialization completed successfully!\n");
  gpio_pin_set_dt(&init_led, 1); // set initialization led high

  /* enable interrupt to receive gps data */
  uart_irq_rx_enable(gps_uart);
  /* enable interrupt to receive cobs data */
  uart_irq_rx_enable(latte_panda_uart);

  k_timer_start(&mssg_timer, K_MSEC(10), K_SECONDS(1));
  while (true) {
    k_msgq_get(&msgq_rx, &com.msg_rx, K_MSEC(4));
    // if (check_crc(&com.msg_rx) != 0)
    //   continue;
    drive.cmd.linear_x = com.msg_rx.auto_cmd.linear_x;
    drive.cmd.angular_z = com.msg_rx.auto_cmd.angular_z;
    uint64_t drive_timestamp = k_uptime_get();
    diffdrive_update(drive.drive_init, drive.cmd, drive.time_last_drive_update);
    drive.time_last_drive_update = k_uptime_get() - drive_timestamp;
    k_sleep(K_USEC(100));
  }
}
