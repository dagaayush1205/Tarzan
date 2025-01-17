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

#define STACK_SIZE 4096   // work_q thread stack size
#define PRIORITY 2        // work_q thread priority
#define STEPPER_TIMER 100 // stepper pulse width in microseconds

/* sbus uart */
static const struct device *const sbus_uart =
    DEVICE_DT_GET(DT_ALIAS(sbus_uart));
/* telemetry uart */
static const struct device *const telemetry_uart =
    DEVICE_DT_GET(DT_ALIAS(telemetry_uart));
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
  struct inverse_msg inv;
  enum msg_type type;
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
K_MSGQ_DEFINE(msgq_rx, sizeof(struct cmd_msg) + 2, 50, 1);
/* defining gps message queue */
K_MSGQ_DEFINE(gps_msgq, sizeof(uint8_t) * 100, 10, 1);

/* workq dedicated thread */
K_THREAD_STACK_DEFINE(stack_area, STACK_SIZE);
/* semaphore for channels */
struct k_sem ch_sem;
/* mutex for channel reader count */
struct k_mutex ch_reader_cnt_mutex;
/* mutex for channel readers to wait for writer */
struct k_mutex ch_writer_mutex;
/* msgq to store sbus data */
struct k_work_q work_q;
/* sbus work item */
struct k_work sbus_work_item;
/* struct for drive variables */
struct drive_arg {
  struct k_work drive_work_item;      // drive work item
  struct k_work auto_drive_work_item; // autonomous drive work item
  struct DiffDriveConfig drive_config;
  struct DiffDriveTwist cmd;
  struct DiffDrive *drive_init;
  uint64_t time_last_drive_update;
} drive;
/* struct for arm variables */
struct arm_arg {
  enum StepperDirection dir[5];
  int pos[5];
  struct k_work imu_work_item;
  struct k_work channel_work_item;
  struct joint baseLink;
  struct joint lowerIMU;
  struct joint upperIMU;
  struct joint endIMU;
  struct joint rover;
} arm;
/* struct for communication with latte panda*/
struct com_arg {
  struct k_work cobs_rx_work_item;
  struct k_work telemetry_tx_work_item;
  struct k_work latte_panda_tx_work_item;
  struct cmd_msg msg_rx;     // to store decoded mssg
  struct cmd_msg inv_msg_tx; // to store encoded inverse mssg
  struct base_station_msg bs_msg_tx;
} com;

uint8_t gps_mssg[100];             // to store gps mssg
int gps_bytes_read;                // to store number of gps bytes read
uint16_t error_mssg_flag = 0x0000; // to store error status byte
const int INV_MSG_LEN = sizeof(struct cmd_msg) + 2; // len of inverse mssg
const int BS_MSG_LEN =
    sizeof(struct base_station_msg) + 2; // len of base station mssg
uint8_t rx_buf[sizeof(struct cmd_msg) + 2] = {0};
uint8_t inv_tx_buf[sizeof(struct cmd_msg) + 2] = {0};
uint8_t bs_tx_buf[sizeof(struct base_station_msg) + 2] = {0};
int cobs_bytes_read; // to store number of cobs bytes read
/* range variables */
float linear_velocity_range[] = {-1.5, 1.5};
float angular_velocity_range[] = {-5.5, 5.5};
float wheel_velocity_range[] = {-10.0, 10.0};
uint32_t pwm_range[] = {1100000, 1900000};
uint32_t servo_pwm_range[] = {500000, 2500000};
uint16_t channel_range[] = {172, 1811};

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
  if (!uart_irq_update(dev)) {
    return;
  }
  if (!uart_irq_rx_ready(dev)) {
    return;
  }
  while (uart_fifo_read(dev, &c, 1) == 1) {
    if (c == 0x00 && cobs_bytes_read > 0) {
      rx_buf[cobs_bytes_read] = 0;
      if (cobs_bytes_read != (INV_MSG_LEN - 1)) {
        cobs_bytes_read = 0;
        continue;
      }
      k_msgq_put(&msgq_rx, rx_buf, K_NO_WAIT);
      k_work_submit_to_queue(&work_q, &com.cobs_rx_work_item);
      cobs_bytes_read = 0;
    } else if (cobs_bytes_read < sizeof(rx_buf)) {
      rx_buf[cobs_bytes_read++] = c;
    }
  }
}
/* received cobs message work handler */
void cobs_rx_work_handler(struct k_work *cobs_rx_work_ptr) {
  uint8_t buf[INV_MSG_LEN];
  struct com_arg *com_info =
      CONTAINER_OF(cobs_rx_work_ptr, struct com_arg, cobs_rx_work_item);
  k_msgq_get(&msgq_rx, buf, K_MSEC(4));
  cobs_decode_result result =
      cobs_decode((void *)&(com_info->msg_rx), sizeof(com_info->msg_rx), buf,
                  INV_MSG_LEN - 1);
  if (result.status != COBS_DECODE_OK) {
    error_mssg_flag = error_mssg_flag | 0x0002;
    // printk("COBS Decode Failed %d\n", result.status);
    return;
  }
  if (check_crc(&com_info->msg_rx) != 0) {
    error_mssg_flag = error_mssg_flag | 0x0004;
    // printk("Message from latte panda is corrupt\n");
    return;
  }
  if (com_info->msg_rx.type == AUTONOMOUS) {
    gpio_pin_set_dt(&auto_led, 1);
    k_work_submit_to_queue(&work_q, &(drive.auto_drive_work_item));
  }
}

void latte_panda_tx_work_handler(struct k_work *latte_panda_tx_work_ptr) {
  struct com_arg *com_info = CONTAINER_OF(
      latte_panda_tx_work_ptr, struct com_arg, latte_panda_tx_work_item);
  k_msgq_get(&gps_msgq, com_info->bs_msg_tx.gps_msg, K_MSEC(4));
  com_info->bs_msg_tx.msg_status = error_mssg_flag;
  com_info->bs_msg_tx.crc =
      crc32_ieee((uint8_t *)(&com_info->bs_msg_tx),
                 sizeof(struct base_station_msg) - sizeof(uint32_t));
  cobs_encode_result result =
      cobs_encode(bs_tx_buf, BS_MSG_LEN, (void *)&com_info->bs_msg_tx,
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

/* autonomous drive work handler */
void auto_drive_work_handler(struct k_work *auto_drive_work_ptr) {
  struct drive_arg *drive_info =
      CONTAINER_OF(auto_drive_work_ptr, struct drive_arg, auto_drive_work_item);
  uint64_t drive_timestamp = k_uptime_get();
  drive_info->cmd.angular_z = com.msg_rx.auto_cmd.angular_z;
  drive_info->cmd.linear_x = com.msg_rx.auto_cmd.linear_x;
  if (diffdrive_update(drive_info->drive_init, drive_info->cmd,
                       drive_info->time_last_drive_update) == 0) {
  }
  drive_info->time_last_drive_update = k_uptime_get() - drive_timestamp;
}
/* timer to write mssg to latte panda */
void mssg_timer_handler(struct k_timer *mssg_timer_ptr) {
  k_work_submit_to_queue(&work_q, &(com.latte_panda_tx_work_item));
}
K_TIMER_DEFINE(mssg_timer, mssg_timer_handler, NULL);

int main() {

  printk("Tarzan version %s\nFile: %s\n", TARZAN_GIT_VERSION, __FILE__);

  int err;
  // uint32_t dtr = 0;

  /* initializing work queue */
  k_work_queue_init(&work_q);
  /* initializing work items */
  k_work_init(&(drive.auto_drive_work_item), auto_drive_work_handler);
  k_work_init(&(com.cobs_rx_work_item), cobs_rx_work_handler);
  k_work_init(&(com.latte_panda_tx_work_item), latte_panda_tx_work_handler);

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
  // /* get uart line control */
  // while (!dtr) {
  //   printk("error\n");
  //   uart_line_ctrl_get(latte_panda_uart, UART_LINE_CTRL_DTR, &dtr);
  //   k_sleep(K_MSEC(100));
  // }
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

  /* start running work queue */
  k_work_queue_start(&work_q, stack_area, K_THREAD_STACK_SIZEOF(stack_area),
                     PRIORITY, NULL);
  /* enable interrupt to receive cobs data */
  uart_irq_rx_enable(latte_panda_uart);
  /* enable interrupt to receive gps data */
  uart_irq_rx_enable(gps_uart);

  k_timer_start(&mssg_timer, K_MSEC(10), K_SECONDS(1));
}
