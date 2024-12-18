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
static const struct device *const sbus_uart = DEVICE_DT_GET(DT_ALIAS(sbus_uart));
/* latte panda uart */
static const struct device *const latte_panda_uart = DEVICE_DT_GET(DT_ALIAS(latte_panda_uart));

/* DT spec for pwm motors */
#define PWM_MOTOR_SETUP(pwm_dev_id)                                            \
  {.dev_spec = PWM_DT_SPEC_GET(pwm_dev_id),                                    \
   .min_pulse = DT_PROP(pwm_dev_id, min_pulse),                                \
   .max_pulse = DT_PROP(pwm_dev_id, max_pulse)},
struct pwm_motor motor[10] = {
    DT_FOREACH_CHILD(DT_PATH(pwmmotors), PWM_MOTOR_SETUP)};
/* DT spec for stepper */
const struct stepper_motor stepper[5] = {
    {.dir = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor1), dir_gpios),
     .step = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor1), step_gpios)},
    {.dir = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor2), dir_gpios),
     .step = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor2), step_gpios)},
    {.dir = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor3), dir_gpios),
     .step = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor3), step_gpios)},
    {.dir = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor4), dir_gpios),
     .step = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor4), step_gpios)},
    {.dir = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor5), dir_gpios),
     .step = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor5), step_gpios)}};
/*DT spec for IMU */
const struct device *const imu[3] = {
    DEVICE_DT_GET(DT_ALIAS(imu_lower_joint)),
    DEVICE_DT_GET(DT_ALIAS(imu_upper_joint)),
    DEVICE_DT_GET(DT_ALIAS(imu_pitch_roll)),
};
/* defining sbus message queue*/
K_MSGQ_DEFINE(uart_msgq, 25 * sizeof(uint8_t), 10, 1);
/* defining cobs message queue */
K_MSGQ_DEFINE(msgq_rx, sizeof(struct inverse_msg) + 2, 50, 1);
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
  struct k_work drive_work_item; // drive work item
  struct DiffDriveConfig drive_config;
  struct DiffDriveTwist cmd;
  struct DiffDrive *drive_init;
  uint64_t time_last_drive_update;
} drive;
/* struct for arm variables */
struct arm_arg {
  // uint16_t direction[5];
  int dir[5];
  int pos[5];
  struct k_work imu_work_item;
  struct joint lowerIMU;
  struct joint upperIMU;
  struct joint endIMU;
} arm;
/* struct for communication with latte panda*/
struct com_arg {
  struct k_work cobs_rx_work_item;
  struct k_work cobs_tx_work_item;
  struct inverse_msg msg_rx; // to store decoded mssg
  struct inverse_msg msg_tx; // to store encoded mssg
} com;

int ch_reader_cnt;          // no. of readers accessing channels
uint16_t channel[16] = {0}; // to store sbus channels
uint8_t packet[25];         // to store sbus packets
int sbus_bytes_read;        // to store number of sbus bytes read
const int MSG_LEN = sizeof(struct inverse_msg) + 2; // len of cobs mssg
uint8_t rx_buf[sizeof(struct inverse_msg) + 2] = {0};
uint8_t tx_buf[sizeof(struct inverse_msg) + 2] = {0};
int cobs_bytes_read; // to store number of cobs bytes read
/* range variables */
float linear_velocity_range[] = {-1.5, 1.5};
float angular_velocity_range[] = {-5.5, 5.5};
float wheel_velocity_range[] = {-10.0, 10.0};
uint32_t pwm_range[] = {1120000, 1830000};
uint32_t servo_pwm_range[] = {500000, 2500000};
uint16_t channel_range[] = {172, 1811};

/* interrupt to store sbus data */
void sbus_cb(const struct device *dev, void *user_data) {
  ARG_UNUSED(user_data);
  uint8_t c;
  if (!uart_irq_update(sbus_uart))
    return;
  if (!uart_irq_rx_ready(sbus_uart))
    return;
  while (sbus_bytes_read < 25 && uart_fifo_read(sbus_uart, &c, 1)) {
    if (sbus_bytes_read == 0 && c != 0x0f)
      continue;
    packet[sbus_bytes_read++] = c;
  }
  if (sbus_bytes_read == 25) {
    k_msgq_put(&uart_msgq, &packet, K_NO_WAIT);
    k_work_submit_to_queue(&work_q, &sbus_work_item);
    sbus_bytes_read = 0;
  }
}
/* interrupt to store message from latte panda */
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
      if (cobs_bytes_read != (MSG_LEN - 1)) {
        cobs_bytes_read = 0;
        continue;
      }
      k_msgq_put(&msgq_rx, rx_buf, K_NO_WAIT);
      k_work_submit_to_queue(&work_q, &com.cobs_rx_work_item);
      k_work_submit_to_queue(&work_q, &(arm.imu_work_item));
      cobs_bytes_read = 0;
    } else if (cobs_bytes_read < sizeof(rx_buf)) {
      rx_buf[cobs_bytes_read++] = c;
    }
  }
}
/* work handler to form sbus packet and return sbus channels */
void sbus_work_handler(struct k_work *sbus_work_ptr) {
  uint8_t buffer[25] = {0};
  int err;
  k_msgq_get(&uart_msgq, buffer, K_NO_WAIT);
  err = parity_checker(packet[23]);
  if (err == 1) {
    printk("Corrupt SBus Packet\n");
  } else {
    if (k_mutex_lock(&ch_writer_mutex, K_FOREVER) == 0) {
      if (k_sem_take(&ch_sem, K_NO_WAIT) == 0) {
        parse_buffer(buffer, channel);
        k_sem_give(&ch_sem);
        k_mutex_unlock(&ch_writer_mutex);
        for (int i = 0; i < 8; i++)
          printk("%u\t", channel[i]);
        printk("\n");
        k_work_submit_to_queue(&work_q, &(drive.drive_work_item));
      }
      k_mutex_unlock(&ch_writer_mutex);
    }
  }
}
/* cobs message work handler */
void cobs_rx_work_handler(struct k_work *cobs_rx_work_ptr) {
  uint8_t buf[MSG_LEN];
  struct com_arg *com_info =
      CONTAINER_OF(cobs_rx_work_ptr, struct com_arg, cobs_rx_work_item);
  k_msgq_get(&msgq_rx, buf, K_MSEC(4));
  cobs_decode_result result = cobs_decode(
      (void *)&(com_info->msg_rx), sizeof(com_info->msg_rx), buf, MSG_LEN - 1);
  if (result.status != COBS_DECODE_OK) {
    printk("Error: COBS Decode Failed %d\n", result.status);
    return;
  }
}
void cobs_tx_work_handler(struct k_work *cobs_tx_work_ptr) {
  struct com_arg *com_info =
      CONTAINER_OF(cobs_tx_work_ptr, struct com_arg, cobs_tx_work_item);
  //k_work_submit_to_queue(&work_q, &(arm.imu_work_item));

  com_info->msg_tx.turn_table = 0;
  com_info->msg_tx.first_link = arm.lowerIMU.pitch;
  com_info->msg_tx.second_link = arm.upperIMU.pitch;
  com_info->msg_tx.pitch = arm.endIMU.pitch;
  com_info->msg_tx.roll = 0;
  com_info->msg_tx.x = 0;
  com_info->msg_tx.y = 0;
  com_info->msg_tx.z = 0;

  cobs_encode_result result = cobs_encode(
      tx_buf, MSG_LEN, (void *)&com_info->msg_tx, sizeof(struct inverse_msg));

  if (result.status != COBS_ENCODE_OK) {
    printk("Error: COBS Encoded Failed %d\n", result.status);
    return;
  }
  tx_buf[MSG_LEN - 1] = 0x00;
  for (int i = 0; i < MSG_LEN; i++) {
    uart_poll_out(latte_panda_uart, tx_buf[i]);
  }
}
int velocity_callback(const float *velocity_buffer, int buffer_len,
                      int wheels_per_side) {
  if (buffer_len < wheels_per_side * 2) {
    return 1;
  }
  if (pwm_motor_write(&(motor[0]), velocity_pwm_interpolation(
                                       *(velocity_buffer), wheel_velocity_range,
                                       pwm_range))) {
    printk("Drive: Unable to write pwm pulse to Left");
    return 1;
  }
  if (pwm_motor_write(&(motor[1]), velocity_pwm_interpolation(
                                       *(velocity_buffer + wheels_per_side + 1),
                                       wheel_velocity_range, pwm_range))) {
    printk("Drive: Unable to write pwm pulse to Right");
    return 1;
  }
  return 0;
}

int feedback_callback(float *feedback_buffer, int buffer_len,
                      int wheels_per_side) {
  return 0;
}

/* work handler to write to motors */
void drive_work_handler(struct k_work *drive_work_ptr) {
  struct drive_arg *drive_info =
      CONTAINER_OF(drive_work_ptr, struct drive_arg, drive_work_item);
  if (k_mutex_lock(&ch_writer_mutex, K_NO_WAIT) != 0) {
    return;
  }
  k_mutex_unlock(&ch_writer_mutex);
  k_mutex_lock(&ch_reader_cnt_mutex, K_FOREVER);
  if (ch_reader_cnt == 0) {
    k_sem_take(&ch_sem, K_FOREVER);
  }
  ch_reader_cnt++;
  k_mutex_unlock(&ch_reader_cnt_mutex);

  // drive motor write
  uint64_t drive_timestamp = k_uptime_get();
  drive_info->cmd.angular_z = sbus_velocity_interpolation(
      channel[0], angular_velocity_range, channel_range);
  drive_info->cmd.linear_x = sbus_velocity_interpolation(
      channel[1], linear_velocity_range, channel_range);
  diffdrive_update(drive_info->drive_init, drive_info->cmd,
                   drive_info->time_last_drive_update);
  drive_info->time_last_drive_update = k_uptime_get() - drive_timestamp;
  // linear actuator write
  if (pwm_motor_write(&(motor[2]), sbus_pwm_interpolation(channel[2], pwm_range,
                                                          channel_range)))
    printk("Linear Actuator: Unable to write at linear actuator");
  if (pwm_motor_write(&(motor[3]), sbus_pwm_interpolation(channel[3], pwm_range,
                                                          channel_range)))
    printk("Linear Actuator: Unable to write at linear actuator");
  // pan servo write
  if (pwm_motor_write(
          &(motor[6]),
          sbus_pwm_interpolation(channel[9], servo_pwm_range, channel_range)))
    printk("Pan Servo: Unable to write at gripper");
  // tilt servo write
  if (pwm_motor_write(
          &(motor[7]),
          sbus_pwm_interpolation(channel[10], servo_pwm_range, channel_range)))
    printk("Tilt Servo: Unable to write at gripper");
  k_mutex_lock(&ch_reader_cnt_mutex, K_FOREVER);
  ch_reader_cnt--;
  if (ch_reader_cnt == 0) {
    k_sem_give(&ch_sem);
  }
  k_mutex_unlock(&ch_reader_cnt_mutex);
}

/* work handler for processing imu */
void arm_imu_work_handler(struct k_work *imu_work_ptr) {
  struct arm_arg *arm_info =
      CONTAINER_OF(imu_work_ptr, struct arm_arg, imu_work_item);
  process_mpu6050(imu[1], &arm_info->endIMU);
  process_mpu6050(imu[2], &arm_info->lowerIMU);
  // process_mpu6050(imu[2], &arm_info->upperIMU); 
  k_work_submit_to_queue(&work_q, &(com.cobs_tx_work_item));
}

/* work handler for stepper motor write*/
void arm_stepper_work_handler(int *dir) {
  for (int i = 0; i < 5; i++) {
    if (dir == NULL)
      continue;
    arm.pos[i] = Stepper_motor_write(&stepper[i], dir[i], arm.pos[i]);
  }
}
// void arm_stepper_work_handler() {
//   if (k_mutex_lock(&ch_writer_mutex, K_NO_WAIT) != 0) {
//     return;
//   }
//   k_mutex_unlock(&ch_writer_mutex);
//   k_mutex_lock(&ch_reader_cnt_mutex, K_FOREVER);
//   if (ch_reader_cnt == 0) {
//     k_sem_take(&ch_sem, K_FOREVER);
//   }
//   ch_reader_cnt++;
//   k_mutex_unlock(&ch_reader_cnt_mutex);
//   arm.cmd[0] = channel[4];
//   arm.cmd[1] = channel[5];
//   arm.cmd[2] = channel[6];
//   /* writing to stepper motor */
//   for (int i = 0; i < 3; i++) {
//     if (arm.cmd[i] > 1000) {
//       arm.pos[i] = Stepper_motor_write(&stepper[i], HIGH_PULSE, arm.pos[i]);
//     } else if (arm.cmd[i] < 800)
//       arm.pos[i] = Stepper_motor_write(&stepper[i], LOW_PULSE, arm.pos[i]);
//     else
//       continue;
//   }
//   k_mutex_lock(&ch_reader_cnt_mutex, K_FOREVER);
//   ch_reader_cnt--;
//   if (ch_reader_cnt == 0) {
//     k_sem_give(&ch_sem);
//   }
//   k_mutex_lock(&ch_reader_cnt_mutex, K_FOREVER);
// }

/* timer to write to stepper motors*/
void stepper_timer_handler(struct k_timer *stepper_timer_ptr) {
  arm.dir[0] =
      update_proportional(com.msg_tx.turn_table, 0); // arm.baseIMU.yaw;
  arm.dir[1] = update_proportional(com.msg_tx.first_link, arm.lowerIMU.pitch);
  arm.dir[2] =
      update_proportional((com.msg_tx.second_link), arm.upperIMU.pitch);
  arm.dir[3] = update_proportional((com.msg_tx.pitch), arm.endIMU.pitch);
  arm.dir[4] = update_proportional(com.msg_tx.roll, arm.endIMU.roll);
  arm_stepper_work_handler(arm.dir);
}
K_TIMER_DEFINE(stepper_timer, stepper_timer_handler, NULL);

int main() {

  printk("Tarzan version %s\nFile: %s\n", GIT_BRANCH_NAME, __FILE__);

  int err, dtr;

  /* initializing work queue */
  k_work_queue_init(&work_q);
  /* initializing channel semaphore */
  k_sem_init(&ch_sem, 1, 1);
  /* initializing mutex for readers to wait */
  k_mutex_init(&ch_writer_mutex);
  /* inittializing mutex for reader count */
  k_mutex_init(&ch_reader_cnt_mutex);
  /* initializing work items */
  k_work_init(&sbus_work_item, sbus_work_handler);
  k_work_init(&(drive.drive_work_item), drive_work_handler);
  k_work_init(&(arm.imu_work_item), arm_imu_work_handler);
  k_work_init(&(com.cobs_rx_work_item), cobs_rx_work_handler);
  k_work_init(&(com.cobs_tx_work_item), cobs_tx_work_handler);
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
  /* initialize imu joints */
  struct joint initialize_imu = {{0, 0, 0}, {0, 0, 0}, 0, 0, 0, {0, 0, 0}};
  arm.upperIMU = initialize_imu;
  arm.lowerIMU = initialize_imu;
  arm.endIMU = initialize_imu;

  /* sbus uart ready check */
  if (!device_is_ready(sbus_uart)) {
    printk("SBUS UART device not ready");
  }
  /* latte panda uart ready check */
  if (!device_is_ready(latte_panda_uart)) {
    printk("LATTE PANDA UART device not ready");
  }
  if (usb_enable(NULL)) {
    return 0;
  }
  /* get uart line control */
  uart_line_ctrl_get(latte_panda_uart, UART_LINE_CTRL_DTR, &dtr);
  if (!dtr) {
    printk("Unable to get uart line contorl\n");
  }
  /* set sbus uart for interrupt */
  err = uart_irq_callback_user_data_set(sbus_uart, sbus_cb, NULL);
  if (err < 0) {
    if (err == -ENOTSUP) {
      printk("Interrupt-driven UART API support 1005not enabled");
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
      printk("Interrupt-driven UART API support 1005not enabled");
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
  /* stepper motor ready check */
  for (size_t i = 0U; i < 5; i++) {
    if (!gpio_is_ready_dt(&stepper[i].dir)) {
      printk("Stepper Motor %d: Dir %d is not ready", i, stepper[i].dir.pin);
    }
    if (!gpio_is_ready_dt(&stepper[i].step)) {
      printk("Stepper Motor %d: Dir %d is not ready", i, stepper[i].step.pin);
    }
  }
  /* configure stepper gpio for output */
  for (size_t i = 0U; i < 5; i++) {
    if (gpio_pin_configure_dt(&(stepper[i].dir), GPIO_OUTPUT_INACTIVE)) {
      printk("Error: Stepper motor %d: Dir %d not configured", i,
             stepper[i].dir.pin);
    }
    if (gpio_pin_configure_dt(&(stepper[i].step), GPIO_OUTPUT_INACTIVE)) {
      printk("Error: Stepper motor %d: Dir %d not configured", i,
             stepper[i].step.pin);
    }
  }
  /* IMU ready checks */
  for (int i = 0; i < 3; i++) {
    if (!device_is_ready(imu[i])) {
      printk("IMU:Device %s is not ready\n", imu[i]->name);
    }
  }
  /* Calibrating IMUs */
  printk("Calibrating IMU %s\n", imu[0]->name);
  if (calibration(imu[0], &arm.lowerIMU)) {
    printk("Calibration failed for device %s\n", imu[0]->name);
  }
  printk("Calibrating IMU %s\n", imu[1]->name);
  if (calibration(imu[1], &arm.upperIMU)) {
    printk("Calibration failed for device %s\n", imu[1]->name);
  }
  printk("Calibrating IMU %s\n", imu[2]->name);
  if (calibration(imu[2], &arm.endIMU)) {
    printk("Calibration failed for device %s\n", imu[2]->name);
  }
  printk("\nInitialization completed successfully!\n");

  /* start running work queue */
  k_work_queue_start(&work_q, stack_area, K_THREAD_STACK_SIZEOF(stack_area),
                     PRIORITY, NULL);
  /* enable interrupt to receive sbus data */
  uart_irq_rx_enable(sbus_uart);
  /* enablke interrupt to receive cobs data */
  uart_irq_rx_enable(latte_panda_uart);
  /* enabling stepper timer */
  k_timer_start(&stepper_timer, K_SECONDS(1), K_USEC((STEPPER_TIMER) / 2));
  k_work_submit_to_queue(&work_q, &(arm.imu_work_item));
}
