#include <stdint.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/thread_stack.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/util.h>
#include <zephyr/toolchain.h>
#include <zephyr/usb/usb_device.h>

#include <Tarzan/lib/arm.h>
#include <Tarzan/lib/cobs.h>
#include <Tarzan/lib/drive.h>
#include <Tarzan/lib/sbus.h>

LOG_MODULE_REGISTER(Tarzan, CONFIG_TARZAN_LOG_LEVEL);

#define STACK_SIZE 4096   // work_q thread stack size
#define PRIORITY 2        // work_q thread priority
#define STEPPER_TIMER 100 // stepper pulse width in microseconds
#define DRIVE_UPDATE_HZ 50
#define DRIVE_UPDATE_MS (1000 / DRIVE_UPDATE_HZ) // 20ms period
#define DT_SEC (1.0f / DRIVE_UPDATE_HZ)          // const dt = 0.02f

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
struct pwm_motor motor[12] = {
    DT_FOREACH_CHILD(DT_PATH(pwmmotors), PWM_MOTOR_SETUP)};

/* DT spec for stepper */
const struct stepper stepper[5] = {
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

/*DT spec for I2C devices */
const struct device *lsm_imu = DEVICE_DT_GET(DT_ALIAS(imu));
const struct device *lsm_mag = DEVICE_DT_GET(DT_ALIAS(mag));

/* DT spec for leds */
static const struct gpio_dt_spec init_led =
    GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec sbus_status_led =
    GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
const struct pwm_dt_spec error_led = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led0));

/* common msg struct for coms */
struct cmd_msg {
  struct DiffDriveTwist auto_cmd;
  uint32_t crc;
};

/* msg struct for com with base station */
struct base_station_msg {
  char gps_msg[100];
  uint32_t crc;
};

/* decleration of check_crc func*/
int check_crc(struct cmd_msg *);

/* defining sbus message queue*/
K_MSGQ_DEFINE(sbus_msgq, 25 * sizeof(uint8_t), 10, 1);
/* defining gps message queue */
K_MSGQ_DEFINE(gps_msgq, sizeof(uint8_t) * 100, 10, 1);
/* defining cobs message queue */
K_MSGQ_DEFINE(drive_msgq, sizeof(struct cmd_msg) + 2, 50, 1);

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
  struct DiffDriveCtx *drive_init;
  uint64_t time_last_drive_update;
  struct joint *lsm;
} drive;

/* struct for arm variables */
struct arm_arg {
  enum StepperDirection dir[5];
  int pos[5];
  struct k_work channel_work_item;
} arm;

/* struct for communication */
struct com_rx_arg {
  struct k_work cobs_rx_work_item;
  struct cmd_msg msg_rx;
  struct k_msgq *msgq_rx;
  int cobs_bytes_read;
  uint8_t rx_buf[sizeof(struct cmd_msg) + 2];
} drive_com = {.msgq_rx = &drive_msgq};

struct com_tx_arg {
  struct k_work latte_panda_tx_work_item;
  struct base_station_msg bs_msg_tx; // to store encoded base station mssg
} com_tx;

int ch_reader_cnt;          // no. of readers accessing channels
uint16_t channel[16] = {0}; // to store sbus channels
uint8_t packet[25];         // to store sbus packets
uint8_t gps_mssg[100];      // to store gps mssg
int sbus_bytes_read;        // to store number of sbus bytes read
int gps_bytes_read;         // to store number of gps bytes read
const int CMD_MSG_LEN = sizeof(struct cmd_msg) + 2; // len of inverse mssg
const int BS_MSG_LEN =
    sizeof(struct base_station_msg) + 2; // len of base station mssg
uint8_t bs_tx_buf[sizeof(struct base_station_msg) + 2] = {0};
/* range variables */
float linear_velocity_range[] = {-1.5, 1.5};
float angular_velocity_range[] = {-5.5, 5.5};
float wheel_velocity_range[] = {-10.0, 10.0};
uint32_t pwm_range[] = {1100000, 1900000};
uint32_t servo_pwm_range[] = {500000, 2500000};
uint16_t channel_range[] = {172, 1811};

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
    k_msgq_put(&sbus_msgq, &packet, K_NO_WAIT);
    k_work_submit_to_queue(&work_q, &sbus_work_item);
    sbus_bytes_read = 0;
  }
}

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

/* interrup to read cobs messages */
void cobs_cb(const struct device *dev, void *user_data) {
  struct com_rx_arg *com_ctx = (struct com_rx_arg *)user_data;
  uint8_t c;

  if (!uart_irq_update(dev)) {
    return;
  }
  if (!uart_irq_rx_ready(dev)) {
    return;
  }
  while (uart_fifo_read(dev, &c, 1) == 1) {
    if (c == 0x00 && com_ctx->cobs_bytes_read > 0) {
      com_ctx->rx_buf[com_ctx->cobs_bytes_read] = 0;
      if (com_ctx->cobs_bytes_read != (CMD_MSG_LEN - 1)) {
        com_ctx->cobs_bytes_read = 0;
        continue;
      }
      k_msgq_put(com_ctx->msgq_rx, com_ctx->rx_buf, K_NO_WAIT);
      k_work_submit_to_queue(&work_q, &com_ctx->cobs_rx_work_item);
      com_ctx->cobs_bytes_read = 0;
    } else if (com_ctx->cobs_bytes_read < sizeof(com_ctx->rx_buf)) {
      com_ctx->rx_buf[com_ctx->cobs_bytes_read++] = c;
    }
  }
}

/* work handler to form sbus packet and return sbus channels */
void sbus_work_handler(struct k_work *sbus_work_ptr) {
  uint8_t buffer[25] = {0};
  int err;
  k_msgq_get(&sbus_msgq, buffer, K_NO_WAIT);
  err = parity_checker(packet[23]);

  if (err == 1) {
    gpio_pin_set_dt(&sbus_status_led, 0); // set sbus status led low
    LOG_ERR("Corrupt SBUS Packet\n");
  } else {
    gpio_pin_set_dt(&sbus_status_led, 1); // set sbus status led high
    k_mutex_lock(&ch_writer_mutex, K_FOREVER);
    if (k_sem_take(&ch_sem, K_NO_WAIT) == 0) {
      k_mutex_unlock(&ch_writer_mutex);
      parse_buffer(buffer, channel);
      k_sem_give(&ch_sem);
      k_work_submit_to_queue(&work_q, &(arm.channel_work_item));
      k_work_submit_to_queue(&work_q, &(drive.drive_work_item));
    } else {
      k_mutex_unlock(&ch_writer_mutex);
    }
  }
}

/* received cobs message work handler */
void cobs_rx_work_handler(struct k_work *cobs_rx_work_ptr) {
  uint8_t buf[CMD_MSG_LEN];
  struct com_rx_arg *com_info = CONTAINER_OF(
      cobs_rx_work_ptr, struct com_rx_arg,
      cobs_rx_work_item); // changed type here to check for conflicts
  k_msgq_get(com_info->msgq_rx, buf, K_MSEC(4));
  cobs_decode_result result =
      cobs_decode((void *)&(com_info->msg_rx), sizeof(com_info->msg_rx), buf,
                  CMD_MSG_LEN - 1);
  if (result.status != COBS_DECODE_OK) {
    LOG_ERR("COBS Decode Failed %d\n", result.status);
    return;
  }
  if (check_crc(&com_info->msg_rx) != 0) {
    LOG_ERR("Message from latte panda is corrupt\n");
    return;
  }

  // submit autonomous drive handler
  k_work_submit_to_queue(&work_q, &(drive.auto_drive_work_item));
}

void latte_panda_tx_work_handler(struct k_work *latte_panda_tx_work_ptr) {
  struct com_tx_arg *com_info =
      CONTAINER_OF(latte_panda_tx_work_ptr, struct com_tx_arg,
                   latte_panda_tx_work_item); // changed type from som_arg to
                                              // com_tx_arg to match type
  k_msgq_get(&gps_msgq, com_info->bs_msg_tx.gps_msg, K_MSEC(4));
  com_info->bs_msg_tx.crc =
      crc32_ieee((uint8_t *)(&com_info->bs_msg_tx),
                 sizeof(struct base_station_msg) - sizeof(uint32_t));
  cobs_encode_result result =
      cobs_encode(bs_tx_buf, BS_MSG_LEN, (void *)&com_info->bs_msg_tx,
                  sizeof(struct base_station_msg));

  if (result.status != COBS_ENCODE_OK) {
    LOG_ERR("COBS Encoded Failed %d\n", result.status);
    return;
  }
  bs_tx_buf[BS_MSG_LEN - 1] = 0x00;
  for (int i = 0; i < BS_MSG_LEN; i++) {
    uart_poll_out(latte_panda_uart, bs_tx_buf[i]);
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
    LOG_ERR("Drive: Unable to write pwm pulse to Left");
    return 1;
  }
  if (pwm_motor_write(&(motor[1]), velocity_pwm_interpolation(
                                       *(velocity_buffer + wheels_per_side + 1),
                                       wheel_velocity_range, pwm_range))) {
    LOG_ERR("Drive: Unable to write pwm pulse to Right");
    return 1;
  }
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

  if (drive_info->time_last_drive_update >= DRIVE_UPDATE_MS) {

    drive_info->cmd.angular_z = sbus_velocity_interpolation(
        channel[0], angular_velocity_range, channel_range);
    drive_info->cmd.linear_x = sbus_velocity_interpolation(
        channel[1], linear_velocity_range, channel_range);

    if (madgwick_filter(lsm_mag, lsm_imu, drive_info->lsm)) {

      float yaw = drive_info->lsm->yaw;
      float yaw_rate = drive_info->lsm->gyro[2];

      diffdrive_update(drive_info->drive_init, drive_info->cmd, DT_SEC, yaw,
                       yaw_rate);

      drive_info->time_last_drive_update = k_uptime_get() - drive_timestamp;

    } else
      LOG_ERR("Rover lsm9ds1: Madgwick filter update failed");
  }

  // tilt servo write
  if (pwm_motor_write(
          &(motor[6]),
          sbus_pwm_interpolation(channel[10], servo_pwm_range, channel_range)))
    LOG_ERR("Tilt Servo: Unable to write");

  // pan servo write
  if (pwm_motor_write(
          &(motor[7]),
          sbus_pwm_interpolation(channel[9], servo_pwm_range, channel_range)))
    LOG_ERR("Pan Servo: Unable to write");

  /* mode 1 */
  if (channel[8] < 992) {

    // writing neutral to gripper
    if (pwm_motor_write(&(motor[4]), 1500000))
      LOG_ERR("Gripper: Unable to write");

    // linear actuator write
    if (pwm_motor_write(&(motor[2]), sbus_pwm_interpolation(
                                         channel[2], pwm_range, channel_range)))
      LOG_ERR("Linear Actuator 1: Unable to write");

    if (pwm_motor_write(&(motor[3]), sbus_pwm_interpolation(
                                         channel[7], pwm_range, channel_range)))
      LOG_ERR("Linear Actuator 2: Unable to write");

    // abox/mini-acctuator/ogger write
    if (pwm_motor_write(&(motor[5]), sbus_pwm_interpolation(
                                         channel[3], pwm_range, channel_range)))
      LOG_ERR("Mini-Acc/Ogger: Unable to write");
  }

  /* mode 0 (gripper/bio-arm written) */
  if (channel[8] >= 992) {
    // writing neutral to abox/mini acc
    if (pwm_motor_write(&(motor[5]), 1500000))
      LOG_ERR("Abox/Mini-Acc: Unable to write");

    if (pwm_motor_write(&(motor[4]), sbus_pwm_interpolation(
                                         channel[3], pwm_range, channel_range)))
      LOG_ERR("Gripper/Bio-Arm: Unable to write");
  }

  k_mutex_lock(&ch_reader_cnt_mutex, K_FOREVER);
  ch_reader_cnt--;
  if (ch_reader_cnt == 0) {
    k_sem_give(&ch_sem);
  }
  k_mutex_unlock(&ch_reader_cnt_mutex);
}

/* autonomous drive work handler */
void auto_drive_work_handler(struct k_work *auto_drive_work_ptr) {
  struct drive_arg *drive_info =
      CONTAINER_OF(auto_drive_work_ptr, struct drive_arg, auto_drive_work_item);

  uint64_t drive_timestamp = k_uptime_get();

  if (drive_info->time_last_drive_update >= DRIVE_UPDATE_MS) {

    drive_info->cmd.angular_z = drive_com.msg_rx.auto_cmd.angular_z;
    drive_info->cmd.linear_x = drive_com.msg_rx.auto_cmd.linear_x;

    if (madgwick_filter(lsm_mag, lsm_imu, drive_info->lsm)) {

      float yaw = drive_info->lsm->yaw;
      float yaw_rate = drive_info->lsm->gyro[2];

      diffdrive_update(drive_info->drive_init, drive_info->cmd, DT_SEC, yaw,
                       yaw_rate);

      drive_info->time_last_drive_update = k_uptime_get() - drive_timestamp;

    } else
      LOG_ERR("Rover lsm9ds1: Madgwick filter update failed");
  }
}

/* arm channel work handler */
void arm_channel_work_handler(struct k_work *work_ptr) {
  struct arm_arg *arm_info =
      CONTAINER_OF(work_ptr, struct arm_arg, channel_work_item);
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

  uint16_t arm_channels[5] = {channel[6], channel[4], channel[5], 850, 850};

  /* neutral */
  if (channel[8] >= 992) {
    // setting pitch (motors same direction)
    if (channel[7] > 992 || channel[7] < 800) {
      arm_channels[3] = channel[7];
      arm_channels[4] = channel[7];
    }
    // setting roll (motors opposite direction)
    if (channel[2] > 992 || channel[2] < 800) {
      arm_channels[3] = channel[2];
      arm_channels[4] =
          abs(channel[2] -
              1400); // subtracting arbitary value to set opposite direction
    }
    // do not move if cmd to both the channels
    if ((channel[2] > 992 || channel[2] < 800) &&
        (channel[7] > 992 || channel[7] < 800)) {
      arm_channels[3] = 850; // neutral
      arm_channels[4] = 850; // neutral
    }
  }

  for (int i = 0; i < 5; i++) {
    if (arm_channels[i] > 1185) {
      arm_info->dir[i] = HIGH_PULSE;
    } else if (arm_channels[i] < 800) {
      arm_info->dir[i] = LOW_PULSE;
    } else {
      arm_info->dir[i] = STOP_PULSE;
    }
  }

  k_mutex_lock(&ch_reader_cnt_mutex, K_FOREVER);
  ch_reader_cnt--;
  if (ch_reader_cnt == 0) {
    k_sem_give(&ch_sem);
  }
  k_mutex_unlock(&ch_reader_cnt_mutex);
}

/* timer to write to stepper motors*/
void stepper_timer_handler(struct k_timer *stepper_timer_ptr) {
  for (int i = 0; i < 5; i++) {
    arm.pos[i] = Stepper_motor_write(&stepper[i], arm.dir[i], arm.pos[i]);
  }
}

K_TIMER_DEFINE(stepper_timer, stepper_timer_handler, NULL);

/* timer to write mssg to latte panda */
void mssg_timer_handler(struct k_timer *mssg_timer_ptr) {
  k_work_submit_to_queue(&work_q, &(com_tx.latte_panda_tx_work_item));
}
K_TIMER_DEFINE(mssg_timer, mssg_timer_handler, NULL);

int main() {

  LOG_INF("Tarzan version %s\nFile: %s\n", TARZAN_GIT_VERSION, __FILE__);

  int err;

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
  k_work_init(&(drive.auto_drive_work_item), auto_drive_work_handler);
  k_work_init(&(arm.channel_work_item), arm_channel_work_handler);
  k_work_init(&(drive_com.cobs_rx_work_item), cobs_rx_work_handler);
  k_work_init(&(com_tx.latte_panda_tx_work_item), latte_panda_tx_work_handler);

  /* initializing drive configs */
  const struct DiffDriveConfig tmp_drive_config = {
      .wheel_separation = 0.77f,
      .wheel_separation_multiplier = 1,
      .wheel_radius = 0.15f,
      .wheels_per_side = 2,
      .command_timeout_seconds = 2,
      .left_wheel_radius_multiplier = 1,
      .right_wheel_radius_multiplier = 1,
  };
  drive.drive_config = tmp_drive_config;
  drive.drive_init = drive_init(&(drive.drive_config), velocity_callback);

  /* sbus uart ready check */
  if (!device_is_ready(sbus_uart))
    LOG_ERR("SBUS UART device not ready");
  /* gps uart ready check */
  if (!device_is_ready(gps_uart))
    LOG_ERR("GPS UART device not ready");
  /* latte panda uart ready check */
  if (!device_is_ready(latte_panda_uart))
    LOG_ERR("LATTE PANDA UART device not ready");
  /* enable usb for latte panda com */
  if (usb_enable(NULL))
    LOG_ERR("CDC ACM UART failed");

  /* set sbus uart for interrupt */
  err = uart_irq_callback_user_data_set(sbus_uart, sbus_cb, NULL);
  if (err < 0) {
    if (err == -ENOTSUP) {
      LOG_ERR("Interrupt-driven UART API support not enabled");
    } else if (err == -ENOSYS) {
      LOG_ERR("UART device does not support interrupt-driven API");
    } else {
      LOG_ERR("Error setting UART callback: %d", err);
    }
  }

  /* set gps uart for interrupt */
  err = uart_irq_callback_user_data_set(gps_uart, gps_cb, NULL);
  if (err < 0) {
    if (err == -ENOTSUP) {
      LOG_ERR("Interrupt-driven UART API support not enabled");
    } else if (err == -ENOSYS) {
      LOG_ERR("UART device does not support interrupt-driven API");
    } else {
      LOG_ERR("Error setting UART callback: %d", err);
    }
  }

  /* set telemtry uart for interrupt */
  err = uart_irq_callback_user_data_set(telemetry_uart, cobs_cb, &drive_com);
  if (err < 0) {
    if (err == -ENOTSUP) {
      LOG_ERR("Interrupt-driven UART API support not enabled");
    } else if (err == -ENOSYS) {
      LOG_ERR("UART device does not support interrupt-driven API");
    } else {
      LOG_ERR("Error setting UART callback: %d", err);
    }
  }

  /* pwm ready check */
  for (size_t i = 0U; i < ARRAY_SIZE(motor); i++) {
    if (!pwm_is_ready_dt(&(motor[i].dev_spec))) {
      LOG_ERR("PWM: Motor %s is not ready", motor[i].dev_spec.dev->name);
    }
  }
  for (size_t i = 0U; i < ARRAY_SIZE(motor); i++) {
    if (pwm_motor_write(&(motor[i]), 1500000)) {
      LOG_ERR("Unable to write pwm pulse to PWM Motor : %d\n", i);
    }
  }

  /* stepper motor ready check */
  for (size_t i = 0U; i < 5; i++) {
    if (!gpio_is_ready_dt(&stepper[i].dir)) {
      LOG_ERR("Stepper Motor %d: Dir %d is not ready", i, stepper[i].dir.pin);
    }
    if (!gpio_is_ready_dt(&stepper[i].step)) {
      LOG_ERR("Stepper Motor %d: Dir %d is not ready", i, stepper[i].step.pin);
    }
  }

  /* configure stepper gpio for output */
  for (size_t i = 0U; i < 5; i++) {
    if (gpio_pin_configure_dt(&(stepper[i].dir), GPIO_OUTPUT_INACTIVE)) {
      LOG_ERR("Stepper motor %d: Dir %d not configured", i, stepper[i].dir.pin);
    }
    if (gpio_pin_configure_dt(&(stepper[i].step), GPIO_OUTPUT_INACTIVE)) {
      LOG_ERR("Stepper motor %d: Dir %d not configured", i,
              stepper[i].step.pin);
    }
  }

  /* imu ready check */
  if (!device_is_ready(lsm_imu) || !device_is_ready(lsm_mag))
    LOG_ERR("Rover lsm9ds1 not ready");

  /* calibrate imu & mag */
  if (calibrate_gyro(lsm_imu, drive.lsm))
    LOG_ERR("Rover lsm9ds1 gyro calibration failed");
  if (calibrate_magnetometer(lsm_mag, drive.lsm))
    LOG_ERR("Rover lsm9ds1 gyro calibration failed");

  /* led ready checks */
  if (!gpio_is_ready_dt(&init_led)) {
    LOG_ERR("Initialization led not ready\n");
  }
  if (!gpio_is_ready_dt(&sbus_status_led)) {
    LOG_ERR("SBUS Status led not ready\n");
  }
  if (!pwm_is_ready_dt(&error_led)) {
    LOG_ERR("Error led is not ready");
  }
  if (gpio_pin_configure_dt(&init_led, GPIO_OUTPUT_INACTIVE) < 0) {
    LOG_ERR("Intitialization led not configured\n");
  }
  if (gpio_pin_configure_dt(&sbus_status_led, GPIO_OUTPUT_INACTIVE) < 0) {
    LOG_ERR("SBUS Status led not configured\n");
  }

  LOG_INF("Initialization completed successfully!\n");

  gpio_pin_set_dt(&init_led, 1); // set initialization led high

  /* start running work queue */
  k_work_queue_start(&work_q, stack_area, K_THREAD_STACK_SIZEOF(stack_area),
                     PRIORITY, NULL);

  /* enable interrupt to receive sbus data */
  uart_irq_rx_enable(sbus_uart);
  /* enable interrupt to receive cobs data */
  uart_irq_rx_enable(latte_panda_uart);
  /* enable interrupt to receive gps data */
  uart_irq_rx_enable(gps_uart);

  /* enabling stepper & mssg timer */
  k_timer_start(&stepper_timer, K_SECONDS(1), K_USEC((STEPPER_TIMER) / 2));
  k_timer_start(&mssg_timer, K_MSEC(10), K_SECONDS(1));

  return 0;
}
