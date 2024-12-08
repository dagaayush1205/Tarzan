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

#include <kyvernitis/lib/kyvernitis.h>

#include <Tarzan/lib/arm.h>
#include <Tarzan/lib/drive.h>
#include <Tarzan/lib/sbus.h>

#define STACK_SIZE 4096 // work_q thread stack size
#define PRIORITY 2      // work_q thread priority

#define STEPPER_TIMER 100 // stepper pulse width in microseconds


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
const struct device *const lower = DEVICE_DT_GET(DT_ALIAS(imu_lower_joint));
const struct device *const upper = DEVICE_DT_GET(DT_ALIAS(imu_upper_joint));
const struct device *const base = DEVICE_DT_GET(DT_ALIAS(imu_turn_table));
/* defining sbus message queue*/
K_MSGQ_DEFINE(uart_msgq, 25 * sizeof(uint8_t), 10, 1);
/* workq dedicated thread */
K_THREAD_STACK_DEFINE(stack_area, STACK_SIZE);
/* mutex for sbus channels */
struct k_mutex ch_mutex;
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
  struct k_work IMU_work_item;
  uint16_t cmd[5];
  int pos[5];
  struct joint baseIMU;
  struct joint lowerIMU;
  struct joint upperIMU;
} arm;


uint16_t channel[16] = {0}; // to store sbus channels
uint8_t packet[25];         // to store sbus packets
int bytes_read;             // to store number of sbus bytes read
float linear_velocity_range[] = {-1.5, 1.5};
float angular_velocity_range[] = {-5.5, 5.5};
float wheel_velocity_range[] = {-10.0, 10.0};
uint32_t pwm_range[] = {1120000, 1830000};
uint16_t channel_range[] = {172, 1811};

/* interrupt to store sbus data */
void serial_cb(const struct device *dev, void *user_data) {
  ARG_UNUSED(user_data);
  uint8_t c;
  if (!uart_irq_update(uart_dev))
    return;
  if (!uart_irq_rx_ready(uart_dev))
    return;
  while (bytes_read < 25 && uart_fifo_read(uart_dev, &c, 1)) {
    if (bytes_read == 0 && c != 0x0f)
      continue;
    packet[bytes_read++] = c;
  }
  if (bytes_read == 25) {
    k_msgq_put(&uart_msgq, &packet, K_NO_WAIT);
    k_work_submit_to_queue(&work_q, &sbus_work_item);
    bytes_read = 0;
  }
}

/* work handler to form sbus packet and return sbus channels */
void sbus_work_handler(struct k_work *sbus_work_ptr) {
  uint8_t buffer[25] = {0};
  int err;
  k_msgq_get(&uart_msgq, buffer, K_NO_WAIT);
  err = parity_checker(packet[23]);
  if (err == 1) { 
    printk("error\n");
  } else {
    if (k_mutex_lock(&ch_mutex, K_NO_WAIT) == 0) {
      parse_buffer(buffer, channel);
      k_mutex_unlock(&ch_mutex);
      // for (int i = 0; i < 8; i++)
      //   printk("%u\t", channel[i]);
      // printk("\n");
      k_work_submit_to_queue(&work_q, &(drive.drive_work_item));
      k_work_submit_to_queue(&work_q, &(arm.IMU_work_item));
    }
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
  // gripper motor write
  if (pwm_motor_write(&(motor[8]), sbus_pwm_interpolation(channel[6], pwm_range,
                                                          channel_range)))
    printk("Gripper: Unable to write at gripper");
}

void arm_imu_handler(struct k_work *IMU_work_ptr){
  struct arm_arg *arm_info = CONTAINER_OF(IMU_work_ptr, struct arm_arg, IMU_work_item);
  process_mpu6050(lower, &arm_info->lowerIMU, 1);
  process_mpu6050(upper, &arm_info->upperIMU, 2);
  process_mpu6050(base, &arm_info->baseIMU, 3);
  printk("% .0f\t% .0f\t% .0f\t% .0f\t% .0f\t% .0f\n",arm.lowerIMU.pitch, arm.lowerIMU.roll, arm.upperIMU.pitch, arm.upperIMU.roll, arm.baseIMU.pitch, arm.baseIMU.roll);
}
void arm_work_handler() {
  if (k_mutex_lock(&ch_mutex, K_NO_WAIT) == 0) {
    arm.cmd[0] = channel[9];
    arm.cmd[1] = channel[4];
    arm.cmd[2] = channel[5];
    arm.cmd[3] = channel[7];
    arm.cmd[4] = channel[8];
    /* writing to stepper motor */
    for (int i = 0; i < 5; i++) {
      if (arm.cmd[0] > 1000) {
        arm.pos[i] = Stepper_motor_write(&stepper[i], HIGH_PULSE, arm.pos[i]);
      } else if (arm.cmd[0] < 800)
        arm.pos[i] = Stepper_motor_write(&stepper[i], LOW_PULSE, arm.pos[i]);
      else
        continue;
    }
    k_mutex_unlock(&ch_mutex);
  }
}

/* main timer to submit work items */
void main_timer_handler(struct k_timer *main_timer_ptr) { arm_work_handler(); }

K_TIMER_DEFINE(main_timer, main_timer_handler, NULL);

int main() {

  printk("Tarzan version %s\nFile: %s\n", GIT_BRANCH_NAME, __FILE__);
  /* initializing work queue */
  k_work_queue_init(&work_q);
  /* initializing mutex for channels */
  k_mutex_init(&ch_mutex);
  /* initializing work items */
  k_work_init(&sbus_work_item, sbus_work_handler);
  k_work_init(&(drive.drive_work_item), drive_work_handler);
  k_work_init(&(arm.IMU_work_item),arm_imu_handler);
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

  /* uart ready check */
  if (!device_is_ready(uart_dev)) {
    printk("UART device not ready");
  }
/*Devcice checks for imu */
  if (!device_is_ready(lower)) {
    printk("Device %s is not ready\n", lower->name);
  }
  if (!device_is_ready(upper)) {
    printk("Device %s is not ready\n", upper->name);
  }
  if (!device_is_ready(base)) {
    printk("Device %s is not ready\n", base->name);
  }
  printk("Calibrating IMU %s\n", base->name);
  if (calibration(base,&arm.baseIMU)) {
    printk("Calibration failed for device %s\n", base->name);
  }

  printk("Calibrating IMU %s\n", lower->name);
  if(calibration(lower, &arm.lowerIMU)) {
    printk("Calibration failed for device %s\n", lower->name);
  }

  printk("Calibrating IMU %s\n", upper->name);
  if(calibration(upper, &arm.upperIMU)){
    printk("Calibration failed for device %s\n", upper->name);
  }


  /* set sbus uart for interrupt */
  int err = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
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

  printk("\nInitialization completed successfully!\n");

  /* start running work queue */
  k_work_queue_start(&work_q, stack_area, K_THREAD_STACK_SIZEOF(stack_area),
                     PRIORITY, NULL);
  /* enable interrupt to receive sbus data */
  uart_irq_rx_enable(uart_dev);
  k_timer_start(&main_timer, K_SECONDS(1), K_USEC((STEPPER_TIMER) / 2));
}
