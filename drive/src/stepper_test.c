#include "zephyr/sys/printk.h"
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <kyvernitis/lib/kyvernitis.h>

#include <Tarzan/lib/arm.h>
#include <Tarzan/lib/drive.h>
#include <Tarzan/lib/sbus.h>
//110101000
LOG_MODULE_REGISTER(stepper_test, CONFIG_LOG_DEFAULT_LEVEL);
static const struct device *const uart_dev =
    DEVICE_DT_GET(DT_ALIAS(sbus_uart)); // data from SBUS
static const struct device *const uart_debug =
    DEVICE_DT_GET(DT_ALIAS(debug_uart)); // debugger

const struct device* imu_lower_joint = DEVICE_DT_GET(DT_ALIAS(imu_lower_joint));
const struct device* imu_upper_joint = DEVICE_DT_GET(DT_ALIAS(imu_pitch_roll));
const struct device* imu_pitch_roll = DEVICE_DT_GET(DT_ALIAS(imu_upper_joint));
# define M_PI  3.14159265358979323846

struct arm_arg {
  // uint16_t direction[5];
  int dir[5];
  int pos[5];
  struct k_work imu_work_item;
  struct joint lowerIMU;
  struct joint upperIMU;
  struct joint endIMU;
} arm;
 float accel_offset [3] = {-0.1, 0.04, -0.59}, gyro_offset[3] = {0.016, -0.055, 0.005};
float angle = 0, k = 0.89; // k here is tau
float target_angle = -50;
uint64_t prev_time = 0;
// DT spec for stepper
const struct stepper_motor stepper[3] = {
    {.dir = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor2), dir_gpios),
     .step = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor2), step_gpios)},
    {.dir = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor3), dir_gpios),
     .step = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor3), step_gpios)},
    {.dir = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor4), dir_gpios),
     .step = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor4), step_gpios)}};

// creating mssg queue to store data
K_MSGQ_DEFINE(uart_msgq, sizeof(uint8_t), 250, 1);

// ranges used in interpolation;
uint16_t channel_range[] = {0, 950, 2047};

uint16_t *ch;
int pos[3] = {0};
uint64_t time, last_time = 0.0;
float stepInterval;
float true_acc[3] = {0, 0, -9.8};
float true_gyro[3] = {0, 0, 0};

int _calibration(const struct device *dev) {
  struct sensor_value accel[3];
  struct sensor_value gyro[3];

   for (int i = 0; i < 1000; i++) {
  //
     int rc = sensor_sample_fetch(dev);

     if (rc == 0)
       rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);

     if (rc == 0)
       rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);

     for (int i = 0; i < 3; i++) {
       gyro_offset[i] += (sensor_value_to_double(&gyro[i]) - true_gyro[i]); 
     }
     k_sleep(K_MSEC(1));
   }
   for (int i = 0; i < 3; i++) {
     gyro_offset[i] = gyro_offset[i] / 1000.0;
   }
  printk("Calibration done\n");
  printk("accel_offset: %0.4f %0.4f %0.4f\n", accel_offset[0], accel_offset[1], accel_offset[2]);
  printk("gyroOffset: %0.4f %0.4f %0.4f\n", gyro_offset[0], gyro_offset[1], gyro_offset[2]);
  
  return 0;
}

static int _process_mpu6050(const struct device *dev, int n) {
  
  struct sensor_value accel[3];
  struct sensor_value gyro[3];
  
  uint64_t current_time = k_uptime_get();
  
  float dt = (current_time - prev_time) / 1000.0;
  
  prev_time = current_time;
  
  float a[3], g[3];
  
  int rc = sensor_sample_fetch(dev);

  if (rc == 0)
    rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
  if (rc == 0)
    rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);

  a[0] = sensor_value_to_double(&accel[0]);
  a[1] = sensor_value_to_double(&accel[1]);
  a[2] = sensor_value_to_double(&accel[2]);
  g[0] = sensor_value_to_double(&gyro[0]) - gyro_offset[0];
  g[1] = sensor_value_to_double(&gyro[1]) - gyro_offset[1];
  g[2] = sensor_value_to_double(&gyro[2]) - gyro_offset[2];

   printk("%d accel % .1f % .1f % .1f m/s/s\t gyro % .1f % .1f % .1f rad/s\n", n, a[0], a[1], a[2], g[0], g[1], g[2]);
  
  if (rc != 0) printk("Sample get/failed\n");
    float pitch_acc = (180 * atan2(-1 * a[0], sqrt(pow(a[1], 2) + pow(a[2], 2))) /M_PI);
    angle = k * (angle + (g[1]) * (dt)) + (1 - k) * pitch_acc;
    printk("ANGLE: % .0f\n", angle);
    if(target_angle > angle+2){
      ch[0]=1500;
      ch[1]=500;
      printk("Move down\n");
    }
    else if(target_angle < angle-2){
      ch[0]=500; 
      ch[1]=1500; 
      printk("Move up\n");
    }
    else {
        ch[0]=940;
        ch[1]=940; 
      printk("target angle reached\n");
    }

    // printk("sample fetch/get failed: %d\n", rc);
  return rc;
}

int _Stepper_motor_write(const struct stepper_motor *motor, uint16_t cmd, int pos) {

  if (abs(cmd - channel_range[1]) < 200) {
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

void _arm_joints(struct k_work *work) {
  uint16_t cmd[2] = {ch[0], ch[1]};
  for (int i=0 ; i<2; i++) {
    pos[i] = _Stepper_motor_write(&stepper[i], cmd[i], pos[i]);
    }
  }
K_WORK_DEFINE(my_work, _arm_joints);
void my_timer_handler(struct k_timer *dummy) { 
  k_work_submit(&my_work); }
K_TIMER_DEFINE(my_timer, my_timer_handler, NULL);

int main() {

  printk("This is tarzan version %s\nFile: %s\n", GIT_BRANCH_NAME, __FILE__);
  int err;
  if (!device_is_ready(imu_lower_joint))
    printk("Lower joint IMU %s: Not ready\n", imu_lower_joint->name);
  if (!device_is_ready(imu_upper_joint))
    printk("Upper joint IMU %s: Not ready\n", imu_upper_joint->name);
  if (!device_is_ready(imu_pitch_roll))
    printk("Pitch Roll IMU %s: Not ready\n", imu_pitch_roll->name);

  printk("Calibrating...\n");

  if (calibration(imu_lower_joint, &arm.lowerIMU)) {
    printk("Lower joint IMU %s: Calibration failed\n", imu_lower_joint->name);
    return 0;
  }
  if (calibration(imu_upper_joint, &arm.upperIMU)) {
    printk("Upper joint IMU %s: Calibration failed\n", imu_upper_joint->name);
    return 0;
  }
  if (calibration(imu_pitch_roll, &arm.endIMU)) {
    printk("Pitch Roll IMU %s: Calibration failed\n", imu_pitch_roll->name);
    return 0;
  }
  printk("\nInitialization completed successfully!\n");

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

  printk("Initialization completed successfully!\n");
  k_timer_start(&my_timer, K_USEC(120), K_USEC(50));
  while(true){
    if (process_mpu6050(imu_lower_joint, &(arm.lowerIMU)) == 1) printk("No data from imu_lower_joint: %s\n", imu_lower_joint->name);
    if (process_mpu6050(imu_upper_joint, &(arm.upperIMU)) == 1) printk("No data from imu_upper_joint: %s\n", imu_upper_joint->name);
  printk("IMU: %03.3f %03.3f %03.3f\n",
         (180*arm.lowerIMU.pitch)/M_PI, (180*arm.upperIMU.pitch)/M_PI,
         (180*arm.endIMU.pitch)/M_PI);
    k_sleep(K_MSEC(100));
  }
}
