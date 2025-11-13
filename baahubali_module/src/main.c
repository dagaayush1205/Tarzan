#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/sensor.h>
#include <Tarzan/lib/arm.h>


static struct imu_data imu_used = {
   .error_code=IMU_NOT_READY,
   .prev_time= 0.0,
   .pitch= 0.0,
   .roll= 0.0
};

static const struct device *imu_joint = DEVICE_DT_GET(DT_ALIAS(imu_joint));

static const struct device *const pico_uart = DEVICE_DT_GET(DT_ALIAS(pico_uart));

#define M_PI 3.14159265358979323846

//float accel_offset[3], gyro_offset[3];
float angle_pitch = 0.0, angle_roll = 0.0;
float k = 0.90; // k here is tau
float target_angle = -45;
uint64_t prev_time = 0;

float true_acc[3] = {0, 0, -9.8};
float true_gyro[3] = {0, 0, 0};

int calibration_mpu(const struct device *dev, struct imu_data *data);
int process_pitch_roll_mpu(const struct device *dev, struct imu_data *data);

int main(void)
{
    if (!device_is_ready(pico_uart)) {
    printk("Uart device not ready\n");
    }
    if (!device_is_ready(imu_joint)) {
        printk("IMU not ready.\n");
    }
    if (calibration_mpu(imu_joint, &imu_used)) {
    printk("Calibration failed for device %s\n", imu_joint->name);
    }
  //put main loop here
}

int calibration_mpu(const struct device *dev, struct imu_data *data) {
  struct sensor_value accel[3];
  struct sensor_value gyro[3];

  for (int i = 0; i < 1000; i++) {

    int rc = sensor_sample_fetch(dev);
     if(rc!=0) {
       data->error_code= IMU_FETCH_FAILED;
       return -1;
     }
    
     rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
     if(rc!=0) {
       data->error_code= IMU_FETCH_FAILED;
       return -1;
     }

      rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
     if(rc!=0) {
      data->error_code= IMU_FETCH_FAILED;
      return -1;
    }

    for (int i = 0; i < 3; i++) {
      data->accel_offset[i] += (sensor_value_to_double(&accel[i]) - true_acc[i]);
      data->gyro_offset[i] += (sensor_value_to_double(&gyro[i]) - true_gyro[i]);
    }
    k_sleep(K_MSEC(1));
  }
  data->error_code= IMU_OK;
  for (int i = 0; i < 3; i++) {
    data->accel_offset[i] = data->accel_offset[i] / 1000.0;
    data->gyro_offset[i] = data->gyro_offset[i] / 1000.0;
  }

  printk("Calibration done\n");
  printk("accel_offset: %f %f %f\n", data->accel_offset[0], data->accel_offset[1],
         data->accel_offset[2]);
  printk("gyroOffset: %f %f %f\n", data->gyro_offset[0], data->gyro_offset[1],
         data->gyro_offset[2]);

  return 0;
}

int process_pitch_roll_mpu(const struct device *dev, struct imu_data *data) {

  struct sensor_value accel[3];
  struct sensor_value gyro[3];

  uint64_t current_time = k_uptime_get();

  double dt = (double)(current_time - data->prev_time) / 1000.0;

  int rc = sensor_sample_fetch(dev);

  if (rc == 0)
    rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
  if (rc == 0)
    rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
  if(rc !=0)
  {
    data->error_code= IMU_FETCH_FAILED;
    return -1;
  }

  data->prev_time = current_time;
  data->accel[0] = sensor_value_to_double(&accel[0]);
  data->accel[1] = sensor_value_to_double(&accel[1]);
  data->accel[2] = sensor_value_to_double(&accel[2]);
  data->gyro[0] = sensor_value_to_double(&gyro[0]) - data->gyro_offset[0];
  data->gyro[1] = sensor_value_to_double(&gyro[1]) - data->gyro_offset[1];
  data->gyro[2] = sensor_value_to_double(&gyro[2]) - data->gyro_offset[2];

  // printk("%03.3f %03.3f %03.3f | %03.3f %03.3f %03.3f\n", data->accel[0],
  // data->accel[1], data->accel[2], data->gyro[0], data->gyro[1], data->gyro[2]);

  if (rc == 0) 
  {
    double pitch_acc = (atan2(-1 * data->accel[0], sqrt(pow(data->accel[1], 2) +
                                                       pow(data->accel[2], 2))));
    double roll_acc = (atan2(-1 * data->accel[1], sqrt(pow(data->accel[2], 2) +
                                                      pow(data->accel[0], 2))));
    data->pitch =k * (data->pitch + (data->gyro[1]) * (dt)) + (1 -k) * pitch_acc;
    data->roll =k * (data->roll + (data->gyro[2]) * (dt)) + (1 - k) * roll_acc;
    data->error_code= IMU_OK;
  } 
  else 
  {
    return 1;
  }
  return 0;
}

