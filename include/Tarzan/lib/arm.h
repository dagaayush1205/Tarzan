#include <stdint.h>
#include <zephyr/device.h>
#define HIGH_PULSE 1
#define LOW_PULSE 0
#define M_PI 3.14159265358979323846
#define k 0.90 // k here is tau

struct joint {
  double accel[3];
  double gyro[3];
  double pitch;
  double roll;
  uint64_t prev_time;
  double gyro_offset[3];
};
int Stepper_motor_write(const struct stepper_motor *motor, int dir, int pos);

void process_mpu6050(const struct device *dev, struct joint *IMU);

int calibration(const struct device *dev, struct joint *IMU);
