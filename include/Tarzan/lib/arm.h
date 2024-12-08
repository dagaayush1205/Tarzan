#include <stdint.h>
#include <zephyr/device.h>
#define HIGH_PULSE 1
#define LOW_PULSE 0
#define M_PI 3.14159265358979323846
#define k 0.90 // k here is tau

struct joint {
  float accel[3];
  float gyro[3];
  float pitch;
  float roll;
  uint64_t prev_time;
  float gyro_offset[3];
};
int Stepper_motor_write(const struct stepper_motor *motor, int dir, int pos);

int process_mpu6050(const struct device *dev, struct joint *IMU, int n);

int calibration(const struct device *dev, struct joint *IMU);
