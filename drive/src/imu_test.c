#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

#include <stdio.h> 

#include <kyvernitis/lib/kyvernitis.h>

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

static const struct device *const uart_debug = DEVICE_DT_GET(DT_ALIAS(debug_uart)); //debugger

const struct device *const mpu6050 = DEVICE_DT_GET(DT_ALIAS(imu_lower_joint));

static const char *now_str(void) {
	static char buf[16]; // ....HH:MM:SS.MMM
	uint32_t now = k_uptime_get_32(); 
	unsigned int ms = now % MSEC_PER_SEC;
	unsigned int s;
	unsigned int min;
	unsigned int h; 

	now /= MSEC_PER_SEC;
	s = now%60U; 
	now /= 60U; 
	min = now%60U; 
	now /= 60U;
	h = now; 

	sprintf(buf, sizeof(buf), "%u:%02u:%02u.%03u", h, min, s, ms);
	return buf; 
} 

static int process_mpu6050(const struct device *dev) { 
	struct sensor_value accel[3]; 
	struct sensor_value gyro[3];

	int rc = sensor_sample_fetch(dev); 

	if(rc == 0) 
	      rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
	if(rc == 0) 
		rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);

	if(rc == 0) {
		printk(" accel %f %f %f m/s/s\n"
		       " gyro %f %f %f rad/s\n",
		       now_str(), 
		       sensor_value_to_double(&accel[0]),
		       sensor_value_to_double(&accel[1]),
		       sensor_value_to_double(&accel[2]),
		       sensor_value_to_double(&gyro[0]),
		       sensor_value_to_double(&gyro[1]),
		       sensor_value_to_double(&gyro[2]));
	}
	else 
printk("sample fetch/get failed: %d\n", rc); 
	
	return rc;
} 

int main() {

int err;
	// device ready chceks
	if (!device_is_ready(uart_debug)) {
		LOG_ERR("UART device not ready");
	}

	if(err<0) {
		if (err == -ENOTSUP) 
			LOG_ERR("Interrupt-driven UART API support not enabled");
		 
		else if (err == -ENOSYS) 
			LOG_ERR("UART device does not support interrupt-driven API");
		
		else 
			LOG_ERR("Error setting UART callback: %d", err);
		
	}

	if(!device_is_ready(mpu6050)) { 
		printk("Device %s is not ready\n",mpu6050->name);
		return 0;
	}
	// enable uart device for communication
	uart_irq_rx_enable(uart_debug);
	
	LOG_INF("Initialization completed successfully!");
	
	while(true) {
		process_mpu6050(mpu6050);	
	}
}
