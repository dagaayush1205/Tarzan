/*
 * Source file for R25
 */

#include <zephyr/sys/reboot.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/crc.h>
#include <zephyr/drivers/uart.h>
#include <string.h>
#include <kyvernitis/lib/kyvernitis.h>

/* msg size in relation to cobs serialization */
#define UART_MSG_SIZE (sizeof(struct mother_msg) + 2)

/* queue to store uart messages */
K_MSGQ_DEFINE(uart_msgq, sizeof(struct mother_msg), 10, 1);

/* DT spec for uart */
static const struct device *const uart_dev = DEVICE_DT_GET(DT_ALIAS(usart1));
uint8_t c;
void serial_cb(const struct device *dev, void *user_data)
{
	if (!uart_irq_update(uart_dev)) {
		return;
	}

	if (!uart_irq_rx_ready(uart_dev)) {
		return;
	}

	/* read until FIFO empty */
	while (uart_fifo_read(uart_dev, &c, 1) == 1) {
		struct mother_msg msg;

		if (c == 0x00 && rx_buf_pos > 0) {
			// uart_fifo_read(uart_dev, &c, 1);
			/* terminate the message with 0x00 */
			rx_buf[rx_buf_pos] = 0;

			if (rx_buf_pos != (sizeof(struct mother_msg) + 2 - 1)) {
				rx_buf_pos = 0;
				continue;
			}
			// Add deserialization guard
			deserialize(rx_buf, (uint8_t *)&msg, sizeof(msg));

			/* if queue is full, message is silently dropped */
			k_msgq_put(&uart_msgq, &msg, K_NO_WAIT);

			/* reset the buffer (it was copied to the msgq) */
			rx_buf_pos = 0;
		} else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
			rx_buf[rx_buf_pos++] = c;
		}

		if (rx_buf_pos > sizeof(struct mother_msg) + 2) {
			rx_buf_pos = 0;
			continue;
		}
	}
}

int main()
{

	int err;

	/* Device ready checks */

	if (!device_is_ready(uart_dev)) {
		log_uart(T_MOTHER_ERROR, "UART device not ready");
	}

	/* Calibrate and Configure devices */
	err = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);

	if (err < 0) {
		if (err == -ENOTSUP) {
			log_uart(T_MOTHER_ERROR, "Interrupt-driven UART API support not enabled");
		} else if (err == -ENOSYS) {
			log_uart(T_MOTHER_ERROR,
				 "UART device does not support interrupt-driven API");
		} else {
			log_uart(T_MOTHER_ERROR, "Error setting UART callback: %d", err);
		}
	}
	log_uart(T_MOTHER_INFO, "Data: %d",c);
	uart_irq_rx_enable(uart_dev);
}
