#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>

static const struct pwm_dt_spec servo = PWM_DT_SPEC_GET(DT_NODELABEL(motor1));
static const uint32_t min_pulse = DT_PROP(DT_NODELABEL(motor1), min_pulse);
static const uint32_t max_pulse = DT_PROP(DT_NODELABEL(motor1), max_pulse);


enum direction {
        DOWN,
        UP,
};

int main(void)
{
        uint32_t pulse_width = min_pulse;
//        enum direction dir = UP;
        int ret;
        printk("Servomotor control\n");

        if (!pwm_is_ready_dt(&servo)) {
                printk("Error: PWM device %s is not ready\n", servo.dev->name);
                return 0;
        }

        while (1) {
		pulse_width = min_pulse;
                ret = pwm_set_pulse_dt(&servo, pulse_width);
		while(pulse_width<max_pulse)
		{
			pulse_width+=100;
			pwm_set_pulse_dt(&servo, pulse_width);
			k_sleep(K_MSEC(1000));
		}	
		if (ret < 0) {
                        printk("Error %d: failed to set pulse width\n", ret);
                        return 0;
                }
        }
        return 0;
}

