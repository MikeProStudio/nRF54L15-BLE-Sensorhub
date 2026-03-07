#include "power_control.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(power_control, LOG_LEVEL_INF);

static const struct gpio_dt_spec sensor_power = GPIO_DT_SPEC_GET(DT_NODELABEL(sensor_power), gpios);
static const struct gpio_dt_spec vbat_enable = GPIO_DT_SPEC_GET(DT_NODELABEL(vbat_enable), gpios);

int power_control_init(void)
{
	if (!gpio_is_ready_dt(&sensor_power)) {
		LOG_ERR("Sensor power GPIO not ready");
		return -ENODEV;
	}
	
	if (!gpio_is_ready_dt(&vbat_enable)) {
		LOG_ERR("VBAT enable GPIO not ready");
		return -ENODEV;
	}
	
	int ret = gpio_pin_configure_dt(&sensor_power, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure sensor power GPIO: %d", ret);
		return ret;
	}
	
	ret = gpio_pin_configure_dt(&vbat_enable, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure VBAT enable GPIO: %d", ret);
		return ret;
	}
	
	LOG_INF("Power control initialized");
	return 0;
}

void power_control_sensors_on(void)
{
	gpio_pin_set_dt(&sensor_power, 1);
	LOG_DBG("Sensors powered ON");
}

void power_control_sensors_off(void)
{
	gpio_pin_set_dt(&sensor_power, 0);
	LOG_DBG("Sensors powered OFF");
}

void power_control_vbat_enable(bool enable)
{
	gpio_pin_set_dt(&vbat_enable, enable ? 1 : 0);
	LOG_DBG("VBAT enable: %s", enable ? "ON" : "OFF");
}
