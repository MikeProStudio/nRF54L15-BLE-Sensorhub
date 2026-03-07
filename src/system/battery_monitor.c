#include "battery_monitor.h"
#include "power_control.h"
#include "app_config.h"
#include <zephyr/drivers/adc.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(battery_monitor, LOG_LEVEL_INF);

#define ADC_NODE DT_NODELABEL(adc)
#define ADC_RESOLUTION 12
#define ADC_CHANNEL 7
#define ADC_GAIN ADC_GAIN_1
#define ADC_REFERENCE ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME_DEFAULT

static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);
static battery_update_cb_t update_callback = NULL;

static struct adc_channel_cfg channel_cfg = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id = ADC_CHANNEL,
	.differential = 0,
};

static int16_t sample_buffer;
static struct adc_sequence sequence = {
	.channels = BIT(ADC_CHANNEL),
	.buffer = &sample_buffer,
	.buffer_size = sizeof(sample_buffer),
	.resolution = ADC_RESOLUTION,
};

static uint8_t voltage_to_percentage(float voltage)
{
	if (voltage >= 4.2f) return 100;
	if (voltage <= 3.3f) return 0;
	
	if (voltage >= 4.0f) {
		return (uint8_t)(80 + ((voltage - 4.0f) / 0.2f) * 20);
	} else if (voltage >= 3.7f) {
		return (uint8_t)(40 + ((voltage - 3.7f) / 0.3f) * 40);
	} else {
		return (uint8_t)(((voltage - 3.3f) / 0.4f) * 40);
	}
}

int battery_monitor_init(void)
{
	if (!device_is_ready(adc_dev)) {
		LOG_ERR("ADC device not ready");
		return -ENODEV;
	}
	
	int ret = adc_channel_setup(adc_dev, &channel_cfg);
	if (ret < 0) {
		LOG_ERR("ADC channel setup failed: %d", ret);
		return ret;
	}
	
	LOG_INF("Battery monitor initialized (ADC channel %d)", ADC_CHANNEL);
	return 0;
}

uint8_t battery_monitor_read_level(void)
{
	power_control_vbat_enable(true);
	k_sleep(K_MSEC(5));
	
	int ret = adc_read(adc_dev, &sequence);
	power_control_vbat_enable(false);
	
	if (ret < 0) {
		LOG_ERR("ADC read failed: %d", ret);
		return 0;
	}
	
	int32_t val_mv = sample_buffer;
	ret = adc_raw_to_millivolts(adc_ref_internal(adc_dev), ADC_GAIN, ADC_RESOLUTION, &val_mv);
	if (ret < 0) {
		LOG_ERR("ADC conversion failed: %d", ret);
		return 0;
	}
	
	float voltage = (val_mv / 1000.0f) * 7.0f;
	uint8_t percentage = voltage_to_percentage(voltage);
	
	LOG_DBG("Battery: %.2fV (%u%%)", (double)voltage, percentage);
	return percentage;
}

void battery_monitor_set_callback(battery_update_cb_t callback)
{
	update_callback = callback;
}
