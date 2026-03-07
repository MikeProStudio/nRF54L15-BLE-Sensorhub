#include "imu_manager.h"
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(imu_manager, LOG_LEVEL_INF);

static const struct device *imu_dev = DEVICE_DT_GET(DT_NODELABEL(lsm6dso));
static int odr_configured = 0;
static int odr_set = 0;

int imu_manager_init(void)
{
	if (!imu_dev || !device_is_ready(imu_dev)) {
		LOG_ERR("IMU device not ready");
		return -ENODEV;
	}
	
	LOG_INF("IMU manager initialized");
	return 0;
}

int imu_manager_reinit(void)
{
	if (!imu_dev || !device_is_ready(imu_dev)) {
		LOG_ERR("IMU device not ready for reinit");
		return -ENODEV;
	}
	
	/* LSM6DS3TR-C requires 15ms boot time after power-on */
	k_sleep(K_MSEC(15));
	
	/* Reset ODR configuration flag to force re-init */
	odr_configured = 0;
	
	/* Configure ODR to 104 Hz */
	struct sensor_value odr_attr;
	odr_attr.val1 = 104;
	odr_attr.val2 = 0;
	int ret = sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ,
	                          SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr);
	if (ret < 0) {
		LOG_ERR("Failed to set ODR: %d", ret);
		return ret;
	}
	
	/* Configure full-scale range to ±4g */
	struct sensor_value fs_attr;
	fs_attr.val1 = 4;
	fs_attr.val2 = 0;
	ret = sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ,
	                      SENSOR_ATTR_FULL_SCALE, &fs_attr);
	if (ret < 0) {
		LOG_ERR("Failed to set full-scale: %d", ret);
		return ret;
	}
	
	odr_configured = 1;
	LOG_INF("IMU re-initialized (ODR=104Hz, FS=4g)");
	return 0;
}

int imu_reinit(void)
{
	/* LSM6DS3TR-C requires 15ms boot time after power-on */
	k_sleep(K_MSEC(15));
	
	/* Reset odr_set flag - next imu_manager_read() will reconfigure */
	odr_set = 0;
	
	LOG_INF("IMU re-init (lightweight)");
	return 0;
}

void imu_manager_power_cycle_reset(void)
{
	/* Reset ODR flags after sensor power cycle */
	odr_set = 0;
	odr_configured = 0;
}

int imu_manager_start(void)
{
	LOG_DBG("IMU started");
	return 0;
}

int imu_manager_stop(void)
{
	LOG_DBG("IMU stopped");
	return 0;
}

int imu_manager_read(struct sensor_value accel[3])
{
	static int first_read = 1;
	
	if (!odr_set) {
		struct sensor_value odr_attr;
		odr_attr.val1 = 104;
		odr_attr.val2 = 0;
		sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ,
		                SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr);
		odr_set = 1;
	}
	
	if (!imu_dev || !device_is_ready(imu_dev)) {
		return -ENODEV;
	}
	
	int ret = sensor_sample_fetch_chan(imu_dev, SENSOR_CHAN_ACCEL_XYZ);
	if (ret < 0) {
		LOG_ERR("Failed to fetch sample: %d", ret);
		return ret;
	}
	
	ret = sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_X, &accel[0]);
	if (ret < 0) {
		LOG_ERR("Failed to get accel X: %d", ret);
		return ret;
	}
	
	ret = sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Y, &accel[1]);
	if (ret < 0) {
		LOG_ERR("Failed to get accel Y: %d", ret);
		return ret;
	}
	
	ret = sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Z, &accel[2]);
	if (ret < 0) {
		LOG_ERR("Failed to get accel Z: %d", ret);
		return ret;
	}
	
	if (first_read) {
		LOG_INF("IMU first read: X=%d.%06d Y=%d.%06d Z=%d.%06d",
		        accel[0].val1, accel[0].val2,
		        accel[1].val1, accel[1].val2,
		        accel[2].val1, accel[2].val2);
		first_read = 0;
	}
	
	return 0;
}

bool imu_manager_is_ready(void)
{
	return imu_dev && device_is_ready(imu_dev);
}
