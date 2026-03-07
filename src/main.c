#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/init.h>
#include <zephyr/logging/log_ctrl.h>

#include "app_config.h"
#include "power_control.h"
#include "pdm_handler.h"
#include "imu_manager.h"
#include "dsp_processor.h"
#include "ble_service.h"
#include "ble_bas.h"
#include "ble_dis.h"
#include "battery_monitor.h"
#include "power_manager.h"
#include "watchdog.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static K_SEM_DEFINE(ble_tx_sem, 0, 1);
static K_SEM_DEFINE(industry_sample_sem, 0, 1);
static atomic_t fft_ready_idx = ATOMIC_INIT(0);
static uint8_t fft_buf[2][FFT_BANDS + 1];

void trigger_industry_sample(void)
{
	k_sem_give(&industry_sample_sem);
}

static int64_t last_uart_imu_time = 0;
static int64_t last_uart_mic_time = 0;
static int64_t last_battery_check = 0;
static uint8_t last_battery_level = 100;

static k_tid_t audio_thread_id;
static k_tid_t sensor_thread_id;
static k_tid_t ble_tx_thread_id;

void audio_thread_suspend(void)
{
	if (audio_thread_id) {
		k_thread_suspend(audio_thread_id);
	}
}

void audio_thread_resume(void)
{
	if (audio_thread_id) {
		k_thread_resume(audio_thread_id);
	}
}

void sensor_thread_suspend(void)
{
	if (sensor_thread_id) {
		k_thread_suspend(sensor_thread_id);
	}
}

void sensor_thread_resume(void)
{
	if (sensor_thread_id) {
		k_thread_resume(sensor_thread_id);
	}
}

static void audio_thread_fn(void)
{
	k_sleep(K_MSEC(100));
	pdm_handler_start();
	
	while (1) {
		watchdog_feed();
		
		void *buf = NULL;
		size_t size = 0;
		int ret = pdm_handler_read(&buf, &size, 150);
		
		if (ret == 0 && buf && size >= BLOCK_SIZE_BYTES) {
			uint8_t bands[FFT_BANDS];
			dsp_processor_compute_fft((int16_t *)buf, BLOCK_SIZE_BYTES / 2, bands);
			
			if (power_manager_get_mode() == MODE_CONTINUOUS) {
				int idx = atomic_get(&fft_ready_idx);
				memcpy(fft_buf[idx], bands, FFT_BANDS);
				atomic_set(&fft_ready_idx, 1 - idx);
				k_sem_give(&ble_tx_sem);
			}
			
			pdm_handler_free(buf);
			
			int64_t now = k_uptime_get();
			if (now - last_uart_mic_time >= UART_INTERVAL_MS) {
				last_uart_mic_time = now;
				
				uint8_t peak_band = 0;
				uint8_t peak_val = 0;
				uint32_t sum_sq = 0;
				
				for (int i = 0; i < FFT_BANDS; i++) {
					if (bands[i] > peak_val) {
						peak_val = bands[i];
						peak_band = i;
					}
					sum_sq += (uint32_t)bands[i] * bands[i];
				}
				
				float rms = sqrtf(sum_sq / (float)FFT_BANDS);
				float peak_db = (peak_val / 255.0f) * 90.0f;
				float rms_db = (rms / 255.0f) * 90.0f;
				
				LOG_INF("MIC: peak_band=%u peak=%.1f dB rms=%.1f dB",
				        peak_band, (double)peak_db, (double)rms_db);
			}
		}
	}
}

static void ble_tx_thread_fn(void)
{
	k_sleep(K_MSEC(100));
	int64_t last_sent = 0;
	
	while (1) {
		watchdog_feed();
		k_sem_take(&ble_tx_sem, K_FOREVER);
		
		int64_t now = k_uptime_get();
		int64_t wait = 50 - (now - last_sent);
		if (wait > 0) { k_sleep(K_MSEC(wait)); }
		
		int ready = (int)atomic_get(&fft_ready_idx);
		int err = ble_service_send_data(fft_buf[ready], FFT_BANDS + 1);
		if (err && err != -ENOTCONN) {
			LOG_WRN("FFT send err %d", err);
		}
		last_sent = k_uptime_get();
	}
}

static void sensor_thread_fn(void)
{
	k_sleep(K_MSEC(100));
	char imu_buf[96];
	
	while (1) {
		watchdog_feed();
		
		struct sensor_value accel[3] = {0};
		int ret = imu_manager_read(accel);
		if (ret < 0) {
			LOG_ERR("IMU read failed: %d", ret);
			k_sleep(K_MSEC(500));
			continue;
		}
		
		double ax = sensor_value_to_double(&accel[0]);
		double ay = sensor_value_to_double(&accel[1]);
		double az = sensor_value_to_double(&accel[2]);
		
		if (power_manager_get_mode() == MODE_CONTINUOUS) {
			snprintf(imu_buf, sizeof(imu_buf),
			         "{\"t\":%u,\"a\":[%.2f,%.2f,%.2f]}",
			         k_uptime_get_32(), ax, ay, az);
			
			int send_ret = ble_service_send_data((uint8_t *)imu_buf, strlen(imu_buf));
			if (send_ret && send_ret != -ENOTCONN) {
				LOG_DBG("IMU BLE send failed: %d", send_ret);
			}
		}
		
		int64_t now = k_uptime_get();
		if (now - last_uart_imu_time >= UART_INTERVAL_MS) {
			last_uart_imu_time = now;
			LOG_INF("IMU: X=%.2f Y=%.2f Z=%.2f", (double)ax, (double)ay, (double)az);
		}
		
		if (now - last_battery_check >= (BATTERY_UPDATE_INTERVAL_SEC * 1000)) {
			last_battery_check = now;
			uint8_t level = battery_monitor_read_level();
			if (abs(level - last_battery_level) >= BATTERY_CHANGE_THRESHOLD) {
				ble_bas_update_level(level);
				last_battery_level = level;
			}
		}
		
		k_sleep(K_MSEC(500));
	}
}

static struct k_thread audio_thread_data;
static struct k_thread ble_tx_thread_data;
static struct k_thread sensor_thread_data;
static struct k_thread industry_thread_data;

static K_THREAD_STACK_DEFINE(audio_thread_stack, 10240);
static K_THREAD_STACK_DEFINE(ble_tx_thread_stack, 2048);
static K_THREAD_STACK_DEFINE(sensor_thread_stack, 4096);
static K_THREAD_STACK_DEFINE(industry_thread_stack, 4096);

static void industry_thread_fn(void)
{
	char buf[256];
	
	while (1) {
		k_sem_take(&industry_sample_sem, K_FOREVER);
		
		LOG_INF("Industry: wake for sample");
		
		power_control_sensors_on();
		k_msleep(100);
		
		struct sensor_value accel[3] = {0};
		imu_manager_read(accel);
		float ax = sensor_value_to_double(&accel[0]);
		float ay = sensor_value_to_double(&accel[1]);
		float az = sensor_value_to_double(&accel[2]);
		
		uint8_t bands[FFT_BANDS] = {0};
		void *audio_buf = NULL;
		size_t audio_size = 0;
		int ret = pdm_handler_read(&audio_buf, &audio_size, 200);
		if (ret == 0 && audio_buf && audio_size >= BLOCK_SIZE_BYTES) {
			dsp_processor_compute_fft((int16_t *)audio_buf, BLOCK_SIZE_BYTES / 2, bands);
		}
		
		power_control_sensors_off();
		
		int len = snprintf(buf, sizeof(buf),
		                   "{\"t\":%u,\"a\":[%.2f,%.2f,%.2f],\"f\":[" ,
		                   k_uptime_get_32(), (double)ax, (double)ay, (double)az);
		
		for (int i = 0; i < FFT_BANDS && len < sizeof(buf) - 10; i++) {
			len += snprintf(buf + len, sizeof(buf) - len, "%s%u", i ? "," : "", bands[i]);
		}
		
		len += snprintf(buf + len, sizeof(buf) - len, "]}");
		
		ble_service_send_data((uint8_t *)buf, len);
		LOG_INF("Industry: sent %d bytes", len);
	}
}

int main(void)
{
	k_msleep(2000);
	
	log_filter_set(NULL, 0, log_source_id_get("bt_att"), LOG_LEVEL_ERR);
	
	LOG_INF("=== nRF54L15 Industrial Sensorhub Starting ===");
	
	if (power_control_init() < 0) {
		LOG_ERR("Power control init failed");
		return -1;
	}
	
	power_control_sensors_on();
	k_msleep(100);
	LOG_INF("Sensors powered on");
	
	if (pdm_handler_init() < 0) {
		LOG_ERR("PDM handler init failed");
		return -1;
	}
	
	if (imu_manager_init() < 0) {
		LOG_ERR("IMU manager init failed");
		return -1;
	}
	
	if (dsp_processor_init() < 0) {
		LOG_ERR("DSP processor init failed");
		return -1;
	}
	
	if (battery_monitor_init() < 0) {
		LOG_ERR("Battery monitor init failed");
		return -1;
	}
	
	if (watchdog_init() < 0) {
		LOG_WRN("Watchdog init failed (continuing without)");
	}
	
	if (ble_bas_init() < 0) {
		LOG_ERR("BLE BAS init failed");
		return -1;
	}
	
	if (ble_dis_init() < 0) {
		LOG_ERR("BLE DIS init failed");
		return -1;
	}
	
	if (ble_service_init() < 0) {
		LOG_ERR("BLE service init failed");
		return -1;
	}
	
	if (power_manager_init() < 0) {
		LOG_ERR("Power manager init failed");
		return -1;
	}
	
	audio_thread_id = k_thread_create(&audio_thread_data, audio_thread_stack,
	                                   K_THREAD_STACK_SIZEOF(audio_thread_stack),
	                                   (k_thread_entry_t)audio_thread_fn,
	                                   NULL, NULL, NULL, 3, 0, K_NO_WAIT);
	
	sensor_thread_id = k_thread_create(&sensor_thread_data, sensor_thread_stack,
	                                    K_THREAD_STACK_SIZEOF(sensor_thread_stack),
	                                    (k_thread_entry_t)sensor_thread_fn,
	                                    NULL, NULL, NULL, 7, 0, K_NO_WAIT);
	
	ble_tx_thread_id = k_thread_create(&ble_tx_thread_data, ble_tx_thread_stack,
	                                    K_THREAD_STACK_SIZEOF(ble_tx_thread_stack),
	                                    (k_thread_entry_t)ble_tx_thread_fn,
	                                    NULL, NULL, NULL, 5, 0, K_NO_WAIT);
	
	k_thread_create(&industry_thread_data, industry_thread_stack,
	                K_THREAD_STACK_SIZEOF(industry_thread_stack),
	                (k_thread_entry_t)industry_thread_fn,
	                NULL, NULL, NULL, 6, 0, K_NO_WAIT);
	
	uint8_t initial_battery = battery_monitor_read_level();
	ble_bas_update_level(initial_battery);
	last_battery_level = initial_battery;
	
	LOG_INF("=== System Ready (Battery: %u%%) ===", initial_battery);
	
	return 0;
}
