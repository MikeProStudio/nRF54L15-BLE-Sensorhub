#include "power_manager.h"
#include "power_control.h"
#include "app_config.h"
#include "../hardware/imu_manager.h"
#include "../hardware/pdm_handler.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(power_manager, LOG_LEVEL_INF);

static uint8_t current_mode = MODE_CONTINUOUS;
static K_MUTEX_DEFINE(mode_mutex);
static K_SEM_DEFINE(mode_change_sem, 0, 1);
static struct k_timer industry_timer;

extern void audio_thread_suspend(void);
extern void audio_thread_resume(void);
extern void sensor_thread_suspend(void);
extern void sensor_thread_resume(void);
extern void trigger_industry_sample(void);

static void industry_timer_handler(struct k_timer *timer)
{
	trigger_industry_sample();
}

int power_manager_init(void)
{
	current_mode = MODE_CONTINUOUS;
	k_timer_init(&industry_timer, industry_timer_handler, NULL);
	LOG_INF("Power manager initialized (Continuous mode)");
	return 0;
}

void power_manager_set_mode(uint8_t mode)
{
	if (mode > MODE_CONTINUOUS) {
		LOG_ERR("Invalid mode: %u", mode);
		return;
	}
	
	k_mutex_lock(&mode_mutex, K_FOREVER);
	
	if (mode == current_mode) {
		k_mutex_unlock(&mode_mutex);
		return;
	}
	
	current_mode = mode;
	
	if (mode == MODE_INDUSTRY) {
		LOG_INF("Switching to Industry Mode (Power-Save)");
		pdm_handler_stop();  // Stop PDM before suspending to prevent RX queue overflow
		audio_thread_suspend();
		sensor_thread_suspend();
		power_control_sensors_off();
		k_timer_start(&industry_timer, K_SECONDS(10), K_SECONDS(10));
	} else {
		LOG_INF("Switching to Continuous Mode (Debug)");
		k_timer_stop(&industry_timer);
		power_control_sensors_on();
		k_sleep(K_MSEC(50));
		imu_reinit();
		pdm_handler_start();  // Restart PDM for continuous audio capture
		audio_thread_resume();
		sensor_thread_resume();
	}
	
	k_mutex_unlock(&mode_mutex);
	k_sem_give(&mode_change_sem);
}

uint8_t power_manager_get_mode(void)
{
	k_mutex_lock(&mode_mutex, K_FOREVER);
	uint8_t mode = current_mode;
	k_mutex_unlock(&mode_mutex);
	return mode;
}
