#include "ble_bas.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ble_bas, LOG_LEVEL_INF);

static uint8_t battery_level = 100;

int ble_bas_init(void)
{
	LOG_INF("Battery service initialized");
	return 0;
}

void ble_bas_update_level(uint8_t level)
{
	if (level > 100) {
		level = 100;
	}
	
	battery_level = level;
	bt_bas_set_battery_level(level);
	
	LOG_DBG("Battery level updated: %u%%", level);
}
