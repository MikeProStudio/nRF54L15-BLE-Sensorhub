#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <zephyr/kernel.h>
#include <stdint.h>

typedef void (*battery_update_cb_t)(uint8_t level);

int battery_monitor_init(void);
uint8_t battery_monitor_read_level(void);
void battery_monitor_set_callback(battery_update_cb_t callback);

#endif
