#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include <zephyr/kernel.h>
#include <stdint.h>

int power_manager_init(void);
void power_manager_set_mode(uint8_t mode);
uint8_t power_manager_get_mode(void);

#endif
