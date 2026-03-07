#ifndef BLE_BAS_H
#define BLE_BAS_H

#include <zephyr/kernel.h>
#include <stdint.h>

int ble_bas_init(void);
void ble_bas_update_level(uint8_t level);

#endif
