#ifndef BLE_SERVICE_H
#define BLE_SERVICE_H

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/conn.h>
#include <stdint.h>

int ble_service_init(void);
int ble_service_send_data(const uint8_t *data, uint16_t len);
struct bt_conn *ble_service_get_connection(void);
void ble_service_release_connection(struct bt_conn *conn);
struct k_sem *ble_service_get_ready_sem(void);

#endif
