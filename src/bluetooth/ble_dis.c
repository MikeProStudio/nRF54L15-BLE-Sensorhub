#include "ble_dis.h"
#include <string.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ble_dis, LOG_LEVEL_INF);

#define MANUFACTURER_NAME "Seeed Studio"
#define MODEL_NUMBER      "XIAO nRF54L15 Sense"
#define FIRMWARE_VERSION  "1.0.0-industrial"
#define HARDWARE_REVISION "Rev A"

static ssize_t read_manufacturer(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                  void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, MANUFACTURER_NAME, strlen(MANUFACTURER_NAME));
}

static ssize_t read_model(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                           void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, MODEL_NUMBER, strlen(MODEL_NUMBER));
}

BT_GATT_SERVICE_DEFINE(dis,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_DIS),
	BT_GATT_CHARACTERISTIC(BT_UUID_DIS_MANUFACTURER_NAME,
		BT_GATT_CHRC_READ,
		BT_GATT_PERM_READ,
		read_manufacturer, NULL, NULL),
	BT_GATT_CHARACTERISTIC(BT_UUID_DIS_MODEL_NUMBER,
		BT_GATT_CHRC_READ,
		BT_GATT_PERM_READ,
		read_model, NULL, NULL),
);

int ble_dis_init(void)
{
	LOG_INF("Device Information Service initialized");
	return 0;
}
