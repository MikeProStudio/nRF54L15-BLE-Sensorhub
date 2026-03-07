#include "ble_service.h"
#include "power_manager.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <bluetooth/services/nus.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ble_service, LOG_LEVEL_INF);

#define BT_UUID_CONTROL_SERVICE_VAL \
	BT_UUID_128_ENCODE(0x22d08ffc, 0x229c, 0x4f6e, 0xbeeb, 0x074e46fd3cc0)

#define BT_UUID_CONTROL_CHAR_VAL \
	BT_UUID_128_ENCODE(0x22d08ffc, 0x229c, 0x4f6e, 0xbeeb, 0x074e46fd3cc1)

#define BT_UUID_CONTROL_SERVICE \
	BT_UUID_DECLARE_128(BT_UUID_CONTROL_SERVICE_VAL)

#define BT_UUID_CONTROL_CHAR \
	BT_UUID_DECLARE_128(BT_UUID_CONTROL_CHAR_VAL)

static struct bt_conn *current_conn = NULL;
static K_MUTEX_DEFINE(conn_mutex);
static K_MUTEX_DEFINE(send_mutex);
static K_SEM_DEFINE(bt_ready_sem, 0, 3);

static uint8_t current_mode = 0x01;

static ssize_t write_control_mode(struct bt_conn *conn,
                                   const struct bt_gatt_attr *attr,
                                   const void *buf, uint16_t len,
                                   uint16_t offset, uint8_t flags)
{
	if (len != 1) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	
	uint8_t mode = *(uint8_t *)buf;
	if (mode > 1) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}
	
	current_mode = mode;
	power_manager_set_mode(mode);
	
	bt_gatt_notify(NULL, attr, &current_mode, sizeof(current_mode));
	
	LOG_INF("Mode switched to: %s", mode ? "Continuous" : "Industry");
	return len;
}

static ssize_t read_control_mode(struct bt_conn *conn,
                                  const struct bt_gatt_attr *attr,
                                  void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &current_mode, sizeof(current_mode));
}

static void control_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	LOG_DBG("Control CCC changed: %u", value);
}

BT_GATT_SERVICE_DEFINE(control_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_CONTROL_SERVICE),
	BT_GATT_CHARACTERISTIC(BT_UUID_CONTROL_CHAR,
		BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
		BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
		read_control_mode, write_control_mode, &current_mode),
	BT_GATT_CCC(control_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1)
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (!err) {
		k_mutex_lock(&conn_mutex, K_FOREVER);
		current_conn = bt_conn_ref(conn);
		k_mutex_unlock(&conn_mutex);
		
		static const struct bt_le_conn_param fast_ci =
			BT_LE_CONN_PARAM_INIT(
				BT_GAP_MS_TO_CONN_INTERVAL(15),
				BT_GAP_MS_TO_CONN_INTERVAL(30),
				0,
				1500);  // 15s timeout for 10s Industry Mode interval
		bt_conn_le_param_update(conn, &fast_ci);
		
		LOG_INF("BLE connected");
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	k_mutex_lock(&conn_mutex, K_FOREVER);
	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
	k_mutex_unlock(&conn_mutex);
	
	LOG_INF("BLE disconnected (reason: %u)", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected
};

static void bt_ready(int err)
{
	if (err) {
		LOG_ERR("BLE init failed: %d", err);
		k_sem_give(&bt_ready_sem);
		return;
	}
	
	LOG_INF("BLE stack ready");
	
	err = bt_nus_init(NULL);
	if (err) {
		LOG_ERR("NUS init failed: %d", err);
		k_sem_give(&bt_ready_sem);
		return;
	}
	
	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising start failed: %d", err);
		k_sem_give(&bt_ready_sem);
		return;
	}
	
	LOG_INF("BLE advertising started");
	
	k_sem_give(&bt_ready_sem);
	k_sem_give(&bt_ready_sem);
	k_sem_give(&bt_ready_sem);
}

int ble_service_init(void)
{
	int err = bt_enable(bt_ready);
	if (err) {
		LOG_ERR("BLE enable failed: %d", err);
		return err;
	}
	
	k_sem_take(&bt_ready_sem, K_FOREVER);
	LOG_INF("BLE service initialized");
	return 0;
}

int ble_service_send_data(const uint8_t *data, uint16_t len)
{
	k_mutex_lock(&conn_mutex, K_FOREVER);
	struct bt_conn *conn = current_conn ? bt_conn_ref(current_conn) : NULL;
	k_mutex_unlock(&conn_mutex);
	
	if (!conn) {
		return -ENOTCONN;
	}
	
	struct bt_conn_info info;
	bool ok = (bt_conn_get_info(conn, &info) == 0) &&
	          (info.state == BT_CONN_STATE_CONNECTED);
	
	int ret = -ENOTCONN;
	if (ok) {
		k_mutex_lock(&send_mutex, K_FOREVER);
		ret = bt_nus_send(conn, data, len);
		k_mutex_unlock(&send_mutex);
	}
	
	bt_conn_unref(conn);
	return ret;
}

struct bt_conn *ble_service_get_connection(void)
{
	k_mutex_lock(&conn_mutex, K_FOREVER);
	struct bt_conn *conn = current_conn ? bt_conn_ref(current_conn) : NULL;
	k_mutex_unlock(&conn_mutex);
	return conn;
}

void ble_service_release_connection(struct bt_conn *conn)
{
	if (conn) {
		bt_conn_unref(conn);
	}
}

struct k_sem *ble_service_get_ready_sem(void)
{
	return &bt_ready_sem;
}
