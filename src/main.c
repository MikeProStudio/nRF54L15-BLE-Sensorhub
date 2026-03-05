#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <bluetooth/services/nus.h>

#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <math.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* Shared state for Mic RMS */
static double global_mic_rms = 0.0;
static K_MUTEX_DEFINE(mic_rms_mutex);

/* PWM setup */
static const struct pwm_dt_spec pwm_led0 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led0));

/* Bluetooth connection */
static struct bt_conn *current_conn;
static K_SEM_DEFINE(bt_ready_sem, 0, 2);

/* Sensor & Audio Devices */
const struct device *imu_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);
const struct device *dmic_dev = DEVICE_DT_GET(DT_NODELABEL(pdm20));

#define SAMPLE_BIT_WIDTH 16
#define BYTES_PER_SAMPLE sizeof(int16_t)
/* Audio Configuration */
#define AUDIO_SAMPLE_FREQ 16000
#define AUDIO_BLOCK_SIZE  (AUDIO_SAMPLE_FREQ / 10) /* 100ms blocks */
#define BLOCK_SIZE_BYTES (AUDIO_BLOCK_SIZE * BYTES_PER_SAMPLE)
#define BLOCK_COUNT 4

K_MEM_SLAB_DEFINE_STATIC(rx_mem_slab, BLOCK_SIZE_BYTES, BLOCK_COUNT, 4);

/* DMIC static configurations */
static struct pcm_stream_cfg stream_cfg = {
    .pcm_width = SAMPLE_BIT_WIDTH,
    .mem_slab  = &rx_mem_slab,
    .pcm_rate  = AUDIO_SAMPLE_FREQ,
    .block_size = BLOCK_SIZE_BYTES,
};

static struct dmic_cfg global_dmic_cfg = {
    .io = {
        .min_pdm_clk_freq = 1000000,
        .max_pdm_clk_freq = 3500000,
        .min_pdm_clk_dc = 40,
        .max_pdm_clk_dc = 60,
    },
    .streams = &stream_cfg,
    .channel = {
        .req_num_streams = 1,
        .req_num_chan = 1,
        .req_chan_map_lo = 0, /* Set dynamically in init */
    },
};

/* Power Initialization */
static int power_init(void)
{
    const struct device *gpio = DEVICE_DT_GET(DT_NODELABEL(gpio1)); // gpio1 statt gpio0
    if (!device_is_ready(gpio)) {
        return -ENODEV;
    }
    /* P1.00 ist IMU&MIC_3V3_EN auf dem XIAO nRF54L15 */
    gpio_pin_configure(gpio, 0, GPIO_OUTPUT_ACTIVE); 
    
    k_busy_wait(100000); 
    return 0;
}
SYS_INIT(power_init, PRE_KERNEL_2, 50);

/* Advertisement data */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL,
		      0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
		      0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* --- PWM Logic --- */
static void set_pwm_duty(uint32_t percentage)
{
	if (percentage > 100) {
		percentage = 100;
	}
	uint32_t pulse = (pwm_led0.period * percentage) / 100;
	int ret = pwm_set_pulse_dt(&pwm_led0, pulse);
	if (ret < 0) {
		LOG_ERR("Failed to set PWM pulse! (err %d)", ret);
	}
}

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data, uint16_t len)
{
	char str[16];
	uint16_t cp_len = len < sizeof(str) - 1 ? len : sizeof(str) - 1;
	memcpy(str, data, cp_len);
	str[cp_len] = '\0';
	
	/* Trim trailing newlines/carriage returns */
	while (cp_len > 0 && (str[cp_len - 1] == '\n' || str[cp_len - 1] == '\r')) {
		str[cp_len - 1] = '\0';
		cp_len--;
	}

	LOG_INF("Received data: %s", str);

	if (strncmp(str, "ON", 2) == 0) {
		set_pwm_duty(100);
	} else if (strncmp(str, "OFF", 3) == 0) {
		set_pwm_duty(0);
	} else {
		char *endptr;
		long value = strtol(str, &endptr, 10);
		
		if (endptr != str && *endptr == '\0') {
			LOG_INF("Parsed Value: %ld", value);
			if (value >= 0 && value <= 100) {
				set_pwm_duty((uint32_t)value);
			} else {
				LOG_WRN("Value out of range (0-100): %ld", value);
			}
		} else {
			LOG_WRN("Unknown command or parsing failed.");
			LOG_HEXDUMP_WRN(data, len, "Raw RX data");
		}
	}
}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_ERR("Connection failed (err %u)", err);
		return;
	}
	current_conn = bt_conn_ref(conn);
	LOG_INF("Connected");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (reason %u)", reason);
	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static void bt_ready(int err)
{
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return;
	}

	LOG_INF("Bluetooth initialized");

	err = bt_nus_init(&nus_cb);
	if (err) {
		LOG_ERR("Failed to initialize UART service (err: %d)", err);
		return;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return;
	}
	LOG_INF("Advertising successfully started");

	k_sem_give(&bt_ready_sem);
	k_sem_give(&bt_ready_sem);
}

static void audio_thread_fn(void)
{
    k_sem_take(&bt_ready_sem, K_FOREVER);
    LOG_INF("Audio thread starting...");

    while (1) {
        void *rx_buffer;
        size_t rx_size;
        
        int ret = dmic_read(dmic_dev, 0, &rx_buffer, &rx_size, 2000);
        
        if (ret == 0 && rx_buffer != NULL) {
            int16_t *samples = (int16_t *)rx_buffer;
            int num_samples = rx_size / sizeof(int16_t);
            
            if (num_samples > 0 && num_samples <= (BLOCK_SIZE_BYTES / sizeof(int16_t))) {
                int64_t sum = 0;
                for (int i = 0; i < num_samples; i++) {
                    sum += samples[i];
                }
                int16_t mean = sum / num_samples;

                int64_t sum_sq = 0;
                for (int i = 0; i < num_samples; i++) {
                    int32_t val = samples[i] - mean;
                    sum_sq += (int64_t)val * val;
                }
                
                double mic_rms = sqrt((double)sum_sq / num_samples);
                
                k_mutex_lock(&mic_rms_mutex, K_FOREVER);
                global_mic_rms = mic_rms;
                k_mutex_unlock(&mic_rms_mutex);
                
                /* Debug LOG for DMIC data flow (every ~1s) */
                static int log_cnt = 0;
                if (log_cnt++ % 10 == 0) {
                    LOG_DBG("DMIC RMS: %.2f (SAMPLES: %d)", mic_rms, num_samples);
                }
            } else {
                LOG_WRN("Invalid rx_size: %u", rx_size);
            }

            k_mem_slab_free(&rx_mem_slab, rx_buffer);
        } else {
            LOG_ERR("dmic_read failed or timed out: %d", ret);
			k_sleep(K_MSEC(100));
        }
    }
}

K_THREAD_DEFINE(audio_thread, 4096, audio_thread_fn, NULL, NULL, NULL, 5, 0, 0);

static void sensor_thread_fn(void)
{
    /* Warten bis Bluetooth bereit ist */
    k_sem_take(&bt_ready_sem, K_FOREVER);
    LOG_INF("Sensor thread starting...");

    /* IMU Initialisierung */
    if (!device_is_ready(imu_dev)) {
        LOG_ERR("IMU nicht bereit");
    } else {
        struct sensor_value odr_attr = { .val1 = 52, .val2 = 0 }; 
        sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr);
        sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr);
        LOG_INF("IMU konfiguriert (52Hz)");
    }

    int64_t last_log_time = 0;
    
    /* --- HIER WAR DER FEHLER: Die Variable muss definiert sein --- */
    char json_buf[160]; 

    while (1) {
        /* IMU DATEN LESEN */
        struct sensor_value accel[3] = {0};
        if (device_is_ready(imu_dev)) {
            sensor_sample_fetch(imu_dev);
            sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
        }

        /* JSON FORMATIEREN */
        double ax = accel[0].val1 + (double)accel[0].val2 / 1000000.0;
        double ay = accel[1].val1 + (double)accel[1].val2 / 1000000.0;
        double az = accel[2].val1 + (double)accel[2].val2 / 1000000.0;

        double current_mic_rms = 0.0;
        k_mutex_lock(&mic_rms_mutex, K_FOREVER);
        current_mic_rms = global_mic_rms;
        k_mutex_unlock(&mic_rms_mutex);

        /* Hier wird json_buf jetzt korrekt erkannt */
        snprintf(json_buf, sizeof(json_buf), 
            "{\"a\":[%.2f,%.2f,%.2f],\"m\":%.2f}",
            ax, ay, az, current_mic_rms);

        /* ÜBER BLUETOOTH SENDEN */
        if (current_conn) {
            bt_nus_send(current_conn, (uint8_t *)json_buf, strlen(json_buf));
        }

        /* UART Logging alle 2 Sekunden */
        int64_t now = k_uptime_get();
        if (now - last_log_time >= 2000) {
            LOG_INF("IMU: x=%.2f y=%.2f z=%.2f | MIC: RMS=%.2f", ax, ay, az, current_mic_rms);
            last_log_time = now;
        }

        k_sleep(K_MSEC(20));
    }
}

K_THREAD_DEFINE(sensor_thread, 4096, sensor_thread_fn, NULL, NULL, NULL, 5, 0, 0);

int main(void)
{
	int err;

	LOG_INF("Starting nRF54L15 Firmware...");

	if (!device_is_ready(pwm_led0.dev)) {
		LOG_ERR("PWM device %s is not ready!", pwm_led0.dev->name);
	} else {
		LOG_INF("PWM device %s is ready. Initializing to 100%%.", pwm_led0.dev->name);
		set_pwm_duty(100);
	}

	if (!device_is_ready(dmic_dev)) {
		LOG_ERR("MIC nicht bereit");
	} else {
		global_dmic_cfg.channel.req_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);
		int ret = dmic_configure(dmic_dev, &global_dmic_cfg);
		if (ret < 0) {
			LOG_ERR("dmic_configure failed (err %d)", ret);
		} else {
			LOG_INF("MIC konfiguriert");
			dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
		}
	}





	err = bt_enable(bt_ready);
	if (err) {
		LOG_ERR("Bluetooth enable failed (err %d)", err);
		return 0;
	}

	while (1) {
		k_sleep(K_FOREVER);
	}

	return 0;
}
