#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <bluetooth/services/nus.h>

#include <zephyr/init.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/drivers/gpio.h>
#include <math.h>
#include <arm_math.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* Bluetooth connection */
static struct bt_conn *current_conn;
static K_MUTEX_DEFINE(conn_mutex);   /* protects current_conn             */
static K_SEM_DEFINE(bt_ready_sem, 0, 3);
static K_SEM_DEFINE(ble_tx_sem, 0, 1); /* audio→BLE TX thread signal        */
static K_MUTEX_DEFINE(ble_send_mutex); /* serialises all bt_nus_send calls    */

/* Sensor & Audio Devices */
const struct device *imu_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);
const struct device *dmic_dev = DEVICE_DT_GET(DT_NODELABEL(pdm20));

/* ---------------------------------------------------------------------------
 * Audio / DMIC Configuration
 *
 * MSM261DGT006 PDM microphone:
 *   - PDM clock range: 1.0 – 3.25 MHz  (typ. 2.4 MHz)
 *   - Clock duty-cycle: 40–60 %
 *   - Single channel (L/R pin tied LOW -> LEFT channel)
 *
 * nrfx_pdm double-buffer scheme:
 *   The hardware driver always needs TWO free slab blocks at any time
 *   (one being filled by DMA, one queued as next).  If the consumer thread
 *   cannot free a block before the third DMA transfer completes the driver
 *   logs "No room in RX queue" and drops the frame.
 *
 * Fix strategy:
 *   1. Increase BLOCK_SIZE so each block covers ~32 ms (512 samples @16 kHz).
 *      This gives the audio thread more time between mandatory reads.
 *   2. Keep BLOCK_COUNT large enough (8) that the slab never starves.
 *   3. Audio thread priority 2 (higher than sensor/BLE threads) so it is
 *      scheduled immediately when a block arrives.
 *   4. Free the slab block BEFORE any heavy computation (FFT/BLE).
 *   5. Accumulate samples into a dedicated ring buffer so FFT always has
 *      exactly FFT_SIZE samples regardless of block boundaries.
 *   6. On read timeout / error: attempt DMIC restart to recover from
 *      hardware overflow state.
 * --------------------------------------------------------------------------- */

/*
 * Sample rate 16000 Hz (proven stable with nrfx_pdm + RC clock on nRF54L15):
 *   - PDM clock needed: 16000 * 64 = 1.024 MHz (well within MSM261DGT006 spec)
 *   - Nyquist ceiling: 8000 Hz
 *   - FFT_SIZE 1024 gives 15.6 Hz/bin resolution (4x better than old 256-point FFT)
 *   - 44100 Hz failed: nrfx_pdm RC clock cannot achieve clean 44.1 kHz ratios
 */
#define AUDIO_SAMPLE_FREQ   16000
#define FFT_SIZE            512      /* 512 samples @ 16 kHz = 32 ms window    */
#define FFT_BINS            (FFT_SIZE / 2)   /* 256 bins, 31.25 Hz/bin          */
#define FFT_BANDS           32       /* 32 log-spaced bands, 33 bytes over BLE  */

#define BYTES_PER_SAMPLE    2

/*
 * One DMA block = exactly one FFT window = 512 samples * 2 = 1024 bytes (32ms).
 * FFT completes in ~2 ms leaving 14 ms headroom per block period.
 * BLOCK_COUNT = 8: 8 kB slab, 8 blocks deep – handles any scheduling jitter.
 */
#define BLOCK_SIZE_BYTES    (FFT_SIZE * BYTES_PER_SAMPLE)
#define BLOCK_COUNT         8

K_MEM_SLAB_DEFINE_STATIC(rx_mem_slab, BLOCK_SIZE_BYTES, BLOCK_COUNT, 4);

static struct pcm_stream_cfg stream_cfg = {
    .pcm_width  = 16,
    .mem_slab   = &rx_mem_slab,
    .pcm_rate   = AUDIO_SAMPLE_FREQ,
    .block_size = BLOCK_SIZE_BYTES,
};

static struct dmic_cfg global_dmic_cfg = {
    .io = {
        .min_pdm_clk_freq = 1000000,
        .max_pdm_clk_freq = 3250000,
        .min_pdm_clk_dc   = 40,
        .max_pdm_clk_dc   = 60,
    },
    .streams = &stream_cfg,
    .channel = {
        .req_num_streams = 1,
        .req_num_chan    = 1,
        .req_chan_map_lo = 0,
    },
};

static int power_init(void)
{
    const struct device *gpio = DEVICE_DT_GET(DT_NODELABEL(gpio1));
    if (gpio && device_is_ready(gpio)) {
        gpio_pin_configure(gpio, 0, GPIO_OUTPUT_ACTIVE);
        k_busy_wait(100000);
    }
    return 0;
}
SYS_INIT(power_init, POST_KERNEL, 50);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, 0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e),
};
static const struct bt_data sd[] = { BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1) };

static void connected(struct bt_conn *conn, uint8_t err) {
	if (!err) {
		k_mutex_lock(&conn_mutex, K_FOREVER);
		current_conn = bt_conn_ref(conn);
		k_mutex_unlock(&conn_mutex);

		/* Request tighter connection interval for stable high-frequency
		 * BLE notifications. Default Chrome CI is ~50ms which leaves
		 * little margin for retransmits under JS load.
		 * 15-30ms CI gives 33-66 connection events/s – much more robust.
		 * Supervision timeout 4s > 2*(1+0)*30ms = 60ms requirement. */
		static const struct bt_le_conn_param fast_ci =
			BT_LE_CONN_PARAM_INIT(
				BT_GAP_MS_TO_CONN_INTERVAL(15),
				BT_GAP_MS_TO_CONN_INTERVAL(30),
				0,
				400);
		bt_conn_le_param_update(conn, &fast_ci);
	}
}
static void disconnected(struct bt_conn *conn, uint8_t reason) {
	k_mutex_lock(&conn_mutex, K_FOREVER);
	if (current_conn) { bt_conn_unref(current_conn); current_conn = NULL; }
	k_mutex_unlock(&conn_mutex);
	/* Drain any pending ble_tx_sem signals so TX thread doesn't burst */
	while (k_sem_take(&ble_tx_sem, K_NO_WAIT) == 0) {}
}
BT_CONN_CB_DEFINE(conn_callbacks) = { .connected = connected, .disconnected = disconnected };

static void bt_ready(int err) {
	if (err) return;
	bt_nus_init(NULL);
	bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	k_sem_give(&bt_ready_sem); k_sem_give(&bt_ready_sem); k_sem_give(&bt_ready_sem);
}

/* ---------------------------------------------------------------------------
 * FFT / Processing state  (all static – no stack pressure)
 * --------------------------------------------------------------------------- */
static arm_rfft_fast_instance_f32 fft_instance;
static float32_t fft_input[FFT_SIZE];
static float32_t fft_output[FFT_SIZE];
static float32_t fft_mag[FFT_BINS];
/* dc_offset / dc_frame_count removed – per-frame exact mean subtraction used instead */

/*
 * Log-spaced band boundary table: 33 entries defining 32 bands.
 * Fs=16000, FFT_SIZE=512 -> 31.25 Hz/bin, bands 78 Hz – 7.4 kHz.
 */
static const uint16_t BAND_BINS[FFT_BANDS + 1] = {
    /*  Strictly increasing – no duplicates.  First 14 bands are 1 bin
     *  each (linear), then transition to wider log-spaced bands.
     *  Computed: 2*(256/2)^(i/32) rounded, then enforced prev+1 min.  */
      2,   3,   4,   5,   6,   7,   8,   9,  10,  11,
     12,  13,  14,  15,  17,  19,  23,  26,  31,  36,
     42,  48,  56,  65,  76,  89, 103, 120, 140, 163,
    189, 220, 256
};

/* ---------------------------------------------------------------------------
 * Double-buffer for FFT → BLE TX hand-off.
 *
 * The audio thread writes into buf[write_idx] then atomically swaps write_idx
 * and signals ble_tx_sem.  The BLE TX thread reads buf[1-write_idx] (the
 * "ready" buffer).  No mutex needed: audio thread always writes to the
 * inactive buffer; BLE TX thread always reads the last completed one.
 * If the BLE thread is busy, the audio thread just overwrites the inactive
 * buffer – a harmless dropped frame, better than stalling.
 * --------------------------------------------------------------------------- */
static uint8_t  fft_buf[2][FFT_BANDS + 1];
static atomic_t fft_ready_idx = ATOMIC_INIT(0); /* index of the READY buffer  */

#define UART_INTERVAL_MS  1000   /* log every 1 s so terminal stays alive */
static int64_t last_uart_imu_time = 0;
static int64_t last_uart_mic_time = 0;

/* ---------------------------------------------------------------------------
 * DMIC helpers
 * dmic_configure_and_start : full init (call once at startup)
 * dmic_restart             : stop + retrigger only (recovery, no reconfigure)
 * --------------------------------------------------------------------------- */
static bool dmic_configure_and_start(void)
{
    global_dmic_cfg.channel.req_chan_map_lo =
        dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);

    if (dmic_configure(dmic_dev, &global_dmic_cfg) < 0) {
        LOG_ERR("dmic_configure failed");
        return false;
    }
    if (dmic_trigger(dmic_dev, DMIC_TRIGGER_START) < 0) {
        LOG_ERR("dmic_trigger START failed");
        return false;
    }
    return true;
}

static bool dmic_restart(void)
{
    dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);
    k_sleep(K_MSEC(20));
    if (dmic_trigger(dmic_dev, DMIC_TRIGGER_START) < 0) {
        LOG_ERR("dmic_trigger START failed on restart");
        return false;
    }
    return true;
}

/* ---------------------------------------------------------------------------
 * Audio Thread  (priority 3)
 *
 * ONLY job: drain PDM queue and compute FFT.
 * NEVER calls bt_nus_send – that is the BLE TX thread's job.
 *
 * Flow per block (32 ms):
 *   1. dmic_read  → get pointer to slab block
 *   2. memcpy samples to local buffer
 *   3. k_mem_slab_free IMMEDIATELY – slab slot back to driver
 *   4. DC remove + Hann window + arm_rfft_fast_f32 + arm_cmplx_mag_f32
 *   5. Map bins → 32 bands → write into inactive fft_buf[write_idx]
 *   6. Flip fft_ready_idx atomically
 *   7. k_sem_give(ble_tx_sem) – wake BLE TX thread (non-blocking)
 *   8. Back to dmic_read in < 5 ms total
 * --------------------------------------------------------------------------- */
static int16_t audio_samples[FFT_SIZE];  /* local copy, freed from slab fast */

static void audio_thread_fn(void)
{
    k_sem_take(&bt_ready_sem, K_FOREVER);

    if (!dmic_dev || !device_is_ready(dmic_dev)) {
        LOG_ERR("DMIC device not ready");
        return;
    }
    if (!dmic_configure_and_start()) {
        LOG_ERR("DMIC start failed");
        return;
    }

    arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);

    uint32_t consecutive_errors = 0;

    while (1) {
        void  *rx_buf = NULL;
        size_t rx_size = 0;

        /* Timeout = 2× block period (64 ms). */
        int ret = dmic_read(dmic_dev, 0, &rx_buf, &rx_size, 64);

        if (ret != 0 || rx_buf == NULL) {
            if (++consecutive_errors >= 3) {
                LOG_WRN("DMIC errors %u, restarting", consecutive_errors);
                dmic_restart();
                consecutive_errors = 0;
            }
            continue;
        }
        consecutive_errors = 0;

        /* STEP 1: copy to local buffer, free slab IMMEDIATELY */
        uint32_t n = (uint32_t)(rx_size / BYTES_PER_SAMPLE);
        if (n > FFT_SIZE) { n = FFT_SIZE; }
        memcpy(audio_samples, rx_buf, n * BYTES_PER_SAMPLE);
        k_mem_slab_free(&rx_mem_slab, rx_buf);   /* <-- slab free, driver unblocked */

        /* STEP 2: Per-frame exact DC removal – guarantees zero DC in FFT input */
        float32_t sum = 0.0f;
        for (uint32_t i = 0; i < n; i++) { sum += (float32_t)audio_samples[i]; }
        float32_t frame_mean = sum / (float32_t)n;

        /* STEP 3: Hann window + DC remove (exact per-frame mean) */
        for (uint32_t i = 0; i < FFT_SIZE; i++) {
            float32_t w = 0.5f * (1.0f - arm_cos_f32(
                2.0f * PI * (float32_t)i / (float32_t)(FFT_SIZE - 1)));
            fft_input[i] = ((float32_t)audio_samples[i] - frame_mean) * w;
        }

        /* STEP 4: FFT + magnitude */
        arm_rfft_fast_f32(&fft_instance, fft_input, fft_output, 0);
        arm_cmplx_mag_f32(fft_output, fft_mag, FFT_BINS);
        /* fft_mag[0] is corrupted: arm_rfft_fast packs DC into [0] and
         * Nyquist into [1], but arm_cmplx_mag treats them as a complex
         * pair → sqrt(DC²+Nyquist²).  Zero it out. */
        fft_mag[0] = 0.0f;

        /* STEP 5: map to 32 log bands → write into INACTIVE buffer */
        int write_idx = 1 - (int)atomic_get(&fft_ready_idx);
        fft_buf[write_idx][0] = 0xFF;
        for (int b = 0; b < FFT_BANDS; b++) {
            int bin_lo = BAND_BINS[b];
            int bin_hi = BAND_BINS[b + 1];
            if (bin_hi > FFT_BINS) { bin_hi = FFT_BINS; }
            if (bin_lo >= bin_hi)  { bin_lo = bin_hi - 1; }
            float32_t m = 0.0f;
            for (int idx = bin_lo; idx < bin_hi; idx++) {
                if (fft_mag[idx] > m) { m = fft_mag[idx]; }
            }
            float32_t db = 20.0f * log10f(m > 1.0f ? m : 1.0f);
            float32_t v  = (db - 30.0f) * (255.0f / 60.0f);
            fft_buf[write_idx][b + 1] = (uint8_t)(v < 0.0f ? 0 : (v > 255.0f ? 255 : v));
        }

        /* MIC terminal logging every 5 s */
        {
            int64_t now_mic = k_uptime_get();
            if (now_mic - last_uart_mic_time >= UART_INTERVAL_MS) {
                last_uart_mic_time = now_mic;
                /* Find peak band and its magnitude */
                int peak_band = 0;
                float32_t peak_val = 0.0f;
                float32_t rms_sum = 0.0f;
                for (int b = 0; b < FFT_BANDS; b++) {
                    float32_t bv = (float32_t)fft_buf[write_idx][b + 1];
                    rms_sum += bv * bv;
                    if (bv > peak_val) { peak_val = bv; peak_band = b; }
                }
                float32_t rms_db = (sqrtf(rms_sum / FFT_BANDS) / 255.0f) * 60.0f + 30.0f;
                float32_t peak_db = (peak_val / 255.0f) * 60.0f + 30.0f;
                LOG_INF("MIC: peak_band=%d peak=%.1f dB rms=%.1f dB",
                        peak_band, (double)peak_db, (double)rms_db);
            }
        }

        /* STEP 6: publish – flip ready index atomically */
        atomic_set(&fft_ready_idx, (atomic_val_t)write_idx);

        /* STEP 7: signal BLE TX thread (give only if not already pending) */
        k_sem_give(&ble_tx_sem);
    }
}

/* ---------------------------------------------------------------------------
 * BLE TX Thread  (priority 5)
 *
 * Sleeps on ble_tx_sem.  When woken, reads the ready buffer and sends over
 * BLE NUS.  bt_nus_send may block briefly – that's fine here, audio thread
 * is completely decoupled.
 * --------------------------------------------------------------------------- */
static void ble_tx_thread_fn(void)
{
    k_sem_take(&bt_ready_sem, K_FOREVER);

    int64_t last_sent = 0;

    while (1) {
        /* Wait for audio thread to signal a new FFT frame */
        k_sem_take(&ble_tx_sem, K_FOREVER);

        /* Enforce minimum 50 ms gap – limits Chrome notification rate to
         * ~20 fps (was 31 fps at 25 ms). Reduces Chrome WebBluetooth
         * event queue pressure by ~35%, preventing main-thread starvation
         * and the resulting BLE supervision timeout disconnect.
         * Sleep BEFORE grabbing the conn ref so we never hold a ref
         * across a sleep window where disconnect could occur.            */
        int64_t now = k_uptime_get();
        int64_t wait = 50 - (now - last_sent);
        if (wait > 0) { k_sleep(K_MSEC(wait)); }

        /* Grab ref AFTER sleep – connection may have dropped during wait */
        k_mutex_lock(&conn_mutex, K_FOREVER);
        struct bt_conn *conn = current_conn ? bt_conn_ref(current_conn) : NULL;
        k_mutex_unlock(&conn_mutex);

        if (conn) {
            /* Verify connection is still established before sending */
            struct bt_conn_info info;
            bool ok = (bt_conn_get_info(conn, &info) == 0) &&
                      (info.state == BT_CONN_STATE_CONNECTED);
            if (ok) {
                int ready = (int)atomic_get(&fft_ready_idx);
                /* Serialise with sensor thread – both threads use bt_nus_send.
                 * BLE TX thread is primary: wait indefinitely for the mutex. */
                k_mutex_lock(&ble_send_mutex, K_FOREVER);
                int err = bt_nus_send(conn, fft_buf[ready], FFT_BANDS + 1);
                k_mutex_unlock(&ble_send_mutex);
                if (err && err != -ENOTCONN) {
                    LOG_WRN("FFT send err %d", err);
                }
                last_sent = k_uptime_get();
            }
            bt_conn_unref(conn);
        }
    }
}

/* ---------------------------------------------------------------------------
 * IMU / Sensor Thread  (priority 7)
 * --------------------------------------------------------------------------- */
static void sensor_thread_fn(void)
{
    k_sem_take(&bt_ready_sem, K_FOREVER);
    if (imu_dev && device_is_ready(imu_dev)) {
        struct sensor_value odr = { .val1 = 52 };
        sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ,
                        SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
    }

    char imu_buf[96];
    while (1) {
        struct sensor_value accel[3] = {0};
        if (imu_dev && device_is_ready(imu_dev)) {
            sensor_sample_fetch(imu_dev);
            sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
        }

        k_mutex_lock(&conn_mutex, K_FOREVER);
        struct bt_conn *imu_conn = current_conn ? bt_conn_ref(current_conn) : NULL;
        k_mutex_unlock(&conn_mutex);

        if (imu_conn) {
            struct bt_conn_info info;
            bool ok = (bt_conn_get_info(imu_conn, &info) == 0) &&
                      (info.state == BT_CONN_STATE_CONNECTED);
            if (ok) {
                snprintf(imu_buf, sizeof(imu_buf),
                         "{\"t\":%u,\"a\":[%.2f,%.2f,%.2f]}",
                         k_uptime_get_32(),
                         sensor_value_to_double(&accel[0]),
                         sensor_value_to_double(&accel[1]),
                         sensor_value_to_double(&accel[2]));
                /* Sensor thread: skip IMU send if BLE TX thread is busy
                 * (K_NO_WAIT trylock). Missing one IMU packet is harmless. */
                if (k_mutex_lock(&ble_send_mutex, K_NO_WAIT) == 0) {
                    bt_nus_send(imu_conn, imu_buf, strlen(imu_buf));
                    k_mutex_unlock(&ble_send_mutex);
                }
            }
            bt_conn_unref(imu_conn);
        }

        int64_t now = k_uptime_get();
        if (now - last_uart_imu_time >= UART_INTERVAL_MS) {
            last_uart_imu_time = now;
            LOG_INF("IMU: X=%.2f Y=%.2f Z=%.2f",
                    sensor_value_to_double(&accel[0]),
                    sensor_value_to_double(&accel[1]),
                    sensor_value_to_double(&accel[2]));
        }

        k_sleep(K_MSEC(500));
    }
}

/* ---------------------------------------------------------------------------
 * Thread definitions
 *   audio_thread:  priority 3 – drains PDM, computes FFT, signals BLE TX
 *   ble_tx_thread: priority 5 – sends FFT packets over BLE NUS
 *   sensor_thread: priority 7 – periodic IMU polling and BLE send
 * --------------------------------------------------------------------------- */
K_THREAD_DEFINE(audio_thread,  10240, audio_thread_fn,    NULL, NULL, NULL, 3, 0, 0);
K_THREAD_DEFINE(ble_tx_thread,  2048, ble_tx_thread_fn,   NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(sensor_thread,  4096, sensor_thread_fn,   NULL, NULL, NULL, 7, 0, 0);

int main(void)
{
    k_msleep(2000); //preventing app crash on startup, preventing lockup on first connect
    /* Suppress bt_att "Not connected" warning – it fires during normal
     * disconnect teardown before our callback runs. Cosmetic only.    */
    log_filter_set(NULL, 0,
                   log_source_id_get("bt_att"), LOG_LEVEL_ERR);
    bt_enable(bt_ready);
    return 0;
}
