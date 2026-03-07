#include "pdm_handler.h"
#include "app_config.h"
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pdm_handler, LOG_LEVEL_INF);

static const struct device *dmic_dev = DEVICE_DT_GET(DT_NODELABEL(pdm20));
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

static uint32_t consecutive_errors = 0;

int pdm_handler_init(void)
{
	if (!dmic_dev || !device_is_ready(dmic_dev)) {
		LOG_ERR("DMIC device not ready");
		return -ENODEV;
	}
	
	global_dmic_cfg.channel.req_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);
	
	if (dmic_configure(dmic_dev, &global_dmic_cfg) < 0) {
		LOG_ERR("dmic_configure failed");
		return -EIO;
	}
	
	LOG_INF("PDM handler initialized");
	return 0;
}

int pdm_handler_start(void)
{
	consecutive_errors = 0;
	
	if (dmic_trigger(dmic_dev, DMIC_TRIGGER_START) < 0) {
		LOG_ERR("dmic_trigger START failed");
		return -EIO;
	}
	
	LOG_INF("PDM started");
	return 0;
}

int pdm_handler_stop(void)
{
	dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);
	LOG_INF("PDM stopped");
	return 0;
}

int pdm_handler_read(void **buffer, size_t *size, int32_t timeout_ms)
{
	int ret = dmic_read(dmic_dev, 0, buffer, size, timeout_ms);
	
	if (ret != 0 || *buffer == NULL) {
		if (++consecutive_errors >= 3) {
			LOG_WRN("DMIC errors %u, restarting", consecutive_errors);
			pdm_handler_stop();
			k_sleep(K_MSEC(20));
			pdm_handler_start();
			consecutive_errors = 0;
		}
		return -EIO;
	}
	
	consecutive_errors = 0;
	return 0;
}

void pdm_handler_free(void *buffer)
{
	k_mem_slab_free(&rx_mem_slab, buffer);
}
