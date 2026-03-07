#include "watchdog.h"
#include "app_config.h"
#include <zephyr/drivers/watchdog.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(watchdog, LOG_LEVEL_INF);

#if DT_HAS_COMPAT_STATUS_OKAY(nordic_nrf_wdt)
#define WDT_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(nordic_nrf_wdt)
static const struct device *wdt_dev = DEVICE_DT_GET(WDT_NODE);
static int wdt_channel_id = -1;
#else
static const struct device *wdt_dev = NULL;
#endif

int watchdog_init(void)
{
#if DT_HAS_COMPAT_STATUS_OKAY(nordic_nrf_wdt)
	if (!device_is_ready(wdt_dev)) {
		LOG_WRN("Watchdog device not ready");
		return -ENODEV;
	}
	
	struct wdt_timeout_cfg wdt_config = {
		.flags = WDT_FLAG_RESET_SOC,
		.window.min = 0,
		.window.max = WATCHDOG_TIMEOUT_MS,
		.callback = NULL,
	};
	
	wdt_channel_id = wdt_install_timeout(wdt_dev, &wdt_config);
	if (wdt_channel_id < 0) {
		LOG_ERR("Watchdog install timeout failed: %d", wdt_channel_id);
		return wdt_channel_id;
	}
	
	int ret = wdt_setup(wdt_dev, WDT_OPT_PAUSE_HALTED_BY_DBG);
	if (ret < 0) {
		LOG_ERR("Watchdog setup failed: %d", ret);
		return ret;
	}
	
	LOG_INF("Watchdog initialized (%d ms timeout)", WATCHDOG_TIMEOUT_MS);
	return 0;
#else
	LOG_WRN("Watchdog not available in devicetree");
	return -ENOTSUP;
#endif
}

void watchdog_feed(void)
{
#if DT_HAS_COMPAT_STATUS_OKAY(nordic_nrf_wdt)
	if (wdt_dev && wdt_channel_id >= 0) {
		wdt_feed(wdt_dev, wdt_channel_id);
	}
#endif
}
