#include "dsp_processor.h"
#include "app_config.h"
#include <arm_math.h>
#include <math.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(dsp_processor, LOG_LEVEL_INF);

static arm_rfft_fast_instance_f32 fft_instance;
static float32_t fft_input[FFT_SIZE];
static float32_t fft_output[FFT_SIZE];
static float32_t fft_mag[FFT_BINS];

static const uint16_t BAND_BINS[FFT_BANDS + 1] = {
	2,   3,   4,   5,   6,   7,   8,   9,  10,  11,
	12,  13,  14,  15,  17,  19,  23,  26,  31,  36,
	42,  48,  56,  65,  76,  89, 103, 120, 140, 163,
	189, 220, 256
};

int dsp_processor_init(void)
{
	arm_status status = arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);
	if (status != ARM_MATH_SUCCESS) {
		LOG_ERR("FFT init failed: %d", status);
		return -EINVAL;
	}
	
	LOG_INF("DSP processor initialized (FFT_SIZE=%d)", FFT_SIZE);
	return 0;
}

void dsp_processor_compute_fft(const int16_t *samples, uint32_t sample_count, uint8_t *output_bands)
{
	uint32_t n = sample_count < FFT_SIZE ? sample_count : FFT_SIZE;
	
	float32_t sum = 0.0f;
	for (uint32_t i = 0; i < n; i++) {
		sum += (float32_t)samples[i];
	}
	float32_t frame_mean = sum / (float32_t)n;
	
	for (uint32_t i = 0; i < FFT_SIZE; i++) {
		float32_t w = 0.5f * (1.0f - arm_cos_f32(
			2.0f * PI * (float32_t)i / (float32_t)(FFT_SIZE - 1)));
		fft_input[i] = ((float32_t)samples[i] - frame_mean) * w;
	}
	
	arm_rfft_fast_f32(&fft_instance, fft_input, fft_output, 0);
	arm_cmplx_mag_f32(fft_output, fft_mag, FFT_BINS);
	fft_mag[0] = 0.0f;
	
	/* BLE protocol marker: 0xFF indicates FFT data packet */
	output_bands[0] = 0xFF;
	
	for (int b = 0; b < FFT_BANDS; b++) {
		int bin_lo = BAND_BINS[b];
		int bin_hi = BAND_BINS[b + 1];
		/* Fix: FFT_BINS=256 but valid indices are 0-255 */
		if (bin_hi >= FFT_BINS) { bin_hi = FFT_BINS - 1; }
		if (bin_lo >= bin_hi)  { bin_lo = bin_hi - 1; }
		
		float32_t m = 0.0f;
		/* Use exclusive upper bound to prevent bin overlap between adjacent bands */
		for (int idx = bin_lo; idx < bin_hi; idx++) {
			if (fft_mag[idx] > m) { m = fft_mag[idx]; }
		}
		
		float32_t db = 20.0f * log10f(m > 1.0f ? m : 1.0f);
		float32_t v  = (db - 30.0f) * (255.0f / 60.0f);
		/* Write bands to index 1-32 (index 0 is marker) */
		output_bands[b + 1] = (uint8_t)(v < 0.0f ? 0 : (v > 255.0f ? 255 : v));
	}
}
