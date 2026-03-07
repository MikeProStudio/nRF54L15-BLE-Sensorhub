#ifndef DSP_PROCESSOR_H
#define DSP_PROCESSOR_H

#include <zephyr/kernel.h>
#include <stdint.h>

int dsp_processor_init(void);
void dsp_processor_compute_fft(const int16_t *samples, uint32_t sample_count, uint8_t *output_bands);

#endif
