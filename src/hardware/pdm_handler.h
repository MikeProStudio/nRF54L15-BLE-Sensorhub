#ifndef PDM_HANDLER_H
#define PDM_HANDLER_H

#include <zephyr/kernel.h>
#include <zephyr/audio/dmic.h>

int pdm_handler_init(void);
int pdm_handler_start(void);
int pdm_handler_stop(void);
int pdm_handler_read(void **buffer, size_t *size, int32_t timeout_ms);
void pdm_handler_free(void *buffer);

#endif
