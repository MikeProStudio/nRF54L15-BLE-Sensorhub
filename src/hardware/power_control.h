#ifndef POWER_CONTROL_H
#define POWER_CONTROL_H

#include <zephyr/kernel.h>

int power_control_init(void);
void power_control_sensors_on(void);
void power_control_sensors_off(void);
void power_control_vbat_enable(bool enable);

#endif
