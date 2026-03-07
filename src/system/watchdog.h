#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <zephyr/kernel.h>

int watchdog_init(void);
void watchdog_feed(void);

#endif
