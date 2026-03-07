#ifndef IMU_MANAGER_H
#define IMU_MANAGER_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>

int imu_manager_init(void);
int imu_manager_reinit(void);
int imu_reinit(void);  // Lightweight re-init for mode switch (Save->Live)
void imu_manager_power_cycle_reset(void);  // Reset ODR flags after power cycle
int imu_manager_start(void);
int imu_manager_stop(void);
int imu_manager_read(struct sensor_value accel[3]);
bool imu_manager_is_ready(void);

#endif
