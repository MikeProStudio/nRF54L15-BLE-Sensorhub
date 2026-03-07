#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nRF54L15_peripheral_uart)

# NORDIC SDK APP START
target_sources(app PRIVATE
  src/main.c
  src/hardware/power_control.c
  src/hardware/pdm_handler.c
  src/hardware/imu_manager.c
  src/processing/dsp_processor.c
  src/bluetooth/ble_service.c
  src/bluetooth/ble_bas.c
  src/bluetooth/ble_dis.c
  src/system/battery_monitor.c
  src/system/power_manager.c
  src/system/watchdog.c
)

target_include_directories(app PRIVATE
  src/
  src/hardware/
  src/processing/
  src/bluetooth/
  src/system/
  src/config/
)

# NORDIC SDK APP END
