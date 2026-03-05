@echo off
set ZEPHYR_BASE=C:\ncs\v3.2.3\zephyr
set PYTHONPATH=C:\ncs\toolchains\fd21892d0f\opt\bin\Lib\site-packages
set PATH=C:\ncs\toolchains\fd21892d0f\opt\bin;C:\ncs\toolchains\fd21892d0f\opt\bin\Scripts;%PATH%
west build -b xiao_nrf54l15/nrf54l15/cpuapp -p always