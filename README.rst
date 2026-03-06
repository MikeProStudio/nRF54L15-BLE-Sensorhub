nRF54L15 BLE Sensorhub with Real-time Audio Spectrum Visualization
==============================================================

A high-performance Bluetooth LE sensor hub featuring real-time audio spectrum analysis,
IMU data streaming, and a responsive web visualization interface. Built on the
SeeedStudio XIAO nRF54L15 with Zephyr RTOS and optimized for ultra-low latency
audio processing.

.. contents::
   :local:
   :depth: 2

Features
********

* **Real-time Audio Spectrum Analysis**
  - 32 log-spaced frequency bands (50 Hz – 7.8 kHz)
  - 1024-point FFT with 15.6 Hz/bin resolution
  - Peak hold markers and RMS level meter
  - Smooth 60 FPS rendering with Canvas 2D

* **Live Spectrogram (Waterfall)**
  - Scrolling time-frequency visualization
  - 11-stop color palette: black → purple → blue → cyan → green → yellow → red
  - Optimized pixel-level rendering

* **3-Axis IMU Data**
  - LSM6DS3TR-C accelerometer/gyroscope
  - Real-time line chart with circular buffer
  - 80-sample history window

* **Web Bluetooth Interface**
  - Native WebBluetooth API (no apps required)
  - Auto-reconnect with exponential backoff
  - Frame budget protection prevents cascading slowdowns
  - GPU-accelerated rendering with `will-change` layer promotion

* **Performance Optimizations**
  - Single unified requestAnimationFrame loop (no scheduling contention)
  - Zero-allocation hot paths (pre-built palettes, circular buffers)
  - BLE TX power +8 dBm for maximum range
  - Connection interval 15-30 ms for low latency

Hardware Requirements
*********************

* SeeedStudio XIAO nRF54L15 Sense
* MSM261DGT006 PDM microphone (onboard)
* LSM6DS3TR-C IMU (onboard)
* USB-C cable for power and programming

Software Requirements
*********************

* nRF Connect SDK v3.2.3
* Zephyr RTOS v4.2.99
* Chrome/Edge browser with WebBluetooth support
* West build tool

Configuration
*************

Key firmware settings (src/main.c):

::

    #define AUDIO_SAMPLE_FREQ    16000   // Hz
    #define FFT_SIZE             1024
    #define FFT_BANDS            32
    #define BLE_TX_GAP_MS        50      // Minimum notification interval
    #define UART_INTERVAL_MS     1000    // Debug logging interval

BLE TX power (prj.conf):

::

    CONFIG_BT_CTLR_TX_PWR_PLUS_8=y   // +8 dBm maximum power

Building and Flashing
*********************

Build the firmware:

::

    west build -b xiao_nrf54l15

Flash to device:

::

    west flash

Monitor serial output:

::

    west serial

Web Interface
*************

1. Open ``index.html`` in Chrome or Edge
2. Click "Connect" and select "nRF54L15 Bluetooth LE"
3. Grant Bluetooth permissions
4. Real-time visualization starts automatically

Performance Notes
*****************

The visualization is optimized for rock-solid 60 FPS:

* **Frame Budget**: ~4.4 ms worst case (13.6 ms headroom)
* **GPU Layers**: Separate layers for FFT, waterfall, IMU, matrix rain
* **Dynamic Throttling**: Non-essential rendering skipped on dropped frames
* **BLE Stability**: Mutex-serialized notifications prevent buffer corruption

Troubleshooting
**************

**Connection drops when touching antenna**
- Fixed with +8 dBm TX power (CONFIG_BT_CTLR_TX_PWR_PLUS_8=y)
- Ensure clear line-of-sight between device and computer

**Progressive slowdown with music**
- Frame budget protection automatically sheds load
- Single rAF loop prevents scheduling contention
- Waterfall and matrix rain skipped when behind schedule

**WebBluetooth not available**
- Use Chrome or Edge on desktop/laptop
- Mobile browsers have limited WebBluetooth support
- Ensure HTTPS or localhost for index.html

Technical Details
****************

Audio Processing
- PDM microphone → 16 kHz PCM samples
- Ring buffer with overrun recovery
- CMSIS-DSP FFT (1024 points, real-valued)
- Log-spaced frequency band averaging
- Exponential smoothing with attack/release

BLE Communication
- Nordic UART Service (NUS) UUID: 6e400001-b5a3-f393-e0a9-e50e24dcca9e
- 33-byte packets: 1 status byte + 32 FFT bands
- IMU data: JSON-encoded accelerometer values
- Mutex serialization prevents concurrent transmission

Web Rendering
- Canvas 2D for all charts (no Chart.js dependency)
- requestAnimationFrame synchronization
- CSS `will-change` and `contain` for GPU acceleration
- No `backdrop-filter` (major performance killer)

License
*******

This project builds upon the Zephyr RTOS samples and is provided under the
Apache 2.0 License. See individual source files for details.
