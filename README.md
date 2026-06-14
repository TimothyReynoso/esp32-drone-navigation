# OpenDrones Nav Module

Autonomous drone navigation system built on the **Seeed XIAO ESP32-S3**. Provides real-time telemetry, GPS waypoint navigation, and flight controller communication through a browser-based dashboard - no ground station software required.

## Dashboard

The Nav Module hosts a WiFi web server with a real-time telemetry dashboard. Open a browser on any device on the same network to view:

| Card | Data |
|------|------|
| **GPS** | Latitude/longitude, satellite count (up to 27), altitude, speed, fix type (3D), UBX packet counters |
| **Compass** | Visual compass rose with heading, raw XYZ magnetometer values, calibration trigger |
| **Barometer** | Temperature (C/F), pressure (hPa), altitude (m/ft) via BMP280 sensor |
| **Flight Controller** | Betaflight arming status, firmware version, roll/pitch/yaw orientation, MSP TX/RX counters |
| **RC Inputs** | Live PWM/PPM/SBUS channel values - throttle, roll, pitch, yaw, arm (CH5), nav (CH7) |
| **Navigation** | Autonomous mode status - drift, nav throttle, altitude hold, idle indicator |
| **System** | Uptime, heap memory, I2C device scan, connected web clients |

**Design:** Glassmorphism UI with frosted-glass cards on a dark gradient background. Fully responsive - works on phones, tablets, and desktops.

## Features

- **UBX Binary GPS Parser** - Direct binary parsing from u-blox NEO-M8N modules. No NMEA string overhead - packets are memory-mapped directly from the UART buffer
- **MSP Protocol** - Full MultiWii Serial Protocol (v2) implementation for two-way communication with Betaflight/INAV flight controllers
- **Waypoint Navigation** - Autonomous flight between coordinates with PID correction
- **Position Hold** - GPS-locked hover using multi-sensor fusion (GPS + barometer + compass)
- **USB Host Mode** - Standalone operation without a companion computer; the ESP32-S3 acts as USB host to the flight controller
- **I2C Sensor Bus** - BMP280 barometer + QMC5883P compass polled via I2C with automatic device detection
- **Real-time WebSocket Telemetry** - Dashboard updates at 10Hz with zero-page-reload streaming

## Hardware

| Component | Model | Interface |
|-----------|-------|----------|
| **MCU** | Seeed XIAO ESP32-S3 | - |
| **GPS** | u-blox NEO-M8N | UART (UBX binary) |
| **Barometer** | Bosch BMP280 | I2C |
| **Compass** | QMC5883P | I2C |
| **Flight Controller** | Betaflight (BTFL) / INAV | UART (MSP) or USB Host |

The ESP32-S3 piggybacks on the drone frame, connecting to the flight controller via a wiring loom. Tested with Betaflight v1.46+.

## Tech Stack

- **Language:** C++ (Arduino framework)
- **MCU:** XIAO ESP32-S3 (ESP32-S3, dual-core 240MHz, 512KB SRAM, WiFi/BLE)
- **GPS Protocol:** UBX binary (u-blox proprietary - 60% less bandwidth than NMEA)
- **FC Protocol:** MSP (MultiWii Serial Protocol v2)
- **Web Server:** AsyncWebServer + WebSocket (ESP-IDF)
- **UI:** Vanilla JS + CSS (glassmorphism, no frameworks, under 50KB total)
- **Build System:** PlatformIO

## Architecture

Data flow: UBX GPS (UART) + BMP280 + QMC5883P (I2C) -> Nav Core (PID + Waypoints) -> MSP to Betaflight FC + WiFi WebSocket -> Browser Dashboard on Phone/Tablet/PC

## Performance

- **GPS lock-to-data latency:** Sub-100ms via UBX binary (vs 300ms+ with NMEA)
- **Bandwidth savings:** 60% reduction vs NMEA string parsing
- **Memory-mapped parsing:** UBX binary packets read directly from UART buffer - no string allocation
- **Dashboard refresh:** 10Hz via WebSocket (100ms updates)
- **Heap usage:** ~45KB free heap during operation on ESP32-S3

## License

MIT
