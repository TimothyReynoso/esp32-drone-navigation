# OpenDrones Nav Module

Autonomous drone navigation system built on the **Seeed XIAO ESP32-S3**. Provides real-time telemetry, GPS waypoint navigation, and flight controller communication through a browser-based dashboard - no ground station software required.

$ DASHBOARD $
The Nav Module hosts a WiFi web server with a real-time telemetry dashboard with cards for GPS (lat/long, satellite count, fix type, UBX packets), Compass (heading, XYZ calibration), Barometer (BMP280 - temp, pressure, altitude), Flight Controller (Betaflight arming status, firmware version, roll/pitch/yaw, MSP counters), RC Inputs (throttle/roll/pitch/yaw/arm/nav PWM channels), Navigation (autonomous mode, drift, altitude hold), and System (uptime, heap, I2C devices, web clients).

Glassmorphism UI with frosted-glass cards on dark gradient. Fully responsive for phones, tablets, and desktops.

$ FEATURES $
- UBX Binary GPS Parser - direct parsing from u-blox NEO-M8N (no NMEA overhead, memory-mapped from UART buffer)
- MSP Protocol - full MultiWii Serial Protocol v2 for Betaflight/INAV communication
- Waypoint Navigation - autonomous flight between coordinates with PID correction
- Position Hold - GPS-locked hover using multi-sensor fusion (GPS + barometer + compass)
- USB Host Mode - standalone operation, ESP32-S3 acts as USB host to flight controller
- I2C Sensor Bus - BMP280 + QMC5883P with automatic device detection
- Real-time WebSocket Telemetry - 10Hz dashboard updates, zero page reloads

$ HARDWARE $
- MCU: Seeed XIAO ESP32-S3 (dual-core 240MHz, 512KB SRAM, WiFi/BLE)
- GPS: u-blox NEO-M8N via UART (UBX binary)
- Barometer: Bosch BMP280 via I2C
- Compass: QMC5883P via I2C
- Flight Controller: Betaflight (BTFL) via UART (MSP) or USB Host (tested v1.46+)

$ TECH STACK $
- Language: C++ (Arduino framework)
- GPS Protocol: UBX binary (u-blox proprietary - 60% less bandwidth than NMEA)
- FC Protocol: MSP (MultiWii Serial Protocol v2)
- Web Server: AsyncWebServer + WebSocket (ESP-IDF)
- UI: Vanilla JS + CSS (glassmorphism, no frameworks, under 50KB)
- Build System: PlatformIO

$ ARCHITECTURE $
Data flow: UBX GPS (UART) + BMP280 + QMC5883P (I2C) -> Nav Core (PID + Waypoints) -> MSP to Betaflight FC + WiFi WebSocket -> Browser Dashboard

$ PERFORMANCE $
- GPS lock-to-data latency: Sub-100ms via UBX binary (vs 300ms+ with NMEA)
- Bandwidth savings: 60% reduction vs NMEA string parsing
- Memory-mapped parsing: UBX binary packets read directly from UART buffer
- Dashboard refresh: 10Hz via WebSocket (100ms updates)
- Heap usage: ~45KB free heap during operation on ESP32-S3

$ LICENSE $
MIT
