# ESP32 Drone Navigation

Custom firmware for autonomous drone navigation using ESP32 microcontroller. Implements UBX binary GPS protocol parsing, MSP (MultiWii Serial Protocol) flight controller communication, and a WiFi dashboard for real-time monitoring.

## Features
- UBX Binary Protocol Parser - direct parsing from u-blox GPS modules (no NMEA overhead)
- - Waypoint navigation - autonomous flight between coordinates
  - - Position hold - GPS-locked hover with PID correction
    - - MSP Protocol - full MultiWii Serial Protocol implementation
      - - Real-time telemetry - attitude, altitude, battery, GPS data
        - - WiFi Dashboard - live position on map, telemetry graphs, mobile-responsive
         
          - ## Hardware
          - - MCU: ESP32 (d# ESP32 Drone Navigation

Custom firmware for autonomous drone navigation using ESP32 microcontroller. Implements UBX binary GPS protocol parsing, MSP (MultiWii Serial Protocol) flight controller communication, and a WiFi dashboard for real-time monitoring.

## Features
- UBX Binary Protocol Parser - direct parsing from u-blox GPS modules (no NMEA overhead)
- Waypoint navigation - autonomous flight between coordinates
- Position hold - GPS-locked hover with PID correction
- MSP Protocol - full MultiWii Serial Protocol implementation
- Real-time telemetry - attitude, altitude, battery, GPS data
- WiFi Dashboard - live position on map, telemetry graphs, mobile-responsive

## Hardware
- MCU: ESP32 (dual-core, WiFi/BLE)
- GPS: u-blox NEO-M8N
- Flight Controller: Betaflight/INAV (MSP-compatible)

## Tech Stack
- Language: C++ (Arduino framework)
- GPS Protocol: UBX binary (u-blox proprietary)
- FC Protocol: MSP (MultiWii Serial Protocol v2)
- WiFi: AsyncWebServer (ESP-IDF)
- Build: PlatformIO

## Results
- Sub-100ms GPS lock-to-data latency
- UBX parsing eliminated NMEA string overhead (60% bandwidth reduction)
- Binary protocol allows direct memory mapping (no string parsing)

## License
MIT
