# OpenDrones Nav Module — Product Specification

> **Product:** Drone Navigation Module (codename TBD)
> **Company:** OpenDrones (subsidiary of Reynoso Industries)
> **Status:** Design phase — parts on hand, firmware architecture complete
> **Last updated:** May 3, 2026

---

## Product Overview

A compact ESP32-based navigation module for FPV drones that adds GPS-based features to any Betaflight-running flight controller. No special firmware flash needed — plug into UART and go.

**Key Features:**
- **Position Hold** — GPS-locked hover in place
- **Return to Home** — automatic flight back to launch point
- **Waypoint Missions** — pre-loaded GPS coordinate sequences
- **Altitude Hold** — barometric pressure-based altitude lock

**Target Market:** FPV freestyle pilots who want GPS nav features without buying a full autopilot system. Budget-conscious drone builders.

---

## Technical Architecture

### Hardware BOM

| Component | Part | Cost | Source |
|-----------|------|------|--------|
| Microcontroller | XIAO ESP32-S3 | $10 | Amazon ✅ ON HAND |
| GPS Module | HGLRC M100 Pro + QMC5883L compass | $20.99 | Amazon ✅ ON HAND |
| Barometer | BMP280 | $9.99/10 pack | Amazon ✅ ON HAND |
| Battery | 850mAh 6S LiPo | $6.67 | Amazon |
| Resistors | BOJACK 37-values kit | $8.99 | Amazon |
| Multimeter | True RMS with test leads | $9.98 | Amazon |
| **Total BOM** | | **~$55** | |

### Communication Chain

```
RadioMaster TX16S (or TX15)
  Switch SB (3-position) → CH7 (1000/1500/2000)
    ↓ (ELRS 2.4GHz)
ELRS Receiver on drone
  ↓ (UART/Serial)
MicoAir743 Flight Controller (Betaflight)
  ↓ (UART5, MSP at 115200 baud)
Our Nav Module (XIAO ESP32-S3)
  - Reads CH7 value via MSP_RAW_RC
  - When CH7 = 2000: activate position hold
  - Module sends MSP_SET_RAW_RC back to FC
  ↓
Betaflight mixes corrections with pilot input
  ↓
Motors respond
```

### How It Connects to Betaflight

1. **ELRS Receiver** → connects to FC via UART (usually UART2 on MicoAir)
2. **Our Nav Module** → connects to FC via UART5 (MSP at 115200 baud)
3. **Betaflight** reads channel values, our module reads them via MSP

### Switch Mapping (TX16S)

| Switch Position | CH7 Value | Mode |
|----------------|-----------|------|
| UP | ~2000 | Position Hold |
| MID | ~1500 | Waypoint Mission |
| DOWN | ~1000 | Return to Home |

Other switches:
- Switch A (CH5) = Arm/Disarm
- Switch B (CH6) = Angle/Horizon/Acro
- Switch C (CH7) = Nav Mode (our feature)
- Switch D (CH8) = Beeper/VTX

### How Each Feature Works

| Feature | Activation | What Module Does |
|---------|-----------|------------------|
| Position Hold | CH7 > 1750 (switch UP) | Reads GPS, calculates drift, sends roll/pitch corrections via MSP_SET_RAW_RC |
| Return to Home | CH7 < 1200 (switch DOWN) | Calculates bearing to home, sends heading + throttle commands |
| Waypoint Mission | CH7 mid-range (1200-1750) | Follows pre-loaded GPS coordinates sequentially, sends flight commands |
| Altitude Hold | Always on when nav active | Reads barometer, adjusts throttle to maintain altitude |

---

## Firmware Architecture

### Core Loop (20Hz update rate)

```cpp
void loop() {
  // 1. Read channel values from Betaflight
  ChannelData channels = msp.requestRawRC();

  // 2. Check if nav mode is activated
  bool navActive = channels.ch7 > 1750;

  if (navActive) {
    // 3. Read sensors
    GPSData gps = readGPS();
    BaroData baro = readBarometer();
    float heading = readCompass();

    // 4. Calculate corrections
    Corrections corr = calculatePositionHold(gps, baro, heading);

    // 5. Send corrections to Betaflight
    msp.setRawRC(corr.roll, corr.pitch, corr.throttle, corr.yaw);
  }

  delay(50); // 20Hz update rate
}
```

### Key Protocols

- **MSP_RAW_RC (command ID: 105)** — reads current channel values from FC
- **MSP_SET_RAW_RC** — sends roll/pitch/throttle/yaw corrections back to FC
- **UART5 at 115200 baud** — serial connection to Betaflight

### Betaflight Setup Required

Just two things:
1. Enable MSP on UART5 (in Ports tab)
2. Plug in our module to UART5 (TX/RX/5V/GND)

No firmware flash, no special Betaflight build. Our module handles everything.

### WiFi Configuration

- XIAO ESP32-S3 has built-in WiFi
- Phone app connects to module's WiFi hotspot
- Used for: loading waypoint missions, configuring settings, viewing GPS status
- Pre-load waypoints before flight via the app

---

## Compass Comparison (BMP280 vs BME280)

| Feature | BMP280 (ours) | BME280 |
|---------|---------------|--------|
| Pressure | ✅ | ✅ |
| Temperature | ✅ | ✅ |
| Humidity | ❌ | ✅ |
| Cost | $9.99/10 pack | $12.99/2 pack |
| Altitude accuracy | Good | Same |
| Use case | Altitude hold — perfect | Overkill for our needs |

**Decision:** BMP280 is fine. Humidity doesn't affect altitude calculations meaningfully for our use case.

---

## Multimeter Usage

The True RMS multimeter is needed for:
| Task | Function | Why |
|------|----------|-----|
| Check 3.3V vs 5V logic | DC Voltage | Confirm XIAO gets correct voltage |
| Verify GPS module power | DC Voltage | GPS chip needs 3.3V |
| Test UART TX/RX | DC Voltage | Confirm data lines toggling (3.3V pulses) |
| Short circuit check | Continuity (beep) | Before powering on |
| Resistor verification | Resistance (Ω) | Pull-up resistors correct |
| Current draw | DC Current (mA) | Total module power consumption |

---

## Competition / Market

**Why this is different:**
- Most GPS nav requires full autopilot (ArduPilot/PX4) — replaces entire flight stack
- Our module ADDS GPS features to existing Betaflight setup
- No firmware flash needed — plug and play
- Much cheaper than full autopilot systems ($55 vs $150+)
- Targets the "freestyle pilot who occasionally wants GPS" market

**Competitors:**
- Betaflight GPS Rescue (built-in, limited — only RTH, no waypoints or position hold)
- ArduPilot/PX4 (full autopilot — overkill for freestyle)
- DJI GPS module (proprietary, expensive, locked to DJI ecosystem)

---

## Next Steps

1. **Assemble prototype** — wire XIAO + GPS + barometer on breadboard
2. **Write firmware** — ESP32 Arduino/MicroPython, MSP communication
3. **Test on bench** — verify MSP communication with Betaflight
4. **Build phone app** — WiFi config interface for waypoints
5. **Flight test** — position hold first, then RTH, then waypoints
6. **Design PCB** — compact custom board for production
7. **Manufacture** — same process as The Arc (Chinese manufacturer)

---

## Files & References

- Reference diagram: ~/agency/projects/opendrones/reference/nav-module-wiring.png
- Battery research: ~/agency/agents/sage/research/fpv-drone-complete-parts-list.md
- BOM details: from April 28 conversation (RTF document provided by Timothy)
