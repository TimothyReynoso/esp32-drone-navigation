# OpenDrones Nav Module — Wiring Guide

> **Components you have in front of you:**
> 1. XIAO ESP32-S3 (white rectangular board, USB-C connector)
> 2. GPS Module (blue square with "Mi" logo — MicoAir M10)
> 3. BMP280 Barometer (small purple board)
> 4. QMC5883L Compass (small blue board with "555" markings)
> 5. Breadboard + jumper wires + resistor kit
> 6. MAX4466 Microphone (purple board — for audio feedback, optional)

---

## Wiring Connections

### POWER

```
Breadboard 3.3V rail → XIAO 3V3 pin
Breadboard GND rail → XIAO GND pin
Breadboard 3.3V rail → GPS VCC
Breadboard 3.3V rail → BMP280 VCC
Breadboard 3.3V rail → Compass VCC
Breadboard GND rail → GPS GND
Breadboard GND rail → BMP280 GND
Breadboard GND rail → Compass GND
```

### GPS Module → XIAO (UART)

| GPS Pin | XIAO Pin | Wire Color | Purpose |
|---------|----------|------------|---------|
| TX | D7 (RX) | Yellow | GPS data → XIAO |
| RX | D6 (TX) | Orange | XIAO commands → GPS |

### BMP280 Barometer → XIAO (I2C)

| BMP280 Pin | XIAO Pin | Wire Color | Purpose |
|------------|----------|------------|---------|
| SCL | D5 (SCL) | Blue | I2C Clock |
| SDA | D4 (SDA) | Green | I2C Data |

### QMC5883L Compass → XIAO (I2C — shared bus)

| Compass Pin | XIAO Pin | Wire Color | Purpose |
|-------------|----------|------------|---------|
| SCL | D5 (SCL) | Blue | I2C Clock (SAME as BMP280) |
| SDA | D4 (SDA) | Green | I2C Data (SAME as BMP280) |

> **Note:** Both BMP280 and Compass share the same I2C bus. They have different addresses so they coexist on the same SCL/SDA lines.

### Connection to Flight Controller (later)

| XIAO Pin | FC Pin | Purpose |
|----------|--------|---------|
| D1 (TX1) | FC UART5 RX | MSP data to FC |
| D2 (RX1) | FC UART5 TX | MSP data from FC |
| GND | FC GND | Common ground |

---

## XIAO ESP32-S3 Pin Reference

```
        USB-C (top)
    ┌──────────────┐
 D0 │ ●          ● │ 5V
 D1 │ ●          ● │ D10
 D2 │ ●          ● │ D9
 D3 │ ●          ● │ D8
 D4 │ ●          ● │ D7
 D5 │ ●          ● │ D6
    │              │
    │   [ESP32]    │
    │              │
 3V3│ ●          ● │ GND
 RST│ ●          ● │ BAT+
    └──────────────┘
```

**Pins we're using:**
- **D4, D5** — I2C (SDA, SCL) for BMP280 + Compass
- **D6, D7** — UART for GPS (D6=TX, D7=RX)
- **D1, D2** — UART1 for MSP to Flight Controller (later)
- **3V3** — Power output
- **GND** — Ground

---

## Step-by-Step Assembly

### Step 1: Power Rails
1. Place XIAO on breadboard (straddle the center gap)
2. Connect USB-C to XIAO for power (or use breadboard power supply)
3. Run jumper wires from XIAO 3V3 → breadboard + rail
4. Run jumper wires from XIAO GND → breadboard - rail

### Step 2: Wire GPS
1. Place GPS module on breadboard
2. GPS VCC → 3.3V rail
3. GPS GND → GND rail
4. GPS TX → XIAO D7
5. GPS RX → XIAO D6

### Step 3: Wire BMP280
1. Place BMP280 on breadboard
2. BMP280 VCC → 3.3V rail
3. BMP280 GND → GND rail
4. BMP280 SCL → XIAO D5
5. BMP280 SDA → XIAO D4

### Step 4: Wire Compass
1. Place compass on breadboard
2. Compass VCC → 3.3V rail
3. Compass GND → GND rail
4. Compass SCL → XIAO D5 (same wire as BMP280 SCL)
5. Compass SDA → XIAO D4 (same wire as BMP280 SDA)

### Step 5: Verify
1. Check all connections with multimeter (continuity test)
2. Verify no shorts between 3.3V and GND
3. Power on via USB-C
4. Check GPS LED starts blinking (acquiring satellites)
5. Upload test firmware to verify each sensor responds

---

## Power Budget

| Component | Current Draw |
|-----------|-------------|
| XIAO ESP32-S3 | ~80mA (active) |
| GPS Module | ~25mA |
| BMP280 | ~2.7µA (ultra low) |
| QMC5883L | ~1mA |
| **Total** | **~106mA** |

USB-C from computer or power bank is plenty for bench testing.
