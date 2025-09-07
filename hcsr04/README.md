# HC-SR04 Ultrasonic Distance Sensor for STM32 Blue Pill

Simple distance measurement using HC-SR04 ultrasonic sensor with STM32 Blue Pill.

## Hardware Requirements

| Component | Notes |
|-----------|-------|
| MCU | STM32 Blue Pill (F103C8) |
| Sensor | HC-SR04 ultrasonic distance sensor |
| Power Supply | 5V for HC-SR04, 3.3V for Blue Pill |
| Serial Debug | USB-TTL adapter on PA2/PA3 |

## Wiring

| HC-SR04 Pin | Blue Pill Pin | Notes |
|-------------|---------------|-------|
| VCC | 5V | External 5V supply |
| GND | GND | Common ground |
| TRIG | PA4 | Trigger output (3.3V is fine) |
| ECHO | PA5 | Echo input (see 5V tolerance notes) |

## Wiring Details

| HC-SR04 Pin | Blue Pill Pin | Notes |
|-------------|---------------|-------|
| VCC | 5V | External 5V supply |
| GND | GND | Common ground |
| TRIG | PA4 | Trigger output (3.3V is fine) |
| ECHO | PA5 | Echo input (see 5V tolerance notes) |

### Serial Debug
| USB-TTL | Blue Pill |
|---------|-----------|
| RX | PA2 (TX2) |
| TX | PA3 (RX2) |
| GND | GND |

## 5V Tolerance Details

**STM32F103 5V-tolerant pins:** Many GPIO pins are 5V-tolerant when configured as digital inputs, including PA0–PA10 and PB0–PB15.

**Not 5V-tolerant:** PC13–PC15, PA11/PA12 (USB), NRST, VBAT, and any pin in analog (ADC) mode.

**Power Options:**
- **5V VCC (typical):** Power HC‑SR04 from 5V for maximum range and reliability. ECHO output will be 5V.
- **3.3V VCC (some newer modules):** If your HC‑SR04 supports 3.3V operation, ECHO will be 3.3V and safe on any pin.

**Safety Guidelines:**
- PA5 (used for ECHO) is 5V-tolerant in digital input mode → direct connection OK when using 5V VCC
- If unsure about your pin choice, add a voltage divider (5kΩ/10kΩ) on ECHO
- TRIG signal from MCU is 3.3V and works fine with HC-SR04
- Never apply 5V to a pin in analog (ADC) mode
- Avoid applying 5V when the MCU is unpowered (back-power risk)

**Voltage Divider (if needed):**
```
HC-SR04 ECHO ----[5kΩ]----+----[10kΩ]---- GND
                            |
                         Blue Pill PA5
```
This produces ~3.3V from a 5V signal.

## Usage

### Build and Upload
```bash
# Build HC-SR04 firmware
pio run -e bluepill_hcsr04

# Upload to Blue Pill
pio run -e bluepill_hcsr04 -t upload

# Open serial monitor
pio device monitor -e bluepill_hcsr04
```

### Expected Output
```
HC-SR04 example starting
Wiring: VCC=5V, GND=GND, TRIG=PA4, ECHO=PA5 (FT pin)
Note: On STM32F103, many pins are 5V-tolerant in digital input mode.
If not using an FT pin or if unsure, add a divider on ECHO (e.g. 5k/10k).
# distance_cm
67.91
67.91
67.93
...
```

Notes for SerialPlot
- The firmware now emits a single numeric value per line (distance in cm) which is ideal for SerialPlot.
- A comment header line `# distance_cm` is printed once at startup; SerialPlot treats lines starting with `#` as comments.
- On timeout (no echo), the firmware prints `nan` so SerialPlot will ignore the sample.
- Serial settings: 115200 baud, 8N1. Use the `bluepill_hcsr04` environment when opening the monitor.

Example: open SerialPlot, select the serial port and 115200 baud, then press play — the plot will update at ~10 Hz (100 ms sample interval).

## Configuration

### Timing Settings
- `MEASURE_INTERVAL_MS`: 100ms (measurement frequency)
- `PULSE_TIMEOUT_US`: 30000µs (30ms timeout, ~5m max range)

### Pin Configuration
Change pins by modifying these constants in `main.cpp`:
```cpp
const uint8_t TRIG_PIN = PA4;  // Trigger output
const uint8_t ECHO_PIN = PA5;  // Echo input (5V-tolerant)
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| No serial output | Check PA2/PA3 wiring, use 115200 baud |
| "Timeout: no echo" | Check sensor power, wiring, object in range |
| Inconsistent readings | Ensure stable 5V power, check connections |
| MCU damage concern | Use voltage divider on ECHO if pin not confirmed 5V-tolerant |

## Technical Notes

- Distance calculation: `distance_cm = pulse_duration_us / 58`
- Sound speed: ~343 m/s at room temperature
- Range: 2cm to 400cm (typical HC-SR04 specs)
- Resolution: ~0.3cm based on timing precision
