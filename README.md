# STM32 Blue Pill Projects

Arduino/PlatformIO projects for STM32 Blue Pill with two main applications:

## Projects

### 1. FOC Motor Control (`foc/`)
BLDC motor control using SimpleFOC library with multiple operation modes:
- Open-loop sensorless control
- Encoder testing and pole pair detection  
- Closed-loop FOC velocity control with PID tuning

**Hardware:** STM32 Blue Pill + SimpleFOC Mini driver + BLDC motor + optional MT6701 encoder

### 2. HC-SR04 Distance Sensor (`hcsr04/`)
Ultrasonic distance measurement with proper 5V tolerance handling for STM32F103.

**Hardware:** STM32 Blue Pill + HC-SR04 sensor + 5V power supply

## Project Structure

```
├── foc/                    # FOC motor control project
│   ├── main.cpp           # FOC firmware with multiple modes
│   └── README.md          # FOC-specific documentation
├── hcsr04/                # HC-SR04 distance sensor project  
│   ├── main.cpp           # Distance sensor firmware
│   └── README.md          # HC-SR04-specific documentation
├── platformio.ini         # Build environments for both projects
└── README.md              # This file
```

## Quick Start

### Build and Upload FOC Project
```bash
# Build FOC firmware
pio run -e bluepill_f103c8

# Upload to Blue Pill via ST-Link
pio run -e bluepill_f103c8 -t upload

# Monitor serial output
pio device monitor -e bluepill_f103c8
```

### Build and Upload HC-SR04 Project  
```bash
# Build HC-SR04 firmware
pio run -e bluepill_hcsr04

# Upload to Blue Pill via ST-Link
pio run -e bluepill_hcsr04 -t upload

# Monitor serial output
pio device monitor -e bluepill_hcsr04
```

## Common Hardware Setup

### Blue Pill Connections
- **Serial Debug:** PA2 (TX2), PA3 (RX2) at 115200 baud
- **Power:** 3.3V logic, GND common
- **Programming:** ST-Link via SWD (SWDIO, SWCLK, GND, 3.3V)

### PlatformIO Environments
- `bluepill_f103c8`: FOC motor control firmware
- `bluepill_hcsr04`: HC-SR04 distance sensor firmware

Each environment compiles only the relevant source files and includes project-specific dependencies.

## Documentation

See the README.md file in each project folder for detailed setup, wiring, and usage instructions:
- `foc/README.md` - Complete FOC setup and tuning guide
- `hcsr04/README.md` - HC-SR04 wiring and 5V tolerance notes

## Requirements

- **PlatformIO** (CLI or IDE)
- **ST-Link** programmer
- **STM32 Blue Pill** (F103C8 recommended for flash size)
- Project-specific hardware (see individual README files)

## License

Educational / personal use. Adjust as needed.
