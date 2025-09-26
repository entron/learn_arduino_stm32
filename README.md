# Quick Arduino Library Testing on STM32 Blue Pill

Rapid prototyping repository for testing various modules using Arduino libraries on STM32 microcontrollers with PlatformIO. The STM32 Blue Pill provides an affordable ARM Cortex-M platform for quickly validating Arduino ecosystem libraries without the overhead of writing custom drivers.

This repo contains quick test implementations for various modules using existing Arduino libraries:

## Quick Test Projects

### 1. FOC Motor Control (`foc/`)
**Arduino Library:** [SimpleFOC](https://simplefoc.com/) - Advanced motor control library
**Purpose:** Rapidly test BLDC motor control using existing Arduino FOC library instead of writing custom STM32 drivers
**Quick Tests:**
- Open-loop sensorless control
- Encoder integration and pole pair detection  
- Closed-loop FOC velocity control with PID

**Hardware:** STM32 Blue Pill + SimpleFOC Mini driver + BLDC motor + MT6701 encoder

### 2. Ultrasonic Distance Sensor (`hcsr04/`)
**Arduino Library:** HC-SR04 ultrasonic sensor libraries
**Purpose:** Quick distance sensing implementation using proven Arduino libraries
**Quick Tests:**
- Distance measurement and serial output
- Timing-critical operations on STM32
- 5V/3.3V level handling

**Hardware:** STM32 Blue Pill + HC-SR04 sensor + 5V power supply

## Repository Structure

```
├── foc/                    # SimpleFOC library quick test
│   ├── main.cpp           # FOC motor control implementation
│   └── README.md          # Setup and test results
├── hcsr04/                # HC-SR04 ultrasonic sensor quick test
│   ├── main.cpp           # Distance sensor implementation
│   └── README.md          # Wiring and test results
├── images/                # Setup photos and test results
├── platformio.ini         # Multi-environment build config
└── README.md              # This testing guide
```

## Getting Started

### Prerequisites
- **PlatformIO** (CLI and VS Code extension)
- **STM32 Blue Pill** (STM32F103C8T6)
- **ST-Link V2** programmer (or compatible)
- Basic understanding of Arduino framework

### Quick Test FOC Motor Control
```bash
# Build and upload FOC test
pio run -e bluepill_foc -t upload

# Monitor motor control output
pio device monitor -e bluepill_foc
```

### Quick Test Ultrasonic Sensor
```bash
# Build and upload sensor test
pio run -e bluepill_hcsr04 -t upload

# Monitor distance readings
pio device monitor -e bluepill_hcsr04
```

## Test Documentation

Each project folder contains setup and test results:
- `foc/README.md` - FOC motor control setup, wiring, and test results
- `hcsr04/README.md` - Ultrasonic sensor wiring, configuration, and performance results


## License

Quick prototyping and testing purposes. Feel free to experiment, modify, and share your test results!
