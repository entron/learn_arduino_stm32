# Learn Arduino Libraries on STM32 Blue Pill

Learning repository for testing Arduino libraries on STM32 microcontrollers using PlatformIO. The STM32 Blue Pill serves as an affordable development platform to explore Arduino ecosystem compatibility with ARM Cortex-M processors.

This repo contains practical examples demonstrating how popular Arduino libraries work on STM32 hardware:

## Learning Projects

### 1. SimpleFOC Library Test (`foc/`)
**Arduino Library:** [SimpleFOC](https://simplefoc.com/) - Advanced motor control library
**Learning Goals:** Test Arduino-based FOC library on STM32, explore real-time control loops
**Features Tested:**
- Open-loop sensorless control
- Encoder integration and pole pair detection  
- Closed-loop FOC velocity control with PID tuning
- Arduino IDE compatibility vs native STM32 HAL performance

**Hardware:** STM32 Blue Pill + SimpleFOC Mini driver + BLDC motor + optional MT6701 encoder

### 2. NewPing/HC-SR04 Library Test (`hcsr04/`)
**Arduino Library:** HC-SR04 ultrasonic sensor libraries
**Learning Goals:** Test sensor libraries on STM32, handle 5V/3.3V level conversion
**Features Tested:**
- Arduino-style sensor interfacing on STM32
- Timing-critical operations compatibility
- 5V tolerance and signal conditioning

**Hardware:** STM32 Blue Pill + HC-SR04 sensor + 5V power supply

## Repository Structure

```
├── foc/                    # SimpleFOC Arduino library test
│   ├── main.cpp           # FOC library implementation examples
│   └── README.md          # Setup guide and learning notes
├── hcsr04/                # HC-SR04 sensor Arduino library test
│   ├── main.cpp           # Sensor library compatibility test
│   └── README.md          # Wiring guide and 5V handling notes
├── images/                # Setup photos and scope captures
├── platformio.ini         # Multi-environment build config
└── README.md              # This learning guide
```

## Getting Started

### Prerequisites
- **PlatformIO** (CLI or VS Code extension)
- **STM32 Blue Pill** (STM32F103C8T6 recommended)
- **ST-Link V2** programmer (or compatible)
- Basic understanding of Arduino framework

### Test SimpleFOC Library
```bash
# Build SimpleFOC test firmware
pio run -e bluepill_foc

# Upload to Blue Pill via ST-Link
pio run -e bluepill_foc -t upload

# Monitor Arduino-style Serial output
pio device monitor -e bluepill_foc
```

### Test HC-SR04 Sensor Libraries
```bash
# Build sensor library test firmware
pio run -e bluepill_hcsr04

# Upload to Blue Pill via ST-Link
pio run -e bluepill_hcsr04 -t upload

# Monitor sensor readings
pio device monitor -e bluepill_hcsr04
```

## Learning Resources

Each project folder contains detailed learning documentation:
- `foc/README.md` - SimpleFOC library integration, performance analysis, Arduino vs STM32 HAL comparison
- `hcsr04/README.md` - Sensor library compatibility, timing analysis, 5V tolerance handling


## License

Educational and learning purposes. Feel free to experiment, modify, and share your findings!
