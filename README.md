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
- **PlatformIO** (CLI and VS Code extension) - see Linux setup below
- **STM32 Blue Pill** (STM32F103C8T6)
- **ST-Link V2** programmer (or compatible)
- Basic understanding of Arduino framework

### Linux Development Environment Setup

Complete setup guide for PlatformIO development on Linux:

#### 1. Install PlatformIO Core

Download and install PlatformIO Core using the official installer:

```bash
# Download the PlatformIO installer
wget -O get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py

# Install PlatformIO Core
python3 get-platformio.py
```

#### 2. Add PlatformIO to PATH

Create local bin directory and symlink PlatformIO executables:

```bash
# Create local bin directory
mkdir -p ~/.local/bin/

# Create symlinks for PlatformIO commands
ln -s ~/.platformio/penv/bin/platformio ~/.local/bin/platformio
ln -s ~/.platformio/penv/bin/pio ~/.local/bin/pio
ln -s ~/.platformio/penv/bin/piodebuggdb ~/.local/bin/piodebuggdb
```

Ensure `~/.local/bin` is in your PATH by adding this to your `~/.bashrc` or `~/.zshrc`:
```bash
export PATH="$HOME/.local/bin:$PATH"
```

#### 3. Configure USB Device Permissions

Install udev rules for STM32 programmers and add user to dialout group:

```bash
# Install PlatformIO udev rules
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules

# Add current user to dialout group for USB device access
sudo usermod -a -G dialout $USER

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

**Note:** Log out and log back in for group membership changes to take effect.

#### 4. Install VS Code Extension

Install the PlatformIO IDE extension in VS Code:
- Open VS Code
- Go to Extensions (Ctrl+Shift+X)
- Search for "PlatformIO IDE"
- Install the official PlatformIO extension

#### 5. Install Serial Communication Tools

For debugging and monitoring serial output:

```bash
# Install CuteCom - GUI serial terminal
sudo apt install cutecom

# Install SerialPlot - real-time data plotting
# Download the latest .deb from: https://github.com/hyOzd/serialplot/releases
# Then install with:
sudo dpkg -i serialplot_*.deb
sudo apt install -f  # Fix any dependency issues
```

#### 6. Verify Installation

Test your setup:

```bash
# Check PlatformIO version
pio --version

# List connected devices
pio device list

# Test project compilation (from this repository)
pio run -e bluepill_foc
```

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

