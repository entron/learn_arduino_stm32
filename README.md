# Arduino STM32 Learning Project

This is a simple Arduino-style project for the STM32 Blue Pill (STM32F103C6) microcontroller using PlatformIO.

## Hardware

- **Board**: STM32 Blue Pill (STM32F103C6)
- **Upload Protocol**: ST-Link
- **Framework**: Arduino

## Project Description

This project demonstrates basic serial communication with the STM32 Blue Pill. The firmware performs the following operations:

1. **Serial Communication Setup**: Initializes serial communication at 115200 baud rate
2. **Startup Message**: Sends a "Blue Pill is alive!" message on startup
3. **Uptime Monitoring**: Continuously prints the system uptime in milliseconds every second

## Code Overview

### Main Functions

- `setup()`: 
  - Initializes serial communication at 115200 baud
  - Waits 2 seconds for serial monitor connection
  - Prints startup confirmation message

- `loop()`:
  - Prints current uptime using `millis()` function
  - Delays for 1 second between updates

## Dependencies

- **Simple FOC Library**: Version 2.3.5 (configured but not used in current code)

## Building and Uploading

### Prerequisites

- PlatformIO Core or PlatformIO IDE
- ST-Link programmer/debugger

### Commands

```bash
# Build the project
platformio run

# Upload to the board
platformio run --target upload

# Monitor serial output
platformio device monitor
```

## Serial Output

### Hardware Connection for Serial Communication

To read the serial output from the Blue Pill, you need to connect it to your computer via a USB-to-Serial adapter:

1. **Connect USB-to-Serial Adapter**:
   - VCC (3.3V) → Blue Pill 3.3V pin
   - GND → Blue Pill GND pin
   - RX → Blue Pill PA9 (TX pin)
   - TX → Blue Pill PA10 (RX pin)

2. **Alternative**: If your Blue Pill has a built-in USB connector, you can use it directly after enabling USB CDC in the code.

### Reading Serial Output

**Using CuteCom (Linux)**:
```bash
# Install CuteCom
sudo apt install cutecom

# Launch CuteCom
cutecom
```

In CuteCom:
- Select the correct device (usually `/dev/ttyUSB0` or `/dev/ttyACM0`)
- Set baud rate to `115200`
- Set data bits to `8`, parity to `None`, stop bits to `1`
- Click "Open device"

**Using PlatformIO Monitor**:
```bash
platformio device monitor
```

**Expected Output**:
When connected to a serial monitor at 115200 baud, you should see output similar to:

```
Blue Pill is alive!
Uptime: 3000 ms
Uptime: 4000 ms
Uptime: 5000 ms
...
```

## Configuration

The project is configured in `platformio.ini` with the following settings:

- **Platform**: ST STM32
- **Board**: Blue Pill F103C6
- **Framework**: Arduino
- **Upload Protocol**: ST-Link
- **Monitor Speed**: 115200 baud
- **Library Archive**: Disabled

## Getting Started

1. **Hardware Setup**:
   - Connect your STM32 Blue Pill to your ST-Link programmer
   - Connect the ST-Link to your computer via USB
   - Connect a USB-to-Serial adapter to the Blue Pill for serial communication (see Serial Output section above)

2. **Software Setup**:
   - Build and upload the firmware using PlatformIO
   - Open CuteCom or another serial monitor to view the output

## Troubleshooting

- Ensure ST-Link drivers are properly installed
- Verify the Blue Pill is properly connected to the ST-Link
- Check USB-to-Serial adapter connections (TX/RX pins)
- Verify the correct COM port/device is selected in the serial monitor (`/dev/ttyUSB0`, `/dev/ttyACM0`, etc.)
- Confirm the baud rate is set to 115200
- On Linux, you may need to add your user to the `dialout` group: `sudo usermod -a -G dialout $USER`
