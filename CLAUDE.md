# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a PlatformIO project for an M5Stack Core ESP32 device that implements a real-time temperature monitoring system using a KMeterISO temperature sensor. The application displays current temperature readings and maintains a 15-minute rolling graph display.

## Development Environment Setup

This project uses PlatformIO for ESP32/Arduino development:

- **Platform**: ESP32 (M5Stack Core)
- **Framework**: Arduino
- **Board**: m5stack-core-esp32
- **Monitor Speed**: 115200 baud

## Common Commands

### Build and Upload
```bash
# Build the project
pio run

# Upload to device
pio run --target upload

# Build and upload in one command
pio run --target upload

# Monitor serial output
pio device monitor

# Build, upload, and monitor
pio run --target upload && pio device monitor
```

### Development
```bash
# Clean build files
pio run --target clean

# Check for available updates
pio pkg update

# List connected devices
pio device list
```

## Code Architecture

### Hardware Configuration
- **I2C Communication**: Uses pins 21 (SDA) and 22 (SCL) at 100kHz
- **Temperature Sensor**: M5Unit KMeterISO with default I2C address
- **Display**: M5Stack LCD with Japanese font support

### Data Flow
1. **Sensor Reading**: KMeterISO sensor provides temperature data via I2C
2. **Data Storage**: Circular buffer (`buf[BUF_SIZE]`) stores 900 readings (15 minutes at 1-second intervals)
3. **Display Update**: Real-time temperature and rolling graph updated every second

### Key Components
- **main.cpp**: Single-file application with setup/loop structure
- **Temperature Scaling**: Raw sensor values divided by 100 for Celsius conversion
- **Graph Rendering**: Fixed range 20-250°C with time-series line plotting
- **Error Handling**: KMeter status checking with error display

### Display Layout
- **Header**: Application title at top
- **Current Value**: Large temperature display below title
- **Graph Area**: 300x160 pixel time-series plot (10,70 to 310,230)
- **Graph Labels**: Temperature range and time scale annotations

## Dependencies

External libraries managed through PlatformIO Library Manager:
- `m5stack/M5Unit-KMeterISO@^1.0.0` - Temperature sensor driver
- `m5stack/M5Unified@^0.2.7` - M5Stack hardware abstraction

## Development Notes

- The application uses a circular buffer pattern for efficient memory usage
- Graph rendering is optimized with startWrite()/endWrite() for LCD performance
- Temperature range is hardcoded (20-250°C) - modify TEMP_MIN/TEMP_MAX constants if different range needed
- Buffer size of 900 provides exactly 15 minutes of history at 1-second intervals