# Coffee Roast Monitor

M5Stack ESP32 based coffee roasting monitor with intelligent gas stove fire control recommendations.

## Features

### 🔥 Gas Stove Fire Control
- **6-Level Roast Selection**: Light, Medium-Light, Medium, Medium-Dark, Dark, French Roast
- **Intelligent Fire Recommendations**: Dynamic fire power suggestions based on roasting stage and temperature trends
- **Visual Fire Indicators**: Color-coded fire power display with real-time recommendations
- **Audio Notifications**: Sound alerts for stage changes and critical temperature warnings

### 📊 Temperature Monitoring
- **Real-time Temperature Display**: High-precision readings from KMeterISO sensor
- **15-Minute Rolling Graph**: Visual temperature progression tracking
- **Rate of Rise (RoR) Calculation**: 60-second temperature change analysis
- **Statistics Tracking**: Min/max/average temperature recording

### 📱 Connectivity
- **Bluetooth LE**: Real-time data streaming to mobile devices
- **Web Interface**: Compatible with Web Bluetooth for browser-based monitoring
- **JSON Data Format**: Structured data output for integration with other tools

### 🎯 Roasting Guidance
- **Stage-Based Profiles**: Automated progression through roasting stages
- **Target Temperature Ranges**: Optimal temperature guidance for each roast level
- **Timing Recommendations**: Stage duration suggestions
- **Visual Progress Indicators**: Stage progression display with color-coded status

## Hardware Requirements

- **M5Stack Core ESP32** - Main controller unit
- **M5Unit KMeterISO** - High-precision temperature sensor
- **Gas Stove** - Any standard home gas stove

## Software Dependencies

- PlatformIO
- M5Unified library (v0.2.7+)
- M5Unit-KMeterISO library (v1.0.0+)
- ESP32 BLE Arduino library

## Installation

1. Clone this repository
2. Open in PlatformIO
3. Build and upload to M5Stack Core ESP32
4. Connect KMeterISO sensor via I2C (SDA: Pin 21, SCL: Pin 22)

## Usage

### Basic Operation
1. **Standby Mode**: Press Button C to start monitoring
2. **Running Mode**: Use Button A to cycle through display modes
3. **Clear Data**: Hold Button C for 2 seconds to clear all data
4. **Stop/Reset**: Press Button C to return to standby

### Display Modes
- **Graph Mode**: Real-time temperature graph
- **Stats Mode**: Temperature statistics and data summary
- **RoR Mode**: Rate of Rise analysis and trends
- **Guide Mode**: Roasting guidance with fire recommendations

### Fire Control Recommendations
- **Visual Indicators**: Fire power displayed in color-coded format
- **Audio Alerts**: Beep notifications for fire adjustments
- **Dynamic Adjustment**: Real-time recommendations based on temperature and RoR

## Roasting Stages

1. **Preheat** (強火): Warm roaster to 180-200°C
2. **Charge** (火力OFF): Add beans, temperature drops
3. **Drying** (弱火): Gentle drying phase, 240-360 seconds
4. **Maillard** (中火): Browning reaction begins
5. **First Crack** (弱火): Listen for audible cracks
6. **Development** (極弱火-中火): Flavor development based on roast level
7. **Second Crack** (極弱火-中火): For darker roasts
8. **Finish** (火力OFF): Drop beans and cool immediately

## Fire Power Levels

- **OFF** (火力OFF): No heat
- **極弱火** (Very Low): Minimal heat for fine control
- **弱火** (Low): Gentle heat for development
- **中火** (Medium): Standard roasting heat
- **強火** (High): Quick heating and preheat
- **最大火力** (Maximum): Emergency high heat

## Bluetooth Connectivity

The device broadcasts temperature and roasting data via Bluetooth LE using Nordic UART Service (NUS):
- Service UUID: `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`
- TX Characteristic: `6E400003-B5A3-F393-E0A9-E50E24DCCA9E`

Data format: JSON with temperature, RoR, stage info, and fire recommendations.

## Contributing

This project was developed with assistance from Claude Code. Contributions are welcome!

## License

MIT License - Feel free to modify and distribute.

## Safety Notice

⚠️ **Important**: Always monitor your roasting process visually and use proper ventilation. This device provides guidance only - use your judgment and follow proper coffee roasting safety practices.