# SmartSolarController
# Solar Charge Controller with ESP32

## Table of Contents
1. [Introduction](#introduction)
2. [Features](#features)
3. [Hardware Requirements](#hardware-requirements)
4. [Software Requirements](#software-requirements)
5. [Installation](#installation)
6. [Usage](#usage)
7. [Configuration](#configuration)
8. [Troubleshooting](#troubleshooting)
9. [Contributing](#contributing)
10. [License](#license)

## Introduction

This project implements a solar charge controller using an ESP32 microcontroller. The controller manages the charging process of a battery from a solar panel, utilizing PWM (Pulse Width Modulation) for efficient charge regulation.

## Features

- Multiple charging stages: Bulk, Absorption, Float
- PWM control for efficient charging
- Real-time monitoring of battery voltage and charging current
- Overcharge, over-discharge, and over-current protection
- Serial output for system status and debugging

## Hardware Requirements

- ESP32 development board
- Solar panel
- Battery (12V lead-acid or similar)
- Voltage divider for battery voltage sensing
- Current sensor (e.g., ACS712)
- MOSFET for PWM control
- Various resistors and capacitors (see schematic)

## Software Requirements

- Arduino IDE
- ESP32 board support package for Arduino

## Installation

1. Clone this repository:
   ```
   git clone https://github.com/yourusername/solar-charge-controller.git
   ```
2. Open the project in Arduino IDE.
3. Install the ESP32 board package in Arduino IDE if you haven't already.
4. Select your ESP32 board from Tools > Board menu.
5. Connect your ESP32 to your computer.
6. Compile and upload the sketch to your ESP32.

## Usage

1. Connect the hardware according to the provided schematic.
2. Power on the system.
3. The controller will automatically start managing the charging process.
4. Monitor the serial output for system status and debugging information.

## Configuration

You can adjust the following parameters in the code to match your specific setup:

- `MAX_BATTERY_VOLTAGE`: Maximum safe battery voltage
- `MIN_BATTERY_VOLTAGE`: Minimum safe battery voltage
- `FLOAT_VOLTAGE`: Float voltage for maintenance charging
- `MAX_CHARGING_CURRENT`: Maximum allowed charging current
- `FLOAT_CURRENT`: Current threshold to enter float stage
- `MIN_CHARGING_CURRENT`: Minimum current to consider active charging

## Troubleshooting

- If the system enters an ERROR state, check all connections and ensure the battery and solar panel voltages are within safe ranges.
- For accurate voltage and current readings, ensure proper calibration of the voltage divider and current sensor.
- If PWM control seems ineffective, verify the MOSFET connections and driving circuit.

## Contributing

Contributions to this project are welcome. Please follow these steps:

1. Fork the repository.
2. Create a new branch for your feature.
3. Commit your changes.
4. Push to the branch.
5. Create a new Pull Request.

## License

This project is licensed under the MIT License - see the LICENSE.md file for details.