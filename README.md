# eCVT - Electronic Continuously Variable Transmission

An ESP32-based control system for an electronic Continuously Variable Transmission (eCVT) developed by NU Baja SAE.

## Overview

This project implements a closed-loop control system for managing the CVT sheave position on a Baja SAE off-road vehicle. The system uses PID control to optimize engine RPM and vehicle performance by automatically adjusting the gear ratio based on sensor inputs.

## Features

- **PID Position Control**: Closed-loop control of sheave position with tunable PID gains
- **Engine RPM Monitoring**: Hall effect sensor input for primary and secondary RPM measurement
- **Wheel Speed Sensing**: Real-time vehicle speed calculation via wheel hall sensor
- **Brake Slam Detection**: Automatic downshift to low gear when brakes are applied suddenly
- **UART Telemetry**: Real-time data logging via serial output for debugging and analysis
- **FreeRTOS Tasks**: Multi-threaded operation for responsive control

## Hardware

- **Microcontroller**: ESP32 DOIT DevKit V1
- **Motor Driver**: PWM-controlled DC motor driver for sheave actuation
- **Sensors**:
  - Potentiometer for sheave position feedback
  - Rotary encoder for precise position tracking
  - Hall effect sensors for engine primary/secondary RPM
  - Hall effect sensor for wheel speed
  - Analog brake pedal sensor

## Pin Configuration

| Pin | Function |
|-----|----------|
| 32 | Potentiometer (POT_PIN) |
| 13 | PWM Output (PWM_PIN) |
| 26 | Direction Control (DIRECTION_PIN) |
| 33 | Primary Hall Sensor |
| 23 | Secondary Hall Sensor |
| 4 | Wheel Speed Hall Sensor |
| 27/14 | Encoder A/B |
| 39 | Brake Sensor |
| 16/17 | Debug UART RX/TX |

## Getting Started

### Prerequisites

- [PlatformIO](https://platformio.org/) IDE or CLI
- USB cable for ESP32 programming

### Building and Uploading

1. Clone the repository:
   ```bash
   git clone https://github.com/NU-Baja-SAE/ecvt.git
   cd ecvt
   ```

2. Build the project:
   ```bash
   pio run
   ```

3. Upload to ESP32:
   ```bash
   pio run --target upload
   ```

4. Monitor serial output:
   ```bash
   pio device monitor
   ```

## Dependencies

- [ESP32Encoder](https://github.com/madhephaestus/ESP32Encoder) v0.11.7 - Rotary encoder library
- [ESP32PulseCounter](https://github.com/mike-gofton/ESP32PulseCounter) v0.2.0 - Pulse counting for RPM measurement

## Project Structure

```
├── src/
│   ├── main.cpp          # Entry point and setup
│   ├── pid.cpp/h         # PID control loop implementation
│   ├── motor.cpp/h       # Motor driver control
│   ├── potentiometer.cpp/h # Sheave position sensing
│   ├── pulseCounter.cpp/h  # Engine RPM measurement
│   ├── wheelSpeed.cpp/h    # Vehicle speed calculation
│   ├── pedal_sensors.cpp/h # Brake pedal input handling
│   └── pins.h            # Pin definitions and macros
├── platformio.ini        # PlatformIO configuration
└── old_code/             # Archived development code
```

## License

This project is developed by Northwestern University Baja SAE team.