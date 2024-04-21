# Puzzlebot-ESP32-Arduino

## Overview
This project demonstrates motor control using an Arduino on ESP32 microcontroller, integrating motor drivers, encoders, and PID controllers. The control system leverages the Manchester Robotics (MCR) libraries to manage motor and encoder operations effectively.

## Dependencies
The project uses the following libraries provided by Manchester Robotics:
- `MCR2_MotorDriver`
- `MCR2_Encoder`
- `MCR2_PIDController`

Ensure that these libraries are correctly installed in your Arduino library directory before compiling and uploading the code to your microcontroller.

## Hardware Setup
### Pin Configuration
- **Motor Driver Pins**:
  - ESP32: `PWMpin_L=4, Pin A=15, Pin B=18`
  - Arduino: `PWMpin_L=2, Pin A=3, Pin B=4`
- **Encoder Pins**:
  - ESP32: `PinA=34, PinB=36`
  - Arduino: `PinA=18, PinB=19`

### Motor and Encoder Connections
Ensure that your motor and encoder pins are configured according to your board type (ESP32 or Arduino) as mentioned above.

## Software Setup
### Serial Communication
- The code uses `HardwareSerial SerialPC(2)` for communication, assuming UART2. Adjust the RXD and TXD pins according to your board documentation.
  - Default pins: `RXD=16`, `TXD=17`

### Configuration
Adjust the motor and encoder configurations based on your setup:
- Motor rotation sign
- Encoder ticks and gear ratios

## Compilation
Compile the project using the Arduino IDE or compatible toolchain targeting your specific microcontroller (ESP32 or Arduino).

## Usage
Upon uploading, the system will read commands from the Serial interface to control the motors:
- `1`: Move forward
- `2`: Turn right
- `3`: Turn left
- `4`: Make a U-turn
- Other values: Stop the motors

## Acknowledgments
Special thanks to Manchester Robotics for providing the essential libraries used in this project.
