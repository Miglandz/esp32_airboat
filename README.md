# ESP32 Airboat Controller

Advanced controller for RC airboats with ESP32, designed specifically for use with ELRS receivers and high-current brushless motors.

## Features

- CRSF protocol communication with ELRS receivers
- GPS-based Return-to-Home functionality
- MPU6050 accelerometer for roll/tilt stabilization
- WS2812B RGB LED status indicators
- High-current monitoring with ACS758 sensor (up to 50A)
- Battery voltage monitoring
- Telemetry feedback to ELRS transmitter
- Multiple LED patterns controllable via RC

## Hardware Requirements

### Main Components
- ESP32 Development Board
- ELRS RC Receiver
- GPS Module (NEO-M8N or similar)
- MPU6050 Accelerometer/Gyro Module
- ACS758 Current Sensor (50A version)
- WS2812B RGB LED Strip
- ESC for Brushless Motor
- Servo for Rudder Control

### Wiring Connections
- **ELRS Receiver**:
  - ELRS TX → ESP32 Pin 16 (RX2)
  - ELRS RX → ESP32 Pin 17 (TX2)
  - 5V
  - GND

- **GPS Module**:
  - GPS TX → ESP32 Pin 32 (RX1)
  - GPS RX → ESP32 Pin 33 (TX1)
  - 5V
  - GND

- **MPU6050**:
  - SDA → ESP32 Pin 25
  - SCL → ESP32 Pin 26
  - 3.3V
  - GND

- **ACS758 Current Sensor**:
  - VCC → 5V
  - GND → GND
  - OUT → ESP32 Pin 34 (ADC)

- **Voltage Divider**:
  - Output → ESP32 Pin 35 (ADC)
  - Use resistors to create ratio specified in config.h

- **WS2812B LEDs**:
  - Data In → ESP32 Pin 21
  - 5V
  - GND

- **ESC**:
  - Signal → ESP32 Pin 18
  - Power from Battery
  - GND

- **Servo**:
  - Signal → ESP32 Pin 19
  - 5V
  - GND

## Setup Instructions

1. Install required libraries:
   - ESP32Servo
   - FastLED
   - TinyGPS++
   - Adafruit_MPU6050
   - Adafruit_Sensor

2. Adjust settings in config.h to match your specific airboat setup

3. Connect hardware according to wiring diagram

4. Upload code to ESP32

## RC Channel Mapping

- CH1: Steering
- CH2: Not used
- CH3: Throttle
- CH4: Not used
- CH5: Arm/Disarm
- CH6: Red LED component
- CH7: Green LED component
- CH8: Blue LED component
- CH9: LED Mode
- CH10: Return-to-Home toggle
- CH11: Stabilization Mode toggle

## Airboat-Specific Notes

- The code includes extra filtering for MPU6050 to handle airboat vibrations
- HEADING_CORRECTION_FACTOR in config.h is increased to improve turning response
- Mount MPU6050 on vibration dampeners to improve readings
- Place GPS module away from motor/ESC to minimize interference
- Mount ACS758 in-line with the main power lead to monitor motor current

## Troubleshooting

- **Poor GPS reception**: Make sure antenna has clear view of sky
- **Erratic stabilization**: Check MPU6050 mounting, increase filtering
- **Incorrect current readings**: Check ACS758_OFFSET value, may need calibration
- **Failed to find MPU6050**: Check I2C wiring
- **LEDs not working**: Verify WS2812 data line has a 330-500 ohm resistor

## License

This project is open source and available under the MIT license.