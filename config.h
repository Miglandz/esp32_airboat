#ifndef CONFIG_H
#define CONFIG_H

// Pin definitions
#define CRSF_RX_PIN 16       // ESP32 RX pin connected to ELRS TX
#define CRSF_TX_PIN 17       // ESP32 TX pin connected to ELRS RX
#define ESC_PIN 18           // ESC control pin
#define SERVO_PIN 19         // Servo control pin
#define WS2812_PIN 21        // WS2812 LED data pin
#define GPS_RX_PIN 32        // GPS RX pin
#define GPS_TX_PIN 33        // GPS TX pin
#define SDA_PIN 25           // I2C SDA pin
#define SCL_PIN 26           // I2C SCL pin
#define CURRENT_SENSOR_PIN 34 // ADC pin for ACS758 current sensor
#define VOLTAGE_SENSOR_PIN 35 // ADC pin for voltage divider

// WS2812 LED configuration
#define NUM_LEDS 8           // Number of LEDs in the strip
#define LED_TYPE WS2812B     // LED type
#define COLOR_ORDER GRB      // Color order for the LEDs

// CRSF protocol definitions
#define CRSF_BAUDRATE 420000  // CRSF baud rate
#define CRSF_FRAME_SIZE_MAX 64
#define CRSF_HEADER_OFFSET 1
#define CRSF_TYPE_OFFSET 2
#define CRSF_LENGTH_OFFSET 1
#define CRSF_CRC_OFFSET 0

// CRSF message types
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16
#define CRSF_FRAMETYPE_BATTERY_SENSOR 0x08
#define CRSF_FRAMETYPE_GPS_SENSOR 0x02
#define CRSF_FRAMETYPE_ATTITUDE 0x1E

// CRSF addresses
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8
#define CRSF_ADDRESS_RADIO_TRANSMITTER 0xEA

// CRSF timing
#define CRSF_TIME_BETWEEN_FRAMES_US 950    // gap between CRSF frames

// Channels
#define CRSF_MAX_CHANNEL 16
#define RC_CHANNEL_MIN 172                 // 988us - actual minimum servo position
#define RC_CHANNEL_CENTER 992              // 1500us
#define RC_CHANNEL_MAX 1811                // 2012us - actual maximum servo position

// Channel assignments
#define THROTTLE_CHANNEL 2           // Channel 3
#define STEERING_CHANNEL 0           // Channel 1
#define ARM_CHANNEL 4                // Channel 5 for arming
#define LED_RED_CHANNEL 5            // Channel 6 for red component
#define LED_GREEN_CHANNEL 6          // Channel 7 for green component
#define LED_BLUE_CHANNEL 7           // Channel 8 for blue component
#define LED_MODE_CHANNEL 8           // Channel 9 for LED mode
#define RTH_CHANNEL 9                // Channel 10 for RTH activation
#define STABILIZE_CHANNEL 10         // Channel 11 for stabilization mode

// GPS and RTH settings
#define GPS_BAUDRATE 9600            // GPS baud rate
#define RTH_SPEED 0.3                // RTH speed (0.0-1.0)
#define SIGNAL_TIMEOUT 2000          // Signal timeout in ms before triggering RTH
#define RTH_DISTANCE_THRESHOLD 5.0   // Distance in meters to consider "at home"
#define HEADING_CORRECTION_FACTOR 80 // Steering correction factor for heading (Increased for airboat)

// Accelerometer settings
#define ACC_SAMPLE_RATE 50           // Accelerometer sampling rate in Hz
#define TILT_COMPENSATION_FACTOR 15  // How much to compensate steering for tilt (reduced for airboat)
#define ROLL_THRESHOLD 15.0          // Roll threshold for stability warning in degrees
#define PITCH_THRESHOLD 25.0         // Pitch threshold for stability warning in degrees

// Battery settings
#define BATTERY_CELLS 3              // Number of LiPo cells
#define BATTERY_CAPACITY 5000        // Battery capacity in mAh
#define VOLTAGE_DIVIDER_RATIO 11.0   // Voltage divider ratio (R1+R2)/R2

// Current sensor settings for ACS758
#define ACS758_SENSITIVITY 0.04      // 40mV per Amp for ACS758 50A
#define ACS758_OFFSET 2.5            // Offset voltage is VCC/2 (typically 2.5V for 5V VCC)

#endif // CONFIG_H