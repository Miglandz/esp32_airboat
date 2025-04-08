#include <Arduino.h>
#include <ESP32Servo.h>
#include <FastLED.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

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
#define HEADING_CORRECTION_FACTOR 50 // Steering correction factor for heading

// Accelerometer settings
#define ACC_SAMPLE_RATE 50           // Accelerometer sampling rate in Hz
#define TILT_COMPENSATION_FACTOR 25  // How much to compensate steering for tilt (0-100)
#define ROLL_THRESHOLD 15.0          // Roll threshold for stability warning in degrees
#define PITCH_THRESHOLD 25.0         // Pitch threshold for stability warning in degrees

// Objects
Servo esc;
Servo servo;
HardwareSerial CrsfSerial(1);
HardwareSerial GpsSerial(2);
TinyGPSPlus gps;
CRGB leds[NUM_LEDS];
Adafruit_INA219 ina219;
Adafruit_MPU6050 mpu;

// Variables
uint8_t crsfData[CRSF_FRAME_SIZE_MAX];
uint16_t rcChannels[CRSF_MAX_CHANNEL];
uint32_t lastFrameTime = 0;
uint8_t frameIndex = 0;
bool frameStart = false;
uint8_t frameLength = 0;
bool armed = false;
bool previousArmState = false;

// LED states
uint8_t ledMode = 0;
uint8_t ledBrightness = 255;
CRGB ledColor = CRGB::Blue;
uint8_t redValue = 0;
uint8_t greenValue = 0;
uint8_t blueValue = 0;

// RTH variables
bool rthActive = false;
bool automaticRthActive = false;
bool rthPreviousState = false;
double homeLat = 0.0;
double homeLon = 0.0;
uint32_t lastSignalTime = 0;
bool gpsValid = false;

// Accelerometer variables
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float roll, pitch;
float temperature;
bool stabilizationMode = false;
bool previousStabilizeState = false;
bool accelerometerAvailable = false;

// Telemetry data
float batteryVoltage = 11.8;
float batteryCurrent = 3.5;
float batteryCapacity = 1500; // mAh
int16_t altitude = 0;
int16_t gpsLatitude = 0;
int16_t gpsLongitude = 0;
uint8_t gpsSatellites = 0;
uint16_t speed = 0;
float heading = 0;

// Function prototypes
void setupCrsf();
void setupGps();
void setupINA219();
void setupMPU6050();
void processCrsfFrame(uint8_t *frame, uint8_t frameLength);
void extractChannels(uint8_t *frame);
void handleArming();
void updateOutputs();
void updateLEDs();
void setAllLeds(CRGB color);
void runLedEffect();
void processGPS();
void processAccelerometer();
void checkSignalTimeout();
void handleRTH();
void calculateRTHControls(int &throttle, int &steering);
void applyStabilization(int &steering);
void updateBatteryData();
void sendTelemetry();
void sendBatteryTelemetry();
void sendGpsTelemetry();
void sendAttitudeTelemetry();
uint8_t crc8(const uint8_t *ptr, uint8_t len);

void setup() {
  Serial.begin(115200); // Debug serial
  
  // Initialize I2C with custom pins
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // Initialize INA219 for voltage monitoring
  setupINA219();
  
  // Initialize MPU6050 accelerometer
  setupMPU6050();
  
  // Initialize GPS
  setupGps();
  
  // Initialize CRSF serial
  setupCrsf();
  
  // Initialize ESC and Servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  esc.setPeriodHertz(50); // Standard 50Hz PWM for ESCs
  servo.setPeriodHertz(50); // Standard 50Hz PWM for servos
  
  esc.attach(ESC_PIN, 1000, 2000); // 1000-2000us is standard for ESCs
  servo.attach(SERVO_PIN, 1000, 2000); // 1000-2000us is standard for servos
  
  // Initial position - safe state
  esc.writeMicroseconds(1000); // Minimal throttle
  servo.writeMicroseconds(1500); // Center position
  
  // Initialize WS2812 LEDs
  FastLED.addLeds<LED_TYPE, WS2812_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(ledBrightness);
  
  // Initialize with a startup sequence
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Red;
    FastLED.show();
    delay(50);
  }
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Green;
    FastLED.show();
    delay(50);
  }
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Blue;
    FastLED.show();
    delay(50);
  }
  // Turn off all LEDs
  setAllLeds(CRGB::Black);
  FastLED.show();
  
  // Initialize last signal time
  lastSignalTime = millis();
  
  Serial.println("ESP32 CRSF Controller with GPS RTH and MPU6050 Ready!");
}

void loop() {
  // Process GPS data
  processGPS();
  
  // Process accelerometer data
  processAccelerometer();
  
  // Update battery data from INA219
  updateBatteryData();
  
  // Check for CRSF data
  while (CrsfSerial.available()) {
    uint8_t inChar = CrsfSerial.read();
    
    // Update last signal time
    lastSignalTime = millis();
    
    // Start of a new CRSF frame
    if (frameStart == false && inChar == CRSF_ADDRESS_FLIGHT_CONTROLLER) {
      frameStart = true;
      frameIndex = 0;
      crsfData[frameIndex++] = inChar;
    } 
    // Middle of a CRSF frame
    else if (frameStart == true) {
      crsfData[frameIndex++] = inChar;
      
      // Get the frame length from the second byte
      if (frameIndex == CRSF_LENGTH_OFFSET + 1) {
        frameLength = inChar;
      }
      
      // We've received the entire frame
      if (frameIndex == frameLength + 2) { // +2 for address and length bytes
        frameStart = false;
        processCrsfFrame(crsfData, frameLength + 2);
      }
      
      // Overflow protection
      if (frameIndex >= CRSF_FRAME_SIZE_MAX) {
        frameStart = false;
      }
    }
  }
  
  // Check signal timeout for automatic RTH
  checkSignalTimeout();
  
  // Handle RTH if active
  if (rthActive) {
    handleRTH();
  }
  
  // Update LED effects
  static uint32_t lastLedUpdate = 0;
  if (millis() - lastLedUpdate > 20) { // Update LEDs at 50Hz
    lastLedUpdate = millis();
    runLedEffect();
  }
  
  // Send telemetry at regular intervals (every 100ms)
  static uint32_t lastTelemetryTime = 0;
  if (millis() - lastTelemetryTime > 100) {
    lastTelemetryTime = millis();
    sendTelemetry();
  }
}

void setupCrsf() {
  CrsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);
}

void setupGps() {
  GpsSerial.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS initialized");
}

void setupINA219() {
  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
  } else {
    Serial.println("INA219 initialized");
  }
}

void setupMPU6050() {
  // Try to initialize the MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    accelerometerAvailable = false;
  } else {
    Serial.println("MPU6050 initialized");
    accelerometerAvailable = true;
    
    // Set up the accelerometer
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
}

void processCrsfFrame(uint8_t *frame, uint8_t frameLength) {
  // Check CRC
  uint8_t crc = crc8(&frame[2], frameLength - 3);
  if (crc != frame[frameLength - 1]) {
    Serial.println("CRC error");
    return;
  }
  
  uint8_t frameType = frame[CRSF_TYPE_OFFSET];
  
  // Process different frame types
  switch (frameType) {
    case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
      extractChannels(frame);
      handleArming();
      updateOutputs();
      updateLEDs();
      break;
      
    // Other frame types can be handled here
    
    default:
      // Unknown frame type
      break;
  }
}

void extractChannels(uint8_t *frame) {
  // CRSF channels are packed in 11 bits each
  uint8_t *payload = &frame[3];
  
  // Extract the RC channels
  rcChannels[0] = ((payload[0] | payload[1] << 8) & 0x07FF);
  rcChannels[1] = ((payload[1] >> 3 | payload[2] << 5) & 0x07FF);
  rcChannels[2] = ((payload[2] >> 6 | payload[3] << 2 | payload[4] << 10) & 0x07FF);
  rcChannels[3] = ((payload[4] >> 1 | payload[5] << 7) & 0x07FF);
  rcChannels[4] = ((payload[5] >> 4 | payload[6] << 4) & 0x07FF);
  rcChannels[5] = ((payload[6] >> 7 | payload[7] << 1 | payload[8] << 9) & 0x07FF);
  rcChannels[6] = ((payload[8] >> 2 | payload[9] << 6) & 0x07FF);
  rcChannels[7] = ((payload[9] >> 5 | payload[10] << 3) & 0x07FF);
  rcChannels[8] = ((payload[11] | payload[12] << 8) & 0x07FF);
  rcChannels[9] = ((payload[12] >> 3 | payload[13] << 5) & 0x07FF);
  rcChannels[10] = ((payload[13] >> 6 | payload[14] << 2 | payload[15] << 10) & 0x07FF);
  
  // Check RTH switch
  bool rthState = (rcChannels[RTH_CHANNEL] > RC_CHANNEL_CENTER);
  if (rthState != rthPreviousState) {
    if (rthState) {
      // Toggle RTH mode
      rthActive = !rthActive;
      automaticRthActive = false; // Manual control takes precedence
      
      if (rthActive) {
        Serial.println("Return To Home ACTIVATED (manual)");
      } else {
        Serial.println("Return To Home DEACTIVATED");
      }
    }
    rthPreviousState = rthState;
  }
  
  // Check stabilization mode switch
  bool stabilizeState = (rcChannels[STABILIZE_CHANNEL] > RC_CHANNEL_CENTER);
  if (stabilizeState != previousStabilizeState) {
    if (stabilizeState) {
      // Toggle stabilization mode
      stabilizationMode = !stabilizationMode;
      
      if (stabilizationMode) {
        Serial.println("Stabilization Mode ACTIVATED");
        // Visual confirmation with blue pulse
        setAllLeds(CRGB::Blue);
        FastLED.show();
        delay(300);
      } else {
        Serial.println("Stabilization Mode DEACTIVATED");
      }
    }
    previousStabilizeState = stabilizeState;
  }
  
  // Debug print
  Serial.print("CH1: "); Serial.print(rcChannels[0]);
  Serial.print(" CH2: "); Serial.print(rcChannels[1]);
  Serial.print(" CH3: "); Serial.print(rcChannels[2]);
  Serial.print(" Arm: "); Serial.print(armed ? "YES" : "NO");
  Serial.print(" RTH: "); Serial.print(rthActive ? "ON" : "OFF");
  Serial.print(" STAB: "); Serial.print(stabilizationMode ? "ON" : "OFF");
  Serial.print(" Roll: "); Serial.print(roll);
  Serial.print(" Pitch: "); Serial.println(pitch);
}

void handleArming() {
  // Check ARM switch position (Channel 5)
  // ARM when switch is high (>1500us)
  bool currentArmState = (rcChannels[ARM_CHANNEL] > RC_CHANNEL_CENTER);
  
  // Detect switch transition from LOW to HIGH for arming
  if (currentArmState && !previousArmState) {
    // Toggle armed state when button is pressed
    armed = !armed;
    
    if (armed) {
      Serial.println("System ARMED");
      // Show green LEDs when armed
      setAllLeds(CRGB::Green);
      FastLED.show();
      delay(500); // Brief indication
      
      // Save home position when first armed (if GPS is valid)
      if (gps.location.isValid() && homeLat == 0.0 && homeLon == 0.0) {
        homeLat = gps.location.lat();
        homeLon = gps.location.lng();
        Serial.print("Home position set: ");
        Serial.print(homeLat, 6);
        Serial.print(", ");
        Serial.println(homeLon, 6);
      }
    } else {
      Serial.println("System DISARMED");
      // Show red LEDs when disarmed
      setAllLeds(CRGB::Red);
      FastLED.show();
      delay(500); // Brief indication
      esc.writeMicroseconds(1000); // Set ESC to min throttle when disarmed
      
      // Turn off RTH when disarming
      rthActive = false;
      automaticRthActive = false;
      
      // Turn off stabilization when disarming
      stabilizationMode = false;
    }
  }
  
  // Store current state for next comparison
  previousArmState = currentArmState;
}

void updateOutputs() {
  // Map channel values to servo and ESC
  int throttleValue, steeringValue;
  
  if (rthActive && gpsValid) {
    // In RTH mode, calculate controls based on GPS data
    calculateRTHControls(throttleValue, steeringValue);
  } else {
    // Normal RC control
    throttleValue = map(rcChannels[THROTTLE_CHANNEL], RC_CHANNEL_MIN, RC_CHANNEL_MAX, 1000, 2000);
    steeringValue = map(rcChannels[STEERING_CHANNEL], RC_CHANNEL_MIN, RC_CHANNEL_MAX, 1000, 2000);
  }
  
  // Apply stabilization if enabled and accelerometer is available
  if (stabilizationMode && accelerometerAvailable) {
    applyStabilization(steeringValue);
  }
  
  // Only update ESC if armed
  if (armed) {
    esc.writeMicroseconds(throttleValue);
  } else {
    // Always set minimum throttle when disarmed
    esc.writeMicroseconds(1000);
  }
  
  // Update servo regardless of arm state
  servo.writeMicroseconds(steeringValue);
}

void updateLEDs() {
  // Get LED color components from channels
  redValue = map(rcChannels[LED_RED_CHANNEL], RC_CHANNEL_MIN, RC_CHANNEL_MAX, 0, 255);
  greenValue = map(rcChannels[LED_GREEN_CHANNEL], RC_CHANNEL_MIN, RC_CHANNEL_MAX, 0, 255);
  blueValue = map(rcChannels[LED_BLUE_CHANNEL], RC_CHANNEL_MIN, RC_CHANNEL_MAX, 0, 255);
  
  // Update the LED color
  ledColor = CRGB(redValue, greenValue, blueValue);
  
  // Get LED mode from channel
  if (rcChannels[LED_MODE_CHANNEL] < 500) {
    ledMode = 0; // Solid color
  } else if (rcChannels[LED_MODE_CHANNEL] < 1000) {
    ledMode = 1; // Breathing
  } else if (rcChannels[LED_MODE_CHANNEL] < 1500) {
    ledMode = 2; // Rainbow
  } else if (rcChannels[LED_MODE_CHANNEL] < 2000) {
    ledMode = 3; // Chase
  } else {
    ledMode = 4; // Alternating
  }
}

void setAllLeds(CRGB color) {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = color;
  }
}

void runLedEffect() {
  static uint8_t hue = 0;
  static uint8_t pos = 0;
  static uint8_t brightness = 0;
  static bool increasing = true;
  
  // Check for excessive tilt and override LEDs if necessary
  if (accelerometerAvailable && armed) {
    if (abs(roll) > ROLL_THRESHOLD || abs(pitch) > PITCH_THRESHOLD) {
      // Excessive tilt detected - flash warning pattern
      if ((millis() / 100) % 2 == 0) {
        setAllLeds(CRGB::Red);
        FastLED.show();
        return;
      }
    }
  }
  
  // Override LED effects for RTH mode
  if (rthActive) {
    // Alternating yellow/blue pattern for RTH
    for(int i = 0; i < NUM_LEDS; i++) {
      if ((i + (millis() / 250) % 2) % 2 == 0) {
        leds[i] = CRGB::Yellow;
      } else {
        leds[i] = CRGB::Blue;
      }
    }
    FastLED.show();
    return;
  }
  
  // Standard LED effects
  switch (ledMode) {
    case 0: // Solid color
      setAllLeds(ledColor);
      break;
      
    case 1: // Breathing effect
      if (increasing) {
        brightness += 5;
        if (brightness >= 250) increasing = false;
      } else {
        brightness -= 5;
        if (brightness <= 5) increasing = true;
      }
      setAllLeds(CRGB(
        (redValue * brightness) / 255,
        (greenValue * brightness) / 255,
        (blueValue * brightness) / 255
      ));
      break;
      
    case 2: // Rainbow effect
      fill_rainbow(leds, NUM_LEDS, hue, 255/NUM_LEDS);
      hue++;
      break;
      
    case 3: // Chase effect
      for(int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB::Black;
      }
      leds[pos] = ledColor;
      pos = (pos + 1) % NUM_LEDS;
      break;
      
    case 4: // Alternating effect
      for(int i = 0; i < NUM_LEDS; i++) {
        if ((i + pos) % 2 == 0) {
          leds[i] = ledColor;
        } else {
          leds[i] = CRGB::Black;
        }
      }
      if (++pos >= 2) pos = 0;
      break;
  }
  
  // Status indicators on first few LEDs
  if (!gpsValid) {
    // GPS not valid - show red on first LED
    leds[0] = CRGB::Red;
  } else if (automaticRthActive) {
    // Signal lost, automatic RTH - show orange on first LED
    leds[0] = CRGB::Orange;
  }
  
  if (!accelerometerAvailable) {
    // Accelerometer not working - show purple on second LED
    leds[1] = CRGB::Purple;
  } else if (stabilizationMode) {
    // Stabilization active - show cyan on second LED
    leds[1] = CRGB::Cyan;
  }
  
  FastLED.show();
}

void processGPS() {
  // Read data from GPS
  while (GpsSerial.available()) {
    if (gps.encode(GpsSerial.read())) {
      // New GPS data available
      if (gps.location.isValid()) {
        gpsLatitude = gps.location.lat() * 10000000;
        gpsLongitude = gps.location.lng() * 10000000;
        gpsSatellites = gps.satellites.value();
        speed = gps.speed.knots();
        altitude = gps.altitude.meters();
        heading = gps.course.deg();
        
        gpsValid = true;
        
        // Set home position if not set yet and we're armed
        if (armed && homeLat == 0.0 && homeLon == 0.0) {
          homeLat = gps.location.lat();
          homeLon = gps.location.lng();
          Serial.print("Home position set: ");
          Serial.print(homeLat, 6);
          Serial.print(", ");
          Serial.println(homeLon, 6);
        }
      } else {
        gpsValid = false;
      }
    }
  }
  
  // Check for GPS timeout
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println("WARNING: No GPS data detected");
    gpsValid = false;
  }
}

void processAccelerometer() {
  static uint32_t lastAccelUpdate = 0;
  
  // Only update at the specified rate
  if (!accelerometerAvailable || millis() - lastAccelUpdate < (1000 / ACC_SAMPLE_RATE)) {
    return;
  }
  
  lastAccelUpdate = millis();
  
  // Get new sensor events
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Save raw values
  accelX = a.acceleration.x;
  accelY = a.acceleration.y;
  accelZ = a.acceleration.z;
  gyroX = g.gyro.x;
  gyroY = g.gyro.y;
  gyroZ = g.gyro.z;
  temperature = temp.temperature;
  
  // Calculate roll and pitch angles (simplified, in degrees)
  // Roll (X-axis rotation, side-to-side tilt)
  roll = atan2(accelY, accelZ) * 180.0 / PI;
  
  // Pitch (Y-axis rotation, front-to-back tilt)
  pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;
}

void checkSignalTimeout() {
  // Check if signal has been lost for too long
  if (armed && !rthActive && (millis() - lastSignalTime) > SIGNAL_TIMEOUT) {
    // Signal timeout detected, activate RTH
    rthActive = true;
    automaticRthActive = true;
    Serial.println("Signal lost! Activating automatic RTH");
    
    // Flash all LEDs red to indicate signal loss
    for (int i = 0; i < 3; i++) {
      setAllLeds(CRGB::Red);
      FastLED.show();
      delay(200);
      setAllLeds(CRGB::Black);
      FastLED.show();
      delay(200);
    }
  }
  
  // If signal is back, allow deactivating automatic RTH
  if (automaticRthActive && (millis() - lastSignalTime) < 500) {
    // Signal is back, but keep RTH active until explicitly cancelled
    automaticRthActive = false;
    Serial.println("Signal recovered, automatic RTH can be cancelled");
  }
}

void handleRTH() {
  // RTH only works if GPS is valid and home position is set
  if (!gpsValid || homeLat == 0.0 || homeLon == 0.0) {
    Serial.println("RTH active but GPS invalid or home position not set");
    return;
  }
  
  // Calculate distance to home
  double distToHome = 
    TinyGPSPlus::distanceBetween(
      gps.location.lat(), 
      gps.location.lng(),
      homeLat, 
      homeLon);
  
  // Calculate course to home
  double courseToHome = 
    TinyGPSPlus::courseTo(
      gps.location.lat(), 
      gps.location.lng(),
      homeLat, 
      homeLon);
  
  Serial.print("Distance to home: ");
  Serial.print(distToHome);
  Serial.print("m, Course: ");
  Serial.print(courseToHome);
  Serial.print("°, Current heading: ");
  Serial.println(heading);
  
  // Check if we've reached home
  if (distToHome < RTH_DISTANCE_THRESHOLD) {
    Serial.println("Reached home position!");
    
    // If automatic RTH, stop at home
    if (automaticRthActive) {
      // Just hover at home at minimum throttle
      esc.writeMicroseconds(1000);
      servo.writeMicroseconds(1500);
    }
    
    // Flash green LEDs to indicate arrival at home
    static uint32_t lastHomeFlash = 0;
    if (millis() - lastHomeFlash > 1000) {
      lastHomeFlash = millis();
      static bool homeFlashState = false;
      homeFlashState = !homeFlashState;
      
      if (homeFlashState) {
        setAllLeds(CRGB::Green);
      } else {
        setAllLeds(CRGB::Blue);
      }
      FastLED.show();
    }
  }
}

void calculateRTHControls(int &throttle, int &steering) {
  // Calculate distance to home
  double distToHome = 
    TinyGPSPlus::distanceBetween(
      gps.location.lat(), 
      gps.location.lng(),
      homeLat, 
      homeLon);
  
  // Calculate course to home
  double courseToHome = 
    TinyGPSPlus::courseTo(
      gps.location.lat(), 
      gps.location.lng(),
      homeLat, 
      homeLon);
  
  // Calculate heading error (how much we need to turn)
  double headingError = courseToHome - heading;
  
  // Normalize heading error to -180 to +180 range
  while (headingError > 180) headingError -= 360;
  while (headingError < -180) headingError += 360;
  
  // Convert heading error to steering value
  // Heading error of +/-180 should map to full deflection
  int steeringOffset = map(headingError * 10, -1800, 1800, -500, 500);
  steeringOffset = constrain(steeringOffset, -500, 500);
  
  // Apply proportional control to reduce oscillations
  steeringOffset = steeringOffset * HEADING_CORRECTION_FACTOR / 100;
  
  // Calculate throttle and steering values
  if (distToHome < RTH_DISTANCE_THRESHOLD) {
    // Close to home - slow down
    throttle = 1000; // Minimum throttle
  } else {
    // En route to home - use moderate throttle
    throttle = 1000 + (RTH_SPEED * 1000.0); // Scale from 0-1 to 0-1000
  }
  
  // Center position + steering correction
  steering = 1500 + steeringOffset;
  
  // Apply stabilization if enabled
  if (stabilizationMode && accelerometerAvailable) {
    applyStabilization(steering);
  }
  
  // Ensure values are within safe range
  throttle = constrain(throttle, 1000, 2000);
  steering = constrain(steering, 1000, 2000);
}

void applyStabilization(int &steering) {
  // Only apply if accelerometer is available
  if (!accelerometerAvailable) {
    return;
  }
  
  // Use roll to compensate steering (helps keep straight line in waves)
  // Roll to the right (positive) should decrease steering value
  // Roll to the left (negative) should increase steering value
  int rollCompensation = -roll * TILT_COMPENSATION_FACTOR / 10;
  
  // Apply compensation to steering
  steering += rollCompensation;
  
  // Ensure steering stays within limits
  steering = constrain(steering, 1000, 2000);
}

void updateBatteryData() {
  // Read voltage and current from INA219
  batteryVoltage = ina219.getBusVoltage_V();
  batteryCurrent = ina219.getCurrent_mA() / 1000.0;
  
  // Update battery capacity estimate
  static unsigned long lastBatteryUpdate = 0;
  if (lastBatteryUpdate > 0) {
    unsigned long timeDelta = millis() - lastBatteryUpdate;
    float currentDraw = batteryCurrent * (timeDelta / 3600000.0); // Convert to hours
    batteryCapacity -= currentDraw;
  }
  lastBatteryUpdate = millis();
}

void sendTelemetry() {
  // Rotate between different telemetry types
  static uint8_t telemetryType = 0;
  
  switch (telemetryType) {
    case 0:
      sendBatteryTelemetry();
      break;
    case 1:
      sendGpsTelemetry();
      break;
    case 2:
      if (accelerometerAvailable) {
        sendAttitudeTelemetry();
      }
      break;
  }
  
  telemetryType = (telemetryType + 1) % 3;
}

void sendBatteryTelemetry() {
  uint8_t frame[10];
  
  // Prepare battery telemetry frame
  frame[0] = CRSF_ADDRESS_RADIO_TRANSMITTER; // Destination address
  frame[1] = 8; // Length (excluding address and length)
  frame[2] = CRSF_FRAMETYPE_BATTERY_SENSOR; // Frame type
  
  // Battery voltage (multiply by 10 and convert to uint16_t)
  uint16_t voltage = batteryVoltage * 10;
  frame[3] = voltage & 0xFF;
  frame[4] = (voltage >> 8) & 0xFF;
  
  // Battery current (multiply by 10 and convert to uint16_t)
  uint16_t current = batteryCurrent * 10;
  frame[5] = current & 0xFF;
  frame[6] = (current >> 8) & 0xFF;
  
  // Fuel (remaining capacity as percentage)
  uint8_t fuelPercent = constrain((batteryCapacity / 1500.0) * 100, 0, 100);
  frame[7] = fuelPercent;
  
  // Calculate CRC
  frame[8] = crc8(&frame[2], 6);
  
  // Send the frame
  CrsfSerial.write(frame, 9);
}

void sendGpsTelemetry() {
  uint8_t frame[18];
  
  // Prepare GPS telemetry frame
  frame[0] = CRSF_ADDRESS_RADIO_TRANSMITTER; // Destination address
  frame[1] = 14; // Length (excluding address and length)
  frame[2] = CRSF_FRAMETYPE_GPS_SENSOR; // Frame type
  
  // Latitude (already in format * 10^7)
  frame[3] = gpsLatitude & 0xFF;
  frame[4] = (gpsLatitude >> 8) & 0xFF;
  frame[5] = (gpsLatitude >> 16) & 0xFF;
  frame[6] = (gpsLatitude >> 24) & 0xFF;
  
  // Longitude (already in format * 10^7)
  frame[7] = gpsLongitude & 0xFF;
  frame[8] = (gpsLongitude >> 8) & 0xFF;
  frame[9] = (gpsLongitude >> 16) & 0xFF;
  frame[10] = (gpsLongitude >> 24) & 0xFF;
  
  // Ground speed (in cm/s)
  uint16_t gSpeed = speed * 51.4444; // Convert knots to cm/s
  frame[11] = gSpeed & 0xFF;
  frame[12] = (gSpeed >> 8) & 0xFF;
  
  // Heading (0-359°)
  frame[13] = (uint16_t)heading & 0xFF;
  
  // Altitude (in meters, offset by 1000m)
  uint16_t alt = altitude + 1000;
  frame[14] = alt & 0xFF;
  frame[15] = (alt >> 8) & 0xFF;
  
  // Satellites
  frame[16] = gpsSatellites;
  
  // Calculate CRC
  frame[17] = crc8(&frame[2], 15);
  
  // Send the frame
  CrsfSerial.write(frame, 18);
}

void sendAttitudeTelemetry() {
  uint8_t frame[8];
  
  // Prepare attitude telemetry frame
  frame[0] = CRSF_ADDRESS_RADIO_TRANSMITTER; // Destination address
  frame[1] = 6; // Length (excluding address and length)
  frame[2] = CRSF_FRAMETYPE_ATTITUDE; // Frame type
  
  // Pitch in decidegrees (-1800..1800) = (-180°..180°) * 10
  int16_t pitchValue = pitch * 10;
  frame[3] = pitchValue & 0xFF;
  frame[4] = (pitchValue >> 8) & 0xFF;
  
  // Roll in decidegrees (-1800..1800) = (-180°..180°) * 10
  int16_t rollValue = roll * 10;
  frame[5] = rollValue & 0xFF;
  frame[6] = (rollValue >> 8) & 0xFF;
  
  // Calculate CRC
  frame[7] = crc8(&frame[2], 5);
  
  // Send the frame
  CrsfSerial.write(frame, 8);
}

// CRC8 implementation for CRSF protocol
uint8_t crc8(const uint8_t *ptr, uint8_t len) {
  static const uint8_t crsf_crc8tab[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
  };
  
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc = crsf_crc8tab[crc ^ ptr[i]];
  }
  return crc;
}