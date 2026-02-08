/*
 * Object Detection and Distance Measurement System
 * Hardware: Arduino Nano, HC-SR04 Ultrasonic Sensor, I2C LCD (16x2), Buzzer
 * Author: Ruthradhesma
 * Description: Real-time distance measurement with threshold-based alerting
 * Accuracy: ±2 cm within 20-40 cm range, Response time: <100 ms
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ==================== PIN DEFINITIONS ====================
#define TRIG_PIN        9      // Ultrasonic sensor trigger pin
#define ECHO_PIN        10     // Ultrasonic sensor echo pin
#define BUZZER_PIN      8      // Buzzer pin

// ==================== CONSTANTS ====================
#define SOUND_SPEED     0.034   // Speed of sound in cm/microsecond (343 m/s)
#define CM_TO_INCH      0.393701 // Conversion factor from cm to inches
#define MAX_DISTANCE    400     // Maximum measurable distance in cm
#define MIN_DISTANCE    2       // Minimum measurable distance in cm

// Distance thresholds for alerting (in cm)
#define THRESHOLD_CRITICAL  10  // Critical proximity - continuous beep
#define THRESHOLD_WARNING   20  // Warning zone - fast beep
#define THRESHOLD_CAUTION   40  // Caution zone - slow beep

// Timing constants
#define MEASUREMENT_DELAY   100  // Delay between measurements (ms)
#define TIMEOUT_US         23500 // Timeout for ultrasonic reading (microseconds)

// ==================== GLOBAL VARIABLES ====================
LiquidCrystal_I2C lcd(0x27, 16, 2);  // I2C address 0x27, 16 columns, 2 rows
                                      // Try 0x3F if 0x27 doesn't work

float distanceCm = 0.0;
float distanceInch = 0.0;
unsigned long lastMeasurementTime = 0;
unsigned long lastBuzzerTime = 0;

// ==================== FUNCTION PROTOTYPES ====================
void initializeSystem(void);
float measureDistance(void);
void displayDistance(float cm, float inch);
void handleProximityAlert(float distance);
void buzzerBeep(unsigned int duration);
bool isValidDistance(float distance);

// ==================== SETUP FUNCTION ====================
void setup() {
  initializeSystem();
}

// ==================== MAIN LOOP ====================
void loop() {
  unsigned long currentTime = millis();
  
  // Perform measurement at specified interval
  if (currentTime - lastMeasurementTime >= MEASUREMENT_DELAY) {
    lastMeasurementTime = currentTime;
    
    // Measure distance
    distanceCm = measureDistance();
    
    // Validate and process measurement
    if (isValidDistance(distanceCm)) {
      // Convert to inches
      distanceInch = distanceCm * CM_TO_INCH;
      
      // Display on LCD
      displayDistance(distanceCm, distanceInch);
      
      // Handle proximity alerting
      handleProximityAlert(distanceCm);
    } else {
      // Display error on LCD
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Out of Range");
      lcd.setCursor(0, 1);
      lcd.print("Check Sensor!");
      noTone(BUZZER_PIN);
    }
  }
}

// ==================== FUNCTION IMPLEMENTATIONS ====================

/**
 * Initialize all system components
 */
void initializeSystem(void) {
  // Initialize serial communication for debugging
  Serial.begin(9600);
  
  // Configure pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  // Ensure trigger pin is low
  digitalWrite(TRIG_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  
  // Initialize I2C LCD
  lcd.init();
  lcd.backlight();
  
  // Display startup message
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Distance Sensor");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  delay(2000);
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Ready!");
  delay(1000);
  
  // Buzzer test beep
  buzzerBeep(100);
  delay(100);
  buzzerBeep(100);
  
  Serial.println("System Initialized Successfully");
}

/**
 * Measure distance using ultrasonic sensor (Time-of-Flight method)
 * Returns: Distance in centimeters
 */
float measureDistance(void) {
  long duration;
  float distance;
  
  // Ensure trigger pin is low
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // Send 10 microsecond pulse to trigger
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Read the echo pin (time of flight)
  duration = pulseIn(ECHO_PIN, HIGH, TIMEOUT_US);
  
  // Calculate distance using time-of-flight formula
  // Distance = (Time × Speed of Sound) / 2
  // Division by 2 because sound travels to object and back
  distance = (duration * SOUND_SPEED) / 2.0;
  
  // Debug output
  Serial.print("Duration: ");
  Serial.print(duration);
  Serial.print(" us, Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  
  return distance;
}

/**
 * Display distance on I2C LCD
 */
void displayDistance(float cm, float inch) {
  lcd.clear();
  
  // First line: Distance in centimeters
  lcd.setCursor(0, 0);
  lcd.print("Dist: ");
  lcd.print(cm, 1);  // 1 decimal place
  lcd.print(" cm");
  
  // Second line: Distance in inches
  lcd.setCursor(0, 1);
  lcd.print("      ");
  lcd.print(inch, 2);  // 2 decimal places
  lcd.print(" in");
}

/**
 * Handle proximity alerting based on distance thresholds
 */
void handleProximityAlert(float distance) {
  unsigned long currentTime = millis();
  
  if (distance <= THRESHOLD_CRITICAL) {
    // Critical proximity - continuous high-pitched beep
    tone(BUZZER_PIN, 2500);  // High frequency
    
    // Add warning indicator on LCD
    lcd.setCursor(15, 0);
    lcd.print("!");
    
    Serial.println("ALERT: Critical Proximity!");
    
  } else if (distance <= THRESHOLD_WARNING) {
    // Warning zone - fast beeping (200ms interval)
    if (currentTime - lastBuzzerTime >= 200) {
      buzzerBeep(100);
      lastBuzzerTime = currentTime;
    }
    
    lcd.setCursor(15, 0);
    lcd.print("*");
    
    Serial.println("WARNING: Close Proximity");
    
  } else if (distance <= THRESHOLD_CAUTION) {
    // Caution zone - slow beeping (500ms interval)
    if (currentTime - lastBuzzerTime >= 500) {
      buzzerBeep(80);
      lastBuzzerTime = currentTime;
    }
    
    Serial.println("CAUTION: Moderate Proximity");
    
  } else {
    // Safe distance - no alert
    noTone(BUZZER_PIN);
    digitalWrite(BUZZER_PIN, LOW);
  }
}

/**
 * Generate a buzzer beep for specified duration
 */
void buzzerBeep(unsigned int duration) {
  tone(BUZZER_PIN, 2000, duration);  // 2000 Hz frequency
}

/**
 * Validate if measured distance is within acceptable range
 */
bool isValidDistance(float distance) {
  return (distance >= MIN_DISTANCE && distance <= MAX_DISTANCE && distance > 0);
}
