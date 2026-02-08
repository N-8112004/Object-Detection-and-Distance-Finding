# Hardware Connection Guide

## Components Required:
- Arduino Nano
- HC-SR04 Ultrasonic Sensor
- I2C LCD Display (16x2)
- Buzzer (Active or Passive)
- Jumper Wires
- Breadboard (optional)

## Pin Connections:

### Ultrasonic Sensor (HC-SR04):
- VCC  → Arduino 5V
- GND  → Arduino GND
- TRIG → Arduino D9
- ECHO → Arduino D10

### I2C LCD Display:
- VCC → Arduino 5V
- GND → Arduino GND
- SDA → Arduino A4 (SDA)
- SCL → Arduino A5 (SCL)

### Buzzer:
- Positive (+) → Arduino D8
- Negative (-) → Arduino GND

## Power Supply:
- USB Power: Connect Arduino Nano via USB
- External Power: 7-12V through Vin pin (if needed)

## Important Notes:
1. Ensure all GND connections are common
2. I2C LCD may have address 0x27 or 0x3F (run I2C scanner if unsure)
3. Use 220Ω resistor for buzzer if too loud
4. Keep ultrasonic sensor free from obstructions
