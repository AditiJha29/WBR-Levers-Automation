#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Define the pins for the servos
#define DL_servo_pin 9       // Direction Lever Servo Pin
#define VB_servo_pin 10      // Vibration Lever Servo Pin
#define button_pin 52        // Push Button Pin on Arduino Mega

// Define motor driver pins
#define ENA_pin 5            // PWM Speed Control
#define IN1_pin 6            // Motor Direction Control 1
#define IN2_pin 7            // Motor Direction Control 2

// Define encoder pins
#define ENCODER_DT 2         // Encoder DT pin
#define ENCODER_CLK 3        // Encoder CLK pin

// Constants for distance calculation
const float WHEEL_CIRCUMFERENCE = 6.3; // Wheel circumference in meters (e.g., 20cm diameter)
const int PULSES_PER_REVOLUTION = 20;   // Encoder pulses per revolution

// Variables for encoder
volatile int pulse_count = 0;           // Count of encoder pulses
float distance_achieved = 0.0;          // Distance in meters

// Create Servo objects
Servo DL_servo;              // Direction Lever Servo
Servo VB_servo;              // Vibration Lever Servo

// LCD setup (Address 0x27, 16 columns, 2 rows)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Button state variables
bool last_button_state = LOW;
bool button_state = LOW;
bool system_running = false;  // Track system state

// Define servo positions
int neutral_pos = 90;
int forward_pos = 180;
int reverse_pos = -180;
int vibration_pos_1 = 180;    // Vibration Mode 1 (VBM1)
int vibration_pos_2 = -180;      // Vibration Mode 2 (VBM2)

void setup() {
  // Initialize the servos
  DL_servo.attach(DL_servo_pin);
  VB_servo.attach(VB_servo_pin);

  // Set initial servo positions
  DL_servo.write(neutral_pos);
  VB_servo.write(neutral_pos);

  // Setup Push Button
  pinMode(button_pin, INPUT);

  // Setup motor driver pins
  pinMode(ENA_pin, OUTPUT);
  pinMode(IN1_pin, OUTPUT);
  pinMode(IN2_pin, OUTPUT);

  // Setup encoder pins
  pinMode(ENCODER_DT, INPUT);
  pinMode(ENCODER_CLK, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK), count_pulses, RISING);

  // Initialize LCD
  lcd.begin(16, 2);  // Initialize with 16 columns and 2 rows
  lcd.backlight();   // Turn on the backlight
  lcd.setCursor(0, 0);
  lcd.print("System Ready");

  Serial.begin(9600); // For debugging
}

void loop() {
  // Read button state
  button_state = digitalRead(button_pin);

  // Toggle system start/stop with button
  if (button_state == HIGH && last_button_state == LOW) {
    system_running = !system_running;
    if (system_running) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Operation Start");
      start_operation();
    } else {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("System Stopped");
      stop_motor();
      DL_servo.write(neutral_pos);
      VB_servo.write(neutral_pos);
    }
    delay(500);  // Debounce delay
  }
  last_button_state = button_state;
}

void start_operation() {
  // Reset pulse count
  pulse_count = 0;

  // Step 1: Forward Operation
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Dist Forward VB1");
  DL_servo.write(forward_pos);
  VB_servo.write(vibration_pos_1);
  run_motor(true);
  measure_distance(7.83);  // Target distance for forward

  // Move to neutral after forward operation
  stop_motor();
  DL_servo.write(neutral_pos);
  VB_servo.write(neutral_pos);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Neutral");
  delay(5000);

  // Reset for next step
  pulse_count = 0;

  // Step 2: Reverse Operation
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print ("Dist Reverse VB2");
  VB_servo.write(vibration_pos_2);
  DL_servo.write(reverse_pos);
  run_motor(false);
  measure_distance(8.37);  // Target distance for reverse

  // Stop motor and end operation
  stop_motor();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Operation Done");
  delay(2000);
}

void measure_distance(float target_distance) {
  while (distance_achieved < target_distance) {
    // Calculate distance achieved
    distance_achieved = (pulse_count * WHEEL_CIRCUMFERENCE) / PULSES_PER_REVOLUTION;

    // Display real-time distance
    lcd.setCursor(0, 1);
    lcd.print("Dist: ");
    lcd.print(distance_achieved, 2);  // Display up to 2 decimal places
    lcd.print("m ");
    delay(200);  // Update every 200ms
  }
  stop_motor();
  lcd.setCursor(0, 1);
  lcd.print("Target Done");
  delay(2000);
}

void run_motor(bool forward) {
  if (forward) {
    // Forward direction
    digitalWrite(IN1_pin, HIGH);
    digitalWrite(IN2_pin, LOW);
  } else {
    // Reverse direction
    digitalWrite(IN1_pin, LOW);
    digitalWrite(IN2_pin, HIGH);
  }
  analogWrite(ENA_pin, 150);  // Medium speed
}

void stop_motor() {
  analogWrite(ENA_pin, 0);  // Stop motor by setting PWM to 0
  digitalWrite(IN1_pin, LOW);
  digitalWrite(IN2_pin, LOW);
}

// Interrupt Service Routine (ISR) for encoder
void count_pulses() {
  pulse_count++;
}

