#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Define the pins for the servos
#define DL_servo_pin 9      // Direction Lever Servo Pin
#define VB_servo_pin 10     // Vibration Lever Servo Pin
#define encoder_pin_A 2     // Encoder A Pin
#define encoder_pin_B 3     // Encoder B Pin
#define button_pin 8        // Push Button Pin

// Create Servo objects
Servo DL_servo;           // Direction Lever Servo
Servo VB_servo;           // Vibration Lever Servo

// Set up LCD display (Address 0x27, 16 columns, 2 rows)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Encoder variables
volatile int encoder_pos = 0;    // Encoder position
int last_encoder_pos = 0;        // Last encoder position to track distance
long distance_travelled = 0;     // To store the distance

// Button state variables
bool last_button_state = LOW;
bool button_state = LOW;
bool system_running = false;    // Track system state

// Define servo positions
int neutral_pos = 90;
int forward_pos = 180;
int reverse_pos = -180;
int vibration_pos_1 = 180;
int vibration_pos_2 = -180;

void setup() {
  // Initialize the servos
  DL_servo.attach(DL_servo_pin);
  VB_servo.attach(VB_servo_pin);

// Set initial servo positions
  DL_servo.write(neutral_pos);
  VB_servo.write(neutral_pos);

// Setup Encoder
  pinMode(encoder_pin_A, INPUT);
  pinMode(encoder_pin_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder_pin_A), encoder_ISR, RISING);

// Setup Push Button
  pinMode(button_pin, INPUT);
// Initialize LCD
  lcd.begin();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("System Ready");
  delay(1000);
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
      DL_servo.write(neutral_pos);
      VB_servo.write(neutral_pos);
    }
    delay(500);  // Debounce delay
  }
  last_button_state = button_state;
 // Update LCD with distance traveled

if (system_running) {
    distance_travelled = encoder_pos / 7.83;  // Assuming encoder counts correspond to 7.83m of distance
    lcd.setCursor(0, 1);
    lcd.print("Dist: ");
    lcd.print(distance_travelled);
    lcd.print(" m");
  }
}void encoder_ISR() {
  int MSB = digitalRead(encoder_pin_A);  // Most significant bit

int LSB = digitalRead(encoder_pin_B);  // Least significant bit

  int encoded = (MSB << 1) | LSB;       // Combine MSB and LSB
  int sum = (encoder_pos << 2) | encoded;  // Add to current position

  // Update encoder position
  if (sum == 0b1101 || sum == 0b0110 || sum == 0b1010 || sum == 0b0101) {
    encoder_pos++;   // Clockwise rotation
  } else {
    encoder_pos--;   // Counter-clockwise rotation
  }
}
void start_operation() {
  // Move Direction Lever to Forward (180°) and Vibration Mode 1 (180°)
  DL_servo.write(forward_pos);
  VB_servo.write(vibration_pos_1);
  delay(500);  // Wait for servos to reach positions
 // Monitor distance using encoder, stop when target distance is reached
  while (encoder_pos / 7.83 < 7.83) {  // Example: 7.83 meters
    // Continue until the target distance is reached
  }
  // Reset servos to Neutral (90°)
  DL_servo.write(neutral_pos);
  VB_servo.write(neutral_pos);
  delay(500);
  // Move Direction Lever to Reverse (-180°) and Vibration Mode 2 (-180°)
  DL_servo.write(reverse_pos);
  VB_servo.write(vibration_pos_2);
  delay(500);  // Wait for servos to reach positions
// Monitor distance again in reverse direction
  while (encoder_pos / 7.83 < 8.37) {  // Example: 8.37 meters in reverse
    // Continue until the target distance is reached
  }

  // Reset servos to Neutral (90°)
  DL_servo.write(neutral_pos);
  VB_servo.write(neutral_pos);
  delay(500);
// Operation Complete
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Operation Complete");
  delay(1000);
}
