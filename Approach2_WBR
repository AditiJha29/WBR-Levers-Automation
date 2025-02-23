// Pin Assignments
#define RELAY1_PIN 22  // Forward direction of DL
#define RELAY2_PIN 23  // Reverse direction of DL
#define RELAY3_PIN 24  // Forward direction of VBL (VBM1)
#define RELAY4_PIN 25  // Reverse direction of VBL (VBM2)
#define ENCODER_PIN A0 // Encoder for distance tracking

// Constants
const float SPEED = 4.5 * 1000 / 3600;  // Convert speed from km/h to m/s
const float DISTANCE1 = 7.87;           // First forward distance in meters
const float DISTANCE_REVERSE = 8.37;    // Reverse distance in meters
const float L2 = 2.0;                   // Adjust based on requirement

// Stroke Length Calculation for Direction Lever (DL) and Vibration Lever (VBL)
const float DL_FD = 100 * cos(35.94 * PI / 180); // Forward stroke length for DL
const float DL_RD = 100 * cos(-29.73 * PI / 180); // Reverse stroke length for DL
const float VBL_FD = 60 * cos(43.97 * PI / 180);  // Forward stroke length for VBL
const float VBL_RD = 60 * cos(-43.97 * PI / 180); // Reverse stroke length for VBL

// Neutral Positions for both levers
const float DL_Neutral = (DL_FD + DL_RD) / 2; // Neutral position for DL
const float VBL_Neutral = (VBL_FD + VBL_RD) / 2; // Neutral position for VBL

// Variables
volatile float distanceTravelled = 0.0;
unsigned long lastTime = 0;

// LCD setup (I2C, 16x2)
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Function to simulate encoder pulse counting
void encoderInterrupt() {
  static int pulseCount = 0;
  float wheelCircumference = 0.3; // Adjust as per your wheel diameter
  int pulsesPerRevolution = 360;  // Encoder ticks per revolution
pulseCount++;
  distanceTravelled = (pulseCount * wheelCircumference) / pulsesPerRevolution;
}
void setup() {
  // Setup relay pins
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY3_PIN, OUTPUT);
  pinMode(RELAY4_PIN, OUTPUT);

 // Initialize relays to LOW (neutral position)
  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);
  digitalWrite(RELAY3_PIN, LOW);
  digitalWrite(RELAY4_PIN, LOW);

// Setup encoder pin and interrupt
  pinMode(ENCODER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderInterrupt, RISING);

// Initialize serial monitor
  Serial.begin(9600);

// Initialize LCD
  lcd.begin();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("System Ready");
  delay(2000); // Display "System Ready" for 2 seconds
  lcd.clear();
}
void loop() {

  // Step 1: Machine ON, initialize
  delay(2000); // Simulate start delay

 // Step 2: Relay 3 HIGH (VBM1), wait 2 seconds
  digitalWrite(RELAY3_PIN, HIGH);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("VBM1 ON");
  delay(2000);

  // Step 3: Relay 1 HIGH (Forward), track distance
  distanceTravelled = 0.0; // Reset distance
  digitalWrite(RELAY1_PIN, HIGH);
 lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Moving Forward");
while (distanceTravelled < DISTANCE1) {
    // Wait until the machine covers 7.87m
    lcd.setCursor(0, 1);
  lcd.print("Dist: ");
    lcd.print(distanceTravelled);
    lcd.print(" m");    delay(100); // Small delay to allow encoder to count pulses
  }
 // Turn OFF Relay 3 and Relay 1 (Neutral position)
  digitalWrite(RELAY3_PIN, LOW);
  digitalWrite(RELAY1_PIN, LOW);
// Step 4: Pause for 5 seconds
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Pause");
  delay(5000);
// Step 5: Relay 4 HIGH (VBM2), wait 2 seconds
  digitalWrite(RELAY4_PIN, HIGH);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("VBM2 ON");
  delay(2000);
// Step 6: Relay 2 HIGH (Reverse), track distance
  distanceTravelled = 0.0; // Reset distance
  digitalWrite(RELAY2_PIN, HIGH);
lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Moving Backward");
while (distanceTravelled < DISTANCE_REVERSE) {

    // Wait until the machine covers 8.37m
    lcd.setCursor(0, 1);
    lcd.print("Dist: ");
    lcd.print(distanceTravelled);
    lcd.print(" m");
    delay(100); // Small delay to allow encoder to count pulses
  }
// Turn OFF Relay 4 and Relay 2 (Neutral position)
  digitalWrite(RELAY4_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);

 // Loop ends, reset system or add further steps if needed
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Operation Complete");
  delay(2000); // Display "Operation Complete" for 2 seconds
  lcd.clear();

  // Optionally print stroke lengths and neutral positions for debugging
  Serial.print("DL Forward Stroke: ");
  Serial.println(DL_FD);
  Serial.print("DL Reverse Stroke: ");
  Serial.println(DL_RD);
  Serial.print("DL Neutral Position: ");
  Serial.println(DL_Neutral);

  Serial.print("VBL Forward Stroke: ");
  Serial.println(VBL_FD);
  Serial.print("VBL Reverse Stroke: ");
  Serial.println(VBL_RD);
  Serial.print("VBL Neutral Position: ");
  Serial.println(VBL_Neutral);
}

