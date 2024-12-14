//Final receiver code of all integration 
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <esp_now.h>

// PCA9685 setup
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo pulse range
#define SERVOMIN 150    // Minimum pulse length for servo
#define SERVOMAX 600    // Maximum pulse length for servo

// Servo Channels
#define BASE_SERVO1_CHANNEL 0 // Base servo 1 channel
//#define BASE_SERVO2_CHANNEL 1 // Base servo 2 channel
#define CLAW_CHANNEL 5        // Claw connected to channel 5
#define WRIST1_CHANNEL 4      // Wrist1 connected to channel 4
#define WRIST2_CHANNEL 3      // Wrist2 connected to channel 3
#define ELBOW_CHANNEL 2       // Elbow connected to channel 2

// Stepper Motor Pins and Settings
#define DIR_PIN 12            // Stepper direction pin
#define STEP_PIN 14           // Stepper step pin
#define STEPS_PER_REV 200     // Steps per revolution (NEMA 17)

// Base servo stationary positions
const int baseServo1Angle = 90;  // Fixed position for Base Servo 1
const int baseServo2Angle = 90;  // Fixed position for Base Servo 2

// Initial positions for other servos
int clawPosition = 90;    // Claw starts at neutral position (90 degrees)
int wrist1Position = 90;  // Wrist1 starts at neutral position (90 degrees)
int wrist2Position = 90;  // Wrist2 starts at neutral position (90 degrees)
int elbowPosition = 90;   // Elbow starts at neutral position (90 degrees)

// Speed of increment/decrement for servos
int speed = 20;

// Define data structure for ESP-NOW
typedef struct struct_message {
  char command; // Command sent by the transmitter
} struct_message;

struct_message receivedData; // Data received from transmitter

// Function to set servo angle
void setServoAngle(uint8_t channel, int angle) {
  int pulseLength = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulseLength);
}

// Function to set base servos to fixed positions
void setBaseServos() {
  int pulseBase1 = map(baseServo1Angle, 0, 180, SERVOMIN, SERVOMAX);
  int pulseBase2 = map(baseServo2Angle, 0, 180, SERVOMIN, SERVOMAX);

  pwm.setPWM(BASE_SERVO1_CHANNEL, 0, pulseBase1); // Set Base Servo 1 to fixed position
  //pwm.setPWM(BASE_SERVO2_CHANNEL, 0, pulseBase2); // Set Base Servo 2 to fixed position
}

// Function to rotate the stepper motor 360 degrees
void rotateStepper(bool clockwise) {
  digitalWrite(DIR_PIN, clockwise ? HIGH : LOW);
  for (int i = 0; i < STEPS_PER_REV; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(1000);  // Adjust for motor speed
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(1000);
  }
}

// Callback function to handle ESP-NOW data reception
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  Serial.print("Received command: ");
  Serial.println(receivedData.command);

  // Handle commands for servos and stepper motor
  switch (receivedData.command) {
    // Claw control
    case 'X':
      clawPosition += speed;
      if (clawPosition > 180) clawPosition = 180;
      setServoAngle(CLAW_CHANNEL, clawPosition);
      Serial.println("Claw opened by 10 degrees.");
      break;
    case 'Y':
      clawPosition -= speed;
      if (clawPosition < 0) clawPosition = 0;
      setServoAngle(CLAW_CHANNEL, clawPosition);
      Serial.println("Claw closed by 10 degrees.");
      break;

    // Wrist1 control
    case 'L':
      wrist1Position -= speed;
      if (wrist1Position < 0) wrist1Position = 0;
      setServoAngle(WRIST1_CHANNEL, wrist1Position);
      Serial.println("Wrist1 rotated left by 10 degrees.");
      break;
    case 'R':
      wrist1Position += speed;
      if (wrist1Position > 180) wrist1Position = 180;
      setServoAngle(WRIST1_CHANNEL, wrist1Position);
      Serial.println("Wrist1 rotated right by 10 degrees.");
      break;

    // Wrist2 control
    case 'P':
      wrist2Position -= speed;
      if (wrist2Position < 20) wrist2Position = 20;//0 tak jana chahiye
      setServoAngle(WRIST2_CHANNEL, wrist2Position);
      Serial.println("Wrist2 rotated up by 10 degrees.");
      break;
    case 'Q':
      wrist2Position += speed;
      if (wrist2Position > 120) wrist2Position = 120;
      setServoAngle(WRIST2_CHANNEL, wrist2Position);
      Serial.println("Wrist2 rotated down by 10 degrees.");
      break;

    // Elbow control
    case 'D':
      elbowPosition -= speed;
      if (elbowPosition < 20) elbowPosition = 20;
      setServoAngle(ELBOW_CHANNEL, elbowPosition);
      Serial.println("Elbow moved up by 10 degrees.");
      break;
    case 'U':
      elbowPosition += speed;
      if (elbowPosition > 140) elbowPosition = 140;
      setServoAngle(ELBOW_CHANNEL, elbowPosition);
      Serial.println("Elbow moved down by 10 degrees.");
      break;

    // Stepper motor control
    case 'C':
      Serial.println("Rotating stepper clockwise...");
      rotateStepper(true);
      Serial.println("Rotation complete.");
      break;
    case 'A':
      Serial.println("Rotating stepper anticlockwise...");
      rotateStepper(false);
      Serial.println("Rotation complete.");
      break;

    // Invalid command
    default:
      Serial.println("Invalid command received. No action taken.");
      break;
  }
}

void setup() {
  // Initialize pins for stepper motor
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);

  // Initialize Serial Monitor
  Serial.begin(115200);

  // Initialize PCA9685
  pwm.begin();
  pwm.setPWMFreq(50); // Set frequency to 50 Hz
  Serial.println("PCA9685 Initialized!");

  // Set base servos to fixed positions
  setBaseServos();

  // Initialize WiFi in Station Mode (needed for ESP-NOW)
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Initialization Failed!");
    while (1);
  }
  Serial.println("ESP-NOW Initialized!");

  // Register receive callback
  esp_now_register_recv_cb(onDataRecv);
}

void loop() {
  // Keep the base servos stationary
  setBaseServos();

  // Keep other servos at their last received positions
  setServoAngle(CLAW_CHANNEL, clawPosition);
  setServoAngle(WRIST1_CHANNEL, wrist1Position);
  setServoAngle(WRIST2_CHANNEL, wrist2Position);
  setServoAngle(ELBOW_CHANNEL, elbowPosition);
}
