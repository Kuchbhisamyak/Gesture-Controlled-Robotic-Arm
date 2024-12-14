#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create an instance of the PCA9685 driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_MIN 150 // Minimum pulse length count (for 0 degrees)
#define SERVO_MAX 600 // Maximum pulse length count (for 180 degrees)
#define SERVO_CHANNEL 0 // Channel on PCA9685 to which the servo is connected

// Function to map degree angles to pulse length
int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing PCA9685...");
  
  pwm.begin();
  pwm.setPWMFreq(50); // Standard servo PWM frequency: 50Hz
  delay(10); // Give time for the PCA9685 to initialize
  
  Serial.println("Rotating servo...");

  pwm.setPWM(SERVO_CHANNEL, 0, angleToPulse(0));
  Serial.println("Servo at 0 degrees");
  delay(1000); // Wait for 1 second

  // Rotate servo to 90 degrees
  pwm.setPWM(SERVO_CHANNEL, 0, angleToPulse(90));
  Serial.println("Servo at 90 degrees");
  delay(1000); // Wait for 1 second

  // Rotate servo to 180 degrees
  pwm.setPWM(SERVO_CHANNEL, 0, angleToPulse(180));
  Serial.println("Servo at 180 degrees");
  delay(1000); // Wait for 1 second
}

void loop() {
 pwm.setPWM(SERVO_CHANNEL, 0, angleToPulse(0));
  Serial.println("Servo at 0 degrees");
  delay(1000); // Wait for 1 second

  // Rotate servo to 90 degrees
  pwm.setPWM(SERVO_CHANNEL, 0, angleToPulse(90));
  Serial.println("Servo at 90 degrees");
  delay(1000); // Wait for 1 second

  // Rotate servo to 180 degrees
  pwm.setPWM(SERVO_CHANNEL, 0, angleToPulse(180));
  Serial.println("Servo at 180 degrees");
  delay(1000); 
}
