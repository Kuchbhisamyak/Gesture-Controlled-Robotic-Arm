#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Initialize PCA9685 driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo limits
#define SERVOMIN  150  // Minimum pulse length count (0°)
#define SERVOMAX  600  // Maximum pulse length count (180°)
#define DELAYTIME 30   // Delay time in milliseconds for smooth motion

void setup() {
  Serial.begin(9600);
  Serial.println("Positioning Servos and Rotating Continuously");

  pwm.begin();
  pwm.setPWMFreq(50);  // Analog servos run at 50 Hz
  delay(10);

  // Set servos 0 and 1 to 60 degrees
  int pos120 = map(120, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(0, 0, pos120);  // Servo on channel 0 to 60 degrees
  pwm.setPWM(1, 0, pos120);  // Servo on channel 1 to 60 degrees

  // Set servo 2 to 150 degrees
  int pos90 = map(90, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(2, 0, pos90);  // Servo on channel 2 to 150 degrees

  // Set servo 3 to 90 degrees (central position)
  int pos90 = map(90, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(3, 0, pos90);  // Servo on channel 3 to 90 degrees
}

void loop() {
  int pos60 = map(60, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(0, 0, pos60);  // Servo on channel 0 to 60 degrees
  pwm.setPWM(1, 0, pos60);
  // Move servo 3 between 40 and 140 degrees (90 ± 50 degrees)
  for (int pos = 40; pos <= 140; pos++) {
    int pulse = map(pos, 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(3, 0, pulse);  // Servo on channel 3
    delay(DELAYTIME);         // Small delay for smoother motion
  }

  // Move servo 3 back from 140 to 40 degrees
  for (int pos = 140; pos >= 40; pos--) {
    int pulse = map(pos, 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(3, 0, pulse);  // Servo on channel 3
    delay(DELAYTIME);         // Small delay for smoother motion
  }

  // Continuously rotate servos 4 and 5 from 0 to 180 degrees and back
  for (int pos = 0; pos <= 180; pos++) {
    int pulse = map(pos, 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(4, 0, pulse);  // Servo on channel 4
    pwm.setPWM(5, 0, pulse);  // Servo on channel 5
    delay(DELAYTIME);         // Small delay for smoother motion
  }

  // Move servos 4 and 5 back from 180 to 0 degrees
  for (int pos = 180; pos >= 0; pos--) {
    int pulse = map(pos, 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(4, 0, pulse);  // Servo on channel 4
    pwm.setPWM(5, 0, pulse);  // Servo on channel 5
    delay(DELAYTIME);         // Small delay for smoother motion
  }
}
