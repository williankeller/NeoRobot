#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// General servo variables
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 280
#define SERVO_MIN_PULSE_WIDTH 600 
#define SERVO_MAX_PULSE_WIDTH 2400 
#define SERVO_FREQUENCY 50

// Hip servo variables
#define HIP_SERVO_PORT 4
#define HIP_NEUTRAL_ANGLE 90

// Thigh servo variables
#define THIGH_NEUTRAL_ANGLE 90

// Ankle servo variables
#define ANKLE_SERVO_PORT 4
#define ANKLE_MIN_ANGLE 80
#define ANKLE_NEUTRAL_ANGLE 120
#define ANKLE_MAX_ANGLE 170


void setup() {
  Serial.begin(9600);

  // Initialize the PWM driver
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQUENCY);

  delay(10);
}

void loop() {

  setAnkleAngle(100);
}

void setAnkleAngle(int angle) {
  // Check that the desired angle is within the valid range for the servo
  if (angle < ANKLE_MIN_ANGLE || angle > ANKLE_MAX_ANGLE) {
    Serial.println("Error: Invalid angle for the ankle");
    return;
  }
  setServoAngle(ANKLE_SERVO_PORT, angle);
}

void setServoAngle(uint8_t servoPort, int angle) {
  int pulseWide, pwmValue;

  // Check that the desired angle is within the valid range for the servo
  if (angle < SERVO_MIN_ANGLE || angle > SERVO_MAX_ANGLE) {
    Serial.println("Error: Invalid angle for servo");
    return;
  }

  // Map the desired angle to a pulse width range of 600 to 2400 microseconds
  pulseWide = map(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);

  // Calculate the PWM value based on the pulse width and frequency
  pwmValue = int(float(pulseWide) / 1000000 * SERVO_FREQUENCY * 4096);

  // Check that the calculated PWM value is within the valid range for the PCA9685 board
  if (pwmValue < 0 || pwmValue > 4095) {
    Serial.println("Error: Invalid PWM value for PCA9685 board");
    return;
  }

  pwm.setPWM(servoPort, 0, pwmValue);
}