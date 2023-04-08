#include <stdio.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// General servo variables
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 280
#define SERVO_MIN_PULSE 600 
#define SERVO_MAX_PULSE 2400 
#define SERVO_FREQUENCY 50

// Upper servo variables

#define UPPER_FL_CHANNEL 1
#define UPPER_FR_CHANNEL 0 // TBD
#define UPPER_RL_CHANNEL 0 // TBD
#define UPPER_RR_CHANNEL 0 // TBD
#define UPPER_MIN_ANGLE  0 // TBD
#define UPPER_MID_ANGLE  115 // TBD
#define UPPER_MAX_ANGLE  270 // TBD

// Middle servo variables
#define MIDDLE_FL_CHANNEL 2
#define MIDDLE_FR_CHANNEL 0 // TBD
#define MIDDLE_RL_CHANNEL 0 // TBD
#define MIDDLE_RR_CHANNEL 0 // TBD
#define MIDDLE_MIN_ANGLE  90
#define MIDDLE_MID_ANGLE  110
#define MIDDLE_MAX_ANGLE  180

// Lower servo variables
#define LOWER_FL_CHANNEL 3
#define LOWER_FR_CHANNEL 0 // TBD
#define LOWER_RL_CHANNEL 0 // TBD
#define LOWER_RR_CHANNEL 0 // TBD
#define LOWER_MIN_ANGLE  80
#define LOWER_MID_ANGLE  120
#define LOWER_MAX_ANGLE  170

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


void setupServos() {
  Serial.begin(9700);

  // Initialize the PWM driver
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQUENCY);

  delay(10);
}

void setServoAngle(uint8_t servoPort, int angle) {
  int pulseWide, pwmValue;

  // Check that the desired angle is within the valid range for the servo
  if (angle < SERVO_MIN_ANGLE || angle > SERVO_MAX_ANGLE) {
    Serial.println("Error: Invalid angle for servo");
    return;
  }

  // Map the desired angle to a pulse width range of 600 to 2400 microseconds
  pulseWide = map(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, SERVO_MIN_PULSE, SERVO_MAX_PULSE);

  // Calculate the PWM value based on the pulse width and frequency
  pwmValue = int(float(pulseWide) / 1000000 * SERVO_FREQUENCY * 4096);

  // Check that the calculated PWM value is within the valid range for the PCA9685 board
  if (pwmValue < 0 || pwmValue > 4095) {
    Serial.println("Error: Invalid PWM value for PCA9685 board");
    return;
  }

  pwm.setPWM(servoPort, 0, pwmValue);
}


void setUpperAngle(uint8_t servoPort, int angle) {
  // Check that the desired angle is within the valid range for the servo
  if (angle < UPPER_MIN_ANGLE || angle > UPPER_MAX_ANGLE) {
    Serial.println("Error: Invalid angle for the hip");
    return;
  }
  setServoAngle(servoPort, angle);
}

void setMiddleAngle(uint8_t servoPort, int angle) {
  // Check that the desired angle is within the valid range for the servo
  if (angle < MIDDLE_MIN_ANGLE || angle > MIDDLE_MAX_ANGLE) {
    Serial.println("Error: Invalid angle for the Middle servo");
    return;
  }
  setServoAngle(servoPort, angle);
}

void setLowerAngle(int angle) {
  // Check that the desired angle is within the valid range for the servo
  if (angle < LOWER_MIN_ANGLE || angle > LOWER_MAX_ANGLE) {
    Serial.println("Error: Invalid angle for the ankle");
    return;
  }
  setServoAngle(LOWER_FL_CHANNEL, angle);
}