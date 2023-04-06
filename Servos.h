#include <stdio.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// General servo variables
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 280
#define SERVO_MIN_PULSE 600 
#define SERVO_MAX_PULSE 2400 
#define SERVO_FREQUENCY 50

// Hip servo variables
#define HIP_FR_CHANNEL 1
#define HIP_FL_CHANNEL 2
#define HIP_RR_CHANNEL 3
#define HIP_RL_CHANNEL 4
#define HIP_MIN_ANGLE  90
#define HIP_MID_ANGLE  90
#define HIP_MAX_ANGLE  170

// Thigh servo variables
#define THIGH_FR_CHANNEL 5
#define THIGH_FL_CHANNEL 6
#define THIGH_RR_CHANNEL 7
#define THIGH_RL_CHANNEL 8
#define THIGH_MIN_ANGLE  90
#define THIGH_MID_ANGLE  90
#define THIGH_MAX_ANGLE  170

// Ankle servo variables
#define ANKLE_FR_CHANNEL 9
#define ANKLE_FL_CHANNEL 10
#define ANKLE_RR_CHANNEL 11
#define ANKLE_RL_CHANNEL 12
#define ANKLE_MIN_ANGLE  80
#define ANKLE_MID_ANGLE  120
#define ANKLE_MAX_ANGLE  170

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

void setAnkleAngle(int angle) {
  // Check that the desired angle is within the valid range for the servo
  if (angle < ANKLE_MIN_ANGLE || angle > ANKLE_MAX_ANGLE) {
    Serial.println("Error: Invalid angle for the ankle");
    return;
  }
  setServoAngle(ANKLE_FL_CHANNEL, angle);
}

void setHipAngle(uint8_t servoPort, int angle) {
  // Check that the desired angle is within the valid range for the servo
  if (angle < HIP_MIN_ANGLE || angle > HIP_MAX_ANGLE) {
    Serial.println("Error: Invalid angle for the hip");
    return;
  }
  setServoAngle(servoPort, angle);
}

void setThighAngle(uint8_t servoPort, int angle) {
  // Check that the desired angle is within the valid range for the servo
  if (angle < THIGH_MIN_ANGLE || angle > THIGH_MAX_ANGLE) {
    Serial.println("Error: Invalid angle for the thigh");
    return;
  }
  setServoAngle(servoPort, angle);
}