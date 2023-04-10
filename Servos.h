#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Servo general settings
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 270
#define SERVO_MIN_PULSE 600
#define SERVO_MAX_PULSE 2400
#define SERVO_FREQUENCY 50

// Robot dimensions
const float UPPER_SEGMENT_LENGTH = 52.5;
const float MIDDLE_SEGMENT_LENGTH = 63.5;
const float LOWER_SEGMENT_LENGTH = 54.0;

struct LegSectionChannels {
  uint8_t channel;
  int minAngle;
  int midAngle;
  int maxAngle;
};

struct LegChannels {
  LegSectionChannels upper;
  LegSectionChannels middle;
  LegSectionChannels lower;
};

LegChannels frontLeftLeg = {
  {1, 70, 96, 120},  // Upper (Shoulder) servo on channel 1
  {2, 90, 110, 180}, // Middle (Thigh) servo on channel 2
  {3, 80, 120, 170}  // Lower (Knee) servo on channel 3
};

LegChannels frontRightLeg = {
  {5, 70, 100, 120}, // Upper (Shoulder) servo on channel 5
  {6, 90, 110, 180}, // Middle (Thigh) servo on channel 6
  {7, 80, 120, 170}  // Lower (Knee) servo on channel 7
};

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

void setLegSectionAngle(LegSectionChannels legSection, int angle) {
  // Check that the desired angle is within the valid range for the leg section
  if (angle < legSection.minAngle || angle > legSection.maxAngle) {
    Serial.print("Error: Invalid angle for channel ");
    Serial.print(legSection.channel);
    Serial.print(": Angle ");
    Serial.print(angle);
    Serial.print(" out of range. Valid range is ");
    Serial.print(legSection.minAngle);
    Serial.print(" to ");
    Serial.print(legSection.maxAngle);
    Serial.print(" degrees.");
    
    delay(1000);
  
    return;
  }
  setServoAngle(legSection.channel, angle);
}

void setServoAngleWithSpeed(uint8_t servoPort, int targetAngle, int speed) {
  int currentAngle = map(pwm.getPWM(servoPort), SERVO_MIN_PULSE, SERVO_MAX_PULSE, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  int angleIncrement = (targetAngle > currentAngle) ? 1 : -1;
  int incrementDelay = 1000 / abs(targetAngle - currentAngle) * (1000 / speed);

  for (int angle = currentAngle; angle != targetAngle; angle += angleIncrement) {
    setServoAngle(servoPort, angle);
    delay(incrementDelay);
  }

  setServoAngle(servoPort, targetAngle);
}