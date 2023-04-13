#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Servo general settings
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 270
#define SERVO_MIN_PULSE 600
#define SERVO_MAX_PULSE 2400
#define SERVO_FREQUENCY 50

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

// Front Left leg (Channel, min range, mid range (stand pos), max range)
LegChannels frontLeftLeg = {
  {1, 70, 95, 120},  // Upper (Shoulder)
  {2, 90, 115, 180}, // Middle (Thigh)
  {3, 80, 115, 170}  // Lower (Knee)
};

// Front Right leg (Channel, min range, mid range (stand pos), max range)
LegChannels frontRightLeg = {
  {5, 120, 157, 180}, // Upper (Shoulder)
  {6, 50, 110, 135},  // Middle (Thigh)
  {7, 90, 135, 180}   // Lower (Knee)
};

// Rear Left leg (Channel, min range, mid range (stand pos), max range)
LegChannels rearLeftLeg = {
  {9, 100, 145, 180}, // Upper (Shoulder)
  {10, 80, 110, 170}, // Middle (Thigh)
  {11, 90, 125, 180}  // Lower (Knee)
};

// Rear Right leg (Channel, min range, mid range (stand pos), max range)
LegChannels rearRightLeg = {
  {13, 0, 270, 270}, // Upper (Shoulder)
  {14, 0, 270, 270},  // Middle (Thigh)
  {15, 0, 270, 270}   // Lower (Knee)
};

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setupServos() {
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
    Serial.println();
    
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