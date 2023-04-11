#include <math.h>
#include "Servos.h"

#define PI 3.1415926535897932384626433832795

// Gait settings
#define GAIT_STEP_DURATION 500  // milliseconds
#define GAIT_STEP_HEIGHT 20     // millimeters
#define GAIT_STEP_DISTANCE 30   // millimeters

// Robot dimensions
const float UPPER_SEGMENT_LENGTH = 52.5;
const float MIDDLE_SEGMENT_LENGTH = 63.5;
const float LOWER_SEGMENT_LENGTH = 54.0;

// Robot states
enum RobotState { STANDING, WALKING };

RobotState currentState = STANDING;

// Forward declarations
void setLegPosition(LegChannels leg, float x, float y, float z);
void setLegAngles(LegChannels leg, int shoulderAngle, int thighAngle, int kneeAngle);
void moveLeg(LegChannels leg, float xOffset, float yOffset, float zOffset, int duration);
void applyGaitCycle(LegChannels leg, int duration);
void walkForward(int steps, int duration);
void maintainPosture();
void changeState(RobotState newState);

void setupKinematics() {
  setupServos();
  maintainPosture();
}

void setLegPosition(LegChannels leg, float x, float y, float z) {
  float L1 = UPPER_SEGMENT_LENGTH;
  float L2 = MIDDLE_SEGMENT_LENGTH;
  float L3 = LOWER_SEGMENT_LENGTH;

  float alpha, beta, gamma;

  // Inverse kinematics equations
  alpha = atan2(y, x);
  float A = -z;
  float B = L1 - sqrt(x * x + y * y);
  float C = (A * A + B * B - L2 * L2 - L3 * L3) / (2 * L2 * L3);
  gamma = atan2(sqrt(1 - C * C), C);
  float D = atan2(L3 * sin(gamma), L2 + L3 * cos(gamma));
  beta = atan2(B - A * tan(D), A + B * tan(D)) - D;

  // Convert radians to degrees
  int shoulderAngle = int(alpha * 180 / PI);
  int thighAngle = int(beta * 180 / PI);
  int kneeAngle = int(gamma * 180 / PI);

  Serial.println("DEBUG: Inverse kinematics");
  Serial.println("- Equations: ");
  Serial.print("  alpha: ");
  Serial.print(alpha);
  Serial.print(" | beta: ");
  Serial.print(beta);
  Serial.print(" | gama: ");
  Serial.println(gamma);
  Serial.println("- Convert radians to degrees: ");
  Serial.print("  shoulder: ");
  Serial.print(shoulderAngle);
  Serial.print(" | thigh: ");
  Serial.print(thighAngle);
  Serial.print(" | knee: ");
  Serial.println(kneeAngle);
  Serial.println("-------------------------------------------");

  setLegAngles(leg, shoulderAngle, thighAngle, kneeAngle);
}

void setLegAngles(LegChannels leg, int shoulderAngle, int thighAngle, int kneeAngle) {
  setLegSectionAngle(leg.upper, shoulderAngle);
  setLegSectionAngle(leg.middle, thighAngle);
  setLegSectionAngle(leg.lower, kneeAngle);
}

void moveLeg(LegChannels leg, float xOffset, float yOffset, float zOffset, int duration) {
  int shoulderAngle = map(pwm.getPWM(leg.upper.channel), SERVO_MIN_PULSE, SERVO_MAX_PULSE, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  int thighAngle = map(pwm.getPWM(leg.middle.channel), SERVO_MIN_PULSE, SERVO_MAX_PULSE, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  int kneeAngle = map(pwm.getPWM(leg.lower.channel), SERVO_MIN_PULSE, SERVO_MAX_PULSE, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

  float x = UPPER_SEGMENT_LENGTH * cos(shoulderAngle * PI / 180) + MIDDLE_SEGMENT_LENGTH * cos((shoulderAngle + thighAngle) * PI / 180) + LOWER_SEGMENT_LENGTH * cos((shoulderAngle + thighAngle + kneeAngle) * PI / 180);
  float y = UPPER_SEGMENT_LENGTH * sin(shoulderAngle * PI / 180);
  float z = MIDDLE_SEGMENT_LENGTH * sin((shoulderAngle + thighAngle) * PI / 180) + LOWER_SEGMENT_LENGTH * sin((shoulderAngle + thighAngle + kneeAngle) * PI / 180);

  float targetX = x + xOffset;
  float targetY = y + yOffset;
  float targetZ = z + zOffset;

  setLegPosition(leg, targetX, targetY, targetZ);
}

void applyGaitCycle(LegChannels leg, int duration) {
  moveLeg(leg, GAIT_STEP_DISTANCE / 2, 0, GAIT_STEP_HEIGHT, duration / 2);
  moveLeg(leg, GAIT_STEP_DISTANCE / 2, 0, -GAIT_STEP_HEIGHT, duration / 2);
}

void walkForward(int steps, int duration) {
  for (int step = 0; step < steps; step++) {
    applyGaitCycle(frontLeftLeg, duration);
    applyGaitCycle(frontRightLeg, duration);
  }
}

void maintainPosture() {
  setLegAngles(frontLeftLeg, frontLeftLeg.upper.midAngle, frontLeftLeg.middle.midAngle, frontLeftLeg.lower.midAngle);
  setLegAngles(frontRightLeg, frontRightLeg.upper.midAngle, frontRightLeg.middle.midAngle, frontRightLeg.lower.midAngle);
}

void changeState(RobotState newState) {
  if (currentState == newState) {
    return;
  }

  currentState = newState;

  switch (currentState) {
    case STANDING:
      maintainPosture();
      break;
    case WALKING:
      break;
  }
}
