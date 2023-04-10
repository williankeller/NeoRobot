#include "Kinematics.h"

void setup() {
  Serial.begin(9600);

  setupKinematics();
}

void loop() {

  changeState(WALKING);
  walkForward(5, GAIT_STEP_DURATION);
  changeState(STANDING);
}

