
#include "Kinematics.h"

void setup() {
  // Initialize serial and wai for port to open:
  Serial.begin(9600);
  // This delay gives the chance to wait for a Serial Monitor without blocking if none is found.
  delay(1500);

  setupKinematics();
}

void loop() {

  if (Serial.available()) {
    int angle = Serial.parseInt();

    if (angle != 0) {
      Serial.print("Angle: ");
      Serial.println(angle);
      setLegSectionAngle(frontRightLeg.middle, angle);
    }
  }
  

  //changeState(WALKING);
  
  //walkForward(5, GAIT_STEP_DURATION);
  //changeState(STANDING);
}
