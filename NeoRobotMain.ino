#include "Servos.h"

void setup() {
  Serial.begin(9600);

  setupServos();
}

void loop() {

  //setLegSectionAngle(frontLeftLeg.lower, frontLeftLeg.lower.midAngle);
  setLegSectionAngle(frontLeftLeg.upper, frontLeftLeg.upper.midAngle);
  //walking();
}

void walking() {
  // Set the number of steps to repeat
  int steps = 4;

  // Loop through the steps
  for (int i = 0; i < steps; i++) {

    // Move front right leg forward
    setLegSectionAngle(frontLeftLeg.upper, 115);
    //setLowerAngle(105);
    //setMiddleAngle(MIDDLE_FL_CHANNEL, 120);
    

    delay(1000);

    // Move front right leg backward
    setLegSectionAngle(frontLeftLeg.upper, 270);    

    delay(1000);

    Serial.print("Step");
    Serial.println(i);
  }
}