#include "Servos.h"

void setup() {
  Serial.begin(9600);

  setupServos();
}

void loop() {

  setUpperAngle(UPPER_FL_CHANNEL, 115);
  //walking();
}

void walking() {
  // Set the number of steps to repeat
  int steps = 4;

  // Loop through the steps
  for (int i = 0; i < steps; i++) {

    // Move front right leg forward
    setUpperAngle(UPPER_FL_CHANNEL, 115);
    //setLowerAngle(105);
    //setMiddleAngle(MIDDLE_FL_CHANNEL, 120);
    

    delay(1000);

    // Move front right leg backward
    //setUpperAngle(UPPER_FL_CHANNEL, 270);    
    //setLowerAngle(115);
    //setMiddleAngle(MIDDLE_FL_CHANNEL, 110);

    delay(1000);

    Serial.print("Step");
    Serial.println(i);
  }
}