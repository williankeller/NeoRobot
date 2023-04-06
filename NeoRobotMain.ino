#include "Servos.h"

void setup() {
  Serial.begin(9600);

  setupServos();

  delay(10);
}

void loop() {

  walking();
}

void walking() {
  // Set the number of steps to repeat
  int steps = 4;

  // Loop through the steps
  for (int i = 0; i < steps; i++) {

    // Move front right leg forward
    setAnkleAngle(105);

    delay(500);

    // Move front right leg backward
    setAnkleAngle(120);

    delay(1000);

    Serial.print("Step");
    Serial.println(i);
  }

  // Wait for a command to stop the loop
  while (Serial.available()) {
    if (Serial.read() == 's') {
      return;
    }
  }
}