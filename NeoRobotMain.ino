#include "LunaLidar.h"
#include "Kinematics.h"


void setup() {
  // Initialize serial and wait for port to open:
  Serial.begin(9600);

  //setupKinematics();
  setupLidar();
}

// Initialize variables
int16_t tfDist = 0;  // Distance to object in centimeters
int16_t tfFlux = 0;  // Strength or quality of return signal
int16_t tfTemp = 0;  // Internal temperature of Lidar sensor chip

void loop() {

  if (lidarData(tfDist, tfFlux, tfTemp)) {

    Serial.print("Distance=");
    Serial.print(tfDist);
    Serial.print('\t');
    Serial.print("Strength=");
    Serial.print(tfFlux);
    Serial.print('\t');
    Serial.print("Temp=");
    Serial.print(tfTemp);
    Serial.print('\t');
    Serial.print('\n');
    delay(50);
  }


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
