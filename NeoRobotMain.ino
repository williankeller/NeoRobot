#include "LunaLidar.h"
#include "Kinematics.h"


void setup() {
  // Initialize serial and wait for port to open:
  Serial.begin(9600);

  setupKinematics();
  //setupLidar();
}

// Initialize variables
int16_t tfDist = 0;  // Distance to object in centimeters
int16_t tfFlux = 0;  // Strength or quality of return signal
int16_t tfTemp = 0;  // Internal temperature of Lidar sensor chip

void loop() {

  //if (lidarData(tfDist, tfFlux, tfTemp)) {

  //Serial.print("Distance=");
  //Serial.print(tfDist);
  //Serial.print('\t');
  //Serial.print("Strength=");
  //Serial.print(tfFlux);
  //Serial.print('\t');
  //Serial.print("Temp=");
  //Serial.print(tfTemp);
  //Serial.print('\t');
  //Serial.print('\n');
  //(50);
  //}

  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    String leg = input.substring(0, input.indexOf(" "));
    String section = input.substring(input.indexOf(" ") + 1, input.lastIndexOf(" "));
    int angle = input.substring(input.lastIndexOf(" ") + 1).toInt();

    if (input == "stand") {
      setLegSectionAngle(frontLeftLeg.upper, frontLeftLeg.upper.midAngle);
      setLegSectionAngle(frontLeftLeg.middle, frontLeftLeg.middle.midAngle);
      setLegSectionAngle(frontLeftLeg.lower, frontLeftLeg.lower.midAngle);

      setLegSectionAngle(frontRightLeg.upper, frontRightLeg.upper.midAngle);
      setLegSectionAngle(frontRightLeg.middle, frontRightLeg.middle.midAngle);
      setLegSectionAngle(frontRightLeg.lower, frontRightLeg.lower.midAngle);

      setLegSectionAngle(rearLeftLeg.upper, rearLeftLeg.upper.midAngle);
      setLegSectionAngle(rearLeftLeg.middle, rearLeftLeg.middle.midAngle);
      setLegSectionAngle(rearLeftLeg.lower, rearLeftLeg.lower.midAngle);

      setLegSectionAngle(frontRightLeg.upper, frontRightLeg.upper.midAngle);
      setLegSectionAngle(frontRightLeg.middle, frontRightLeg.middle.midAngle);
      setLegSectionAngle(frontRightLeg.lower, frontRightLeg.lower.midAngle);
    }

    if (input == "sit") {
      setLegSectionAngle(frontRightLeg.upper, frontRightLeg.upper.midAngle);
      setLegSectionAngle(frontRightLeg.middle, frontRightLeg.middle.midAngle);
      setLegSectionAngle(frontRightLeg.lower, frontRightLeg.lower.midAngle);

      setLegSectionAngle(frontLeftLeg.upper, frontLeftLeg.upper.midAngle);
      setLegSectionAngle(frontLeftLeg.middle, frontLeftLeg.middle.midAngle);
      setLegSectionAngle(frontLeftLeg.lower, frontLeftLeg.lower.midAngle);
    }

    if (leg == "frontRight") {
      if (section == "upper") {
        setLegSectionAngle(frontRightLeg.upper, angle);
      } else if (section == "middle") {
        setLegSectionAngle(frontRightLeg.middle, angle);
      } else if (section == "lower") {
        setLegSectionAngle(frontRightLeg.lower, angle);
      }
    } else if (leg == "frontLeft") {
      if (section == "upper") {
        setLegSectionAngle(frontLeftLeg.upper, angle);
      } else if (section == "middle") {
        setLegSectionAngle(frontLeftLeg.middle, angle);
      } else if (section == "lower") {
        setLegSectionAngle(frontLeftLeg.lower, angle);
      }
    } else if (leg == "rearLeft") {
      if (section == "upper") {
        setLegSectionAngle(rearLeftLeg.upper, angle);
      } else if (section == "middle") {
        setLegSectionAngle(rearLeftLeg.middle, angle);
      } else if (section == "lower") {
        setLegSectionAngle(rearLeftLeg.lower, angle);
      }
    } else if (leg == "rearRight") {
      if (section == "upper") {
        setLegSectionAngle(rearRightLeg.upper, angle);
      } else if (section == "middle") {
        setLegSectionAngle(rearRightLeg.middle, angle);
      } else if (section == "lower") {
        setLegSectionAngle(rearRightLeg.lower, angle);
      }
    }
  }


  //changeState(WALKING);

  //walkForward(5, GAIT_STEP_DURATION);
  //changeState(STANDING);
}
