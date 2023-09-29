#include <Arduino.h>
#include <RPLidar.h>

RPLidar lidar;

// The PWM pin for control the speed of RPLIDAR's motor.
constexpr int RPLIDAR_MOTOR = 3;

constexpr float MAX_lidarDistanceCm_CM = 20.0;
constexpr int MAX_MOTOR_SPEED = 255;

void setup() {
  Serial.begin(9600);
  
  // Bind the RPLIDAR driver to the Arduino hardware serial
  lidar.begin(Serial1);

  pinMode(RPLIDAR_MOTOR, OUTPUT);
}

void loop() {

  if (IS_OK(lidar.waitPoint())) {
    float lidarDistanceCm = lidar.getCurrentPoint().distance / 10.0;
    float lidarAngleDeg = lidar.getCurrentPoint().angle;
    //bool  startBit = lidar.getCurrentPoint().startBit;
    byte  quality  = lidar.getCurrentPoint().quality;

    if (quality == 0 || lidarDistanceCm == 0.00) {
      return;
    }

    if (lidarDistanceCm <= MAX_lidarDistanceCm_CM) {
      Serial.println("Distance: " + String(lidarDistanceCm) + "cm - Angle: " + String(lidarAngleDeg));
    }
  } else {
    // Stop the rplidar motor
    analogWrite(RPLIDAR_MOTOR, 0);
    Serial.println("Nope :(");

    // try to detect RPLIDAR...
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
      // detected...
      lidar.startScan();

      // start motor rotating at max allowed speed
      analogWrite(RPLIDAR_MOTOR, MAX_MOTOR_SPEED);
      delay(1000);
    }
  }
}
