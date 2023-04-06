#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 280
#define SERVO_MIN_PULSE_WIDTH 600 
#define SERVO_MAX_PULSE_WIDTH 2400 
#define SERVO_STEP 1
#define SERVO_FREQUENCY 50

#define HIP_SERVO_PORT 4
#define HIP_NEUTRAL_ANGLE 90

#define THIGH_NEUTRAL_ANGLE 90

#define ANKLE_SERVO_PORT 4
#define ANKLE_MIN_ANGLE 80
#define ANKLE_NEUTRAL_ANGLE 120
#define ANKLE_MAX_ANGLE 170


void setup() {
  Serial.begin(9600);

  pwm.begin();

  pwm.setOscillatorFrequency(27000000);
  // Analog servos run at ~50 Hz updates
  pwm.setPWMFreq(SERVO_FREQUENCY);

  delay(10);
}

void loop() {

  setServoAngle(ANKLE_NEUTRAL_ANGLE);
}

void setServoAngle(int angle) {

  int pulseWide, analogValue;

  pulseWide = map(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);

  analogValue = int(float(pulseWide) / 1000000 * SERVO_FREQUENCY * 4096);
  
  Serial.print(analogValue);

  pwm.setPWM(ANKLE_SERVO_PORT, 0, analogValue);
}