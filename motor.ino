/*
 * PoE Lab 3 (DC Motor Control)
 * Sam Myers, Toby Shapinsky
 */

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

const int IR_SENSOR_LEFT = A0;
const int IR_SENSOR_RIGHT = A1;

const int SENSOR_THRESHOLD = 40;

uint8_t motorSpeed = 150;
byte sensorReadings = B00;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);

byte readFromSensors() {
  // Serial.println(analogRead(IR_SENSOR_LEFT));
  // Serial.println(analogRead(IR_SENSOR_RIGHT));
  byte leftReading = analogRead(IR_SENSOR_LEFT) > SENSOR_THRESHOLD;
  byte rightReading = analogRead(IR_SENSOR_RIGHT) > SENSOR_THRESHOLD;
  return leftReading << 1 | rightReading;
}

void driveForward() {
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
}

void turnLeft() {
  leftMotor->run(RELEASE);
  rightMotor->run(FORWARD);
}

void turnRight() {
  leftMotor->run(FORWARD);
  rightMotor->run(RELEASE);
}

void stop() {
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}

void setup() {
  Serial.begin(9600);
  AFMS.begin();
  leftMotor->setSpeed(motorSpeed);
  rightMotor->setSpeed(motorSpeed);
}

void loop() {
  byte sensorReading = readFromSensors();
  Serial.println(String(sensorReading, BIN));
  switch (sensorReading) {
    case B11:
      driveForward();
      break;
    case B10:
      turnRight();
      break;
    case B01:
      turnLeft();
      break;
    default:
      stop();
      break;
  }
  delay(1000);
}
