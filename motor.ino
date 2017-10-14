/*
 * PoE Lab 3 (DC Motor Control)
 * Sam Myers, Toby Shapinsky
 */

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "pid.h"

const int IR_SENSOR_LEFT = A0;
const int IR_SENSOR_RIGHT = A1;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);

int initialSpeed = 2;

double iMax = 10;
double iMin = -10;

double Kp = 10;
double Ki = 0;
double Kd = 0;

int leftSetPoint = 25;
int rightSetPoint = 24; 

PIDState leftPID = {
  initialSpeed,
  0,
  Kp + 1,
  Ki,
  Kd
};

PIDState rightPID = {
  initialSpeed,
  0,
  Kp,
  Ki,
  Kd
};

double updatePID(PIDState *pid, int error, int position) {
  double pTerm = pid->pGain * error;

  pid->integrator += error;

  double iTerm = pid->iGain * pid->integrator;

  if (iTerm > iMax) iTerm = iMax;
  if (iTerm < iMin) iTerm = iMin;

  double dTerm = pid->dGain * (pid->position - position);

  pid->position = position;

  return pTerm + iTerm + dTerm;
}

void setup() {
  Serial.begin(115200);
  AFMS.begin();

  leftMotor->setSpeed(initialSpeed);
  rightMotor->setSpeed(initialSpeed);

  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
}

void parseCommand(String command) {
  PIDState *pid;

  switch (command[0]) {
    case 'L':
      pid = &leftPID;
      break;

    case 'R':
      pid = &rightPID;
      break;

    default:
      return;
  }

  String newValString = String(command);
  newValString.remove(0, 2);
  int newVal = newValString.toInt();

  switch (command[1]) {
    case 'P':
      pid->pGain = newVal;
      break;

    case 'I':
      pid->iGain = newVal;
      break;

    case 'D':
      pid->dGain = newVal;
      break;

    default:
      break;
  }
}

void writeCSVToSerial(int leftPosition, int rightPosition, double leftSpeed, double rightSpeed) {
  Serial.print(leftPosition);
  Serial.print(", ");
  Serial.print(rightPosition);
  Serial.print(", ");
  Serial.print(leftSpeed);
  Serial.print(", ");
  Serial.println(rightSpeed);
}

void loop() {
  if (Serial.available() > 0) {
    parseCommand(Serial.readStringUntil('\n'));
  }

  int leftPosition = analogRead(IR_SENSOR_LEFT);
  int rightPosition = analogRead(IR_SENSOR_RIGHT);

  int leftError = leftPosition - leftSetPoint;
  int rightError = rightPosition - rightSetPoint;

  double leftSpeed = updatePID(&leftPID, rightError, rightPosition) - initialSpeed;
  double rightSpeed = updatePID(&rightPID, leftError, leftPosition) - initialSpeed;

  leftMotor->setSpeed(leftSpeed);
  rightMotor->setSpeed(rightSpeed);

  writeCSVToSerial(leftPosition, rightPosition, leftSpeed, rightSpeed);
}
