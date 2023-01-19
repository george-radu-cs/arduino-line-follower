#include <QTRSensors.h>

// pins
const int firstMotorPin1 = 7;
const int firstMotorPin2 = 6;
const int secondMotorPin1 = 5;
const int secondMotorPin2 = 4;
const int enableFirstMotorPin = 11;
const int enableSecondMotorPin = 10;
const int rightLedPin = 12;

bool rightLedPinOn = false;

int firstMotorSpeed = 0;
int secondMotorSpeed = 0;

// PID configuration values
const float kp = 12;
const float ki = 0;
const float kd = 8;
int p = 1;
int i = 0;
int d = 0;
// read error from the centered black line
int error = 0;
int lastError = 0;

const int maxSpeed = 225;
const int minSpeed = -225;
const int baseSpeed = 200;
const int straightLineDecreaseSpeedValue = 30;
const int turnPrecisionDivider = 10;
const int minBlockingSpeed = 0;
const int maxBlockingSpeed = 125;
const int minNonBlockingSpeed = 135;
const int nonWhiteValueCalibration = 400;
const int nonWhiteValueReturn = 400;
const int minStraightLineError = 3;

// reflectance sensor configuration
QTRSensors qtr;
const int sensorCount = 6;
int sensorValues[sensorCount];
int sensors[sensorCount] = {0, 0, 0, 0, 0, 0};

// reflectance error values
int minReadErrorValue = 0;
int maxReadErrorValue = 5000;
int minOutputErrorValue = -50;
int maxOutputErrorValue = 50;

void setup() {
  configureMotors();
  configureQtrSensors();
  calibrateQtr();

  Serial.begin(9600);
}

void loop() {
  updateDirection();
}

void configureMotors() {
  pinMode(firstMotorPin1, OUTPUT);
  pinMode(firstMotorPin2, OUTPUT);
  pinMode(secondMotorPin1, OUTPUT);
  pinMode(secondMotorPin2, OUTPUT);
  pinMode(enableFirstMotorPin, OUTPUT);
  pinMode(enableSecondMotorPin, OUTPUT);
}

void configureQtrSensors() {
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]) {A0, A1, A2, A3, A4, A5}, sensorCount);
  delay(500);
}

void calibrateQtr() {
  // turn on Arduino's LED to indicate we are in calibration mode
  // calibrate the sensor. For maximum grade the line follower should do
  // the movement itself, without human interaction.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // we use different motor speed in calibration mode
  int motorSpeed = 180;

  int lastTurnDirection = 1;
  int sensorCalibrationValues[6];
  int rotationCount = 9;
  int currentRotations = 0;
  // are all sensors outside of the black line?
  bool allWhite = false;
  // during last iteration were all sensors on white?
  bool lastWasWhite = false;

  setMotorSpeed(baseSpeed * -1, baseSpeed);
  while (currentRotations < rotationCount) {
    qtr.read(sensorCalibrationValues);
    qtr.calibrate();
    allWhite = true;
    for (byte i = 0; i < sensorCount; i++) {
      // checks if any of the line-follower sensors are on the black-line
      // (all sensors are checked to prevent possible issues)
      if (sensorCalibrationValues[i] > nonWhiteValueCalibration) {
        lastWasWhite = false;
        allWhite = false;
        break;
      }
    }

    // continue until the line-follower is on black
    if (lastWasWhite) {
      continue;
    }

    // the robot moved over the black line and it must change its direction
    if (allWhite) {
      // debugging during direction and makes the robot look cooler
      if (!rightLedPinOn) {
        digitalWrite(rightLedPin, HIGH);
        rightLedPinOn = true;
      } else {
        digitalWrite(rightLedPin, LOW);
        rightLedPinOn = false;
      }

      currentRotations++;
      setMotorSpeed(motorSpeed * lastTurnDirection, motorSpeed * lastTurnDirection * -1);
      lastTurnDirection *= -1;
      lastWasWhite = true;
    }
  }

  // the line follower needs to return to the black line after calibration
  bool isStillOnWhite = true;
  while (isStillOnWhite) {
    qtr.read(sensorCalibrationValues);
    if (sensorCalibrationValues[2] > nonWhiteValueReturn) {
      isStillOnWhite = false;
      break;
    }
  }

  // stop to the robot after calibration step
  setMotorSpeed(0, 0);
  digitalWrite(LED_BUILTIN, LOW);
}

void updateDirection() {
  // read the state of the robot compared on the line, compute the p, i & d
  int error = readRobotError();
  updatePIDValues(error);

  // computes the motor speed differentiator
  int motorSpeedDiff = computeMotorSpeedDiff(kp, ki, kd, p, i, d);
  firstMotorSpeed = baseSpeed;
  secondMotorSpeed = baseSpeed;

  // a bit counterintuitive because of the signs basically add the negative in the 2nd if you add the error to
  // secondMotorSpeed (you subtract the negative) it's just the way the values of the sensors and/or motors lined up
  // for better precision on turns lower/raise the other motor speed by a proportion of the diff
  if (error < minStraightLineError) {
    firstMotorSpeed += motorSpeedDiff;
    secondMotorSpeed -= motorSpeedDiff / turnPrecisionDivider;
  } else if (error > minStraightLineError) {
    secondMotorSpeed -= motorSpeedDiff;
    firstMotorSpeed += motorSpeedDiff / turnPrecisionDivider;
  } else {
    // for straight lines decrease the speed
    firstMotorSpeed -= straightLineDecreaseSpeedValue;
    secondMotorSpeed -= straightLineDecreaseSpeedValue;
  }

  // when robot has low speed on both motors, the wheels will lose adhesion, so we increase the speed of both of them
  // values taken empirically
  if (firstMotorSpeed < maxBlockingSpeed && secondMotorSpeed < maxBlockingSpeed && firstMotorSpeed > minBlockingSpeed &&
      secondMotorSpeed > minBlockingSpeed) {
    firstMotorSpeed = minNonBlockingSpeed;
    secondMotorSpeed = minNonBlockingSpeed;
  }
  if (firstMotorSpeed > -1 * maxBlockingSpeed && secondMotorSpeed > -1 * maxBlockingSpeed &&
      firstMotorSpeed < minBlockingSpeed && secondMotorSpeed < minBlockingSpeed) {
    firstMotorSpeed = -1 * minNonBlockingSpeed;
    secondMotorSpeed = -1 * minNonBlockingSpeed;
  }

  // make sure the motor speed doesn't go over/under limit. The limit accepted of the motors are -255 & 255
  firstMotorSpeed = constrain(firstMotorSpeed, minSpeed, maxSpeed);
  secondMotorSpeed = constrain(secondMotorSpeed, minSpeed, maxSpeed);
  setMotorSpeed(firstMotorSpeed, secondMotorSpeed);
}

int readRobotError() {
  return map(qtr.readLineBlack(sensorValues), minReadErrorValue, maxReadErrorValue, minOutputErrorValue,
             maxOutputErrorValue);
}

void updatePIDValues(int error) {
  // since we run in a fast loop, update the i & d based on the error after a few steps
  static int numberStepsForLastIUpdate = 0;
  static int numberStepsForLastDUpdate = 0;
  static const int maxStepsI = 5;
  static const int maxStepsD = 5;

  p = error;

  numberStepsForLastIUpdate += 1;
  if (numberStepsForLastIUpdate > maxStepsI) {
    i = i + error;
    numberStepsForLastIUpdate = 0;
  }

  numberStepsForLastDUpdate += 1;
  if (numberStepsForLastDUpdate > maxStepsD) {
    d = error - lastError;
    lastError = error;
    numberStepsForLastDUpdate = 0;
  }
}

// calculate PID value based on error, kp, kd, ki, p, i and d.
int computeMotorSpeedDiff(float kp, float ki, float kd, int p, int i, int d) {
  return int(kp * p + ki * i + kd * d);
}

// each argument takes values between -255 and 255. The negative values
// represent the motor speed in reverse.
void setMotorSpeed(int motor1Speed, int motor2Speed) {
  // remove comment if any of the motors are going in reverse
  // motor1Speed = -motor1Speed;
  // motor2Speed = -motor2Speed;
  if (motor1Speed == 0) {
    digitalWrite(firstMotorPin1, LOW);
    digitalWrite(firstMotorPin2, LOW);
    analogWrite(enableFirstMotorPin, motor1Speed);
  } else {
    if (motor1Speed > 0) {
      digitalWrite(firstMotorPin1, HIGH);
      digitalWrite(firstMotorPin2, LOW);
      analogWrite(enableFirstMotorPin, motor1Speed);
    }
    if (motor1Speed < 0) {
      digitalWrite(firstMotorPin1, LOW);
      digitalWrite(firstMotorPin2, HIGH);
      analogWrite(enableFirstMotorPin, -motor1Speed);
    }
  }
  if (motor2Speed == 0) {
    digitalWrite(secondMotorPin1, LOW);
    digitalWrite(secondMotorPin2, LOW);
    analogWrite(enableSecondMotorPin, motor2Speed);
  } else {
    if (motor2Speed > 0) {
      digitalWrite(secondMotorPin1, HIGH);
      digitalWrite(secondMotorPin2, LOW);
      analogWrite(enableSecondMotorPin, motor2Speed);
    }
    if (motor2Speed < 0) {
      digitalWrite(secondMotorPin1, LOW);
      digitalWrite(secondMotorPin2, HIGH);
      analogWrite(enableSecondMotorPin, -motor2Speed);
    }
  }
}