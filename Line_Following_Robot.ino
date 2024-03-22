#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
#include <SparkFun_TB6612.h>

bool isBlackLine = 1;    //Inputting Which coloured line bot needs to follow.
unsigned int lineThickness = 25;  //Enter the thickness of the line
unsigned int numSensors = 5;  //Enter the number of IR in IR array sensor 
bool brakeEnabled = 0;
//Connections between Nano and Motor driver
#define MOTOR_AIN1 4
#define MOTOR_BIN1 6
#define MOTOR_AIN2 3
#define MOTOR_BIN2 7
#define MOTOR_PWMA 9
#define MOTOR_PWMB 10
#define MOTOR_STBY 5

const int motorOffsetA = 1;
const int motorOffsetB = 1;

Motor motor1 = Motor(MOTOR_AIN1, MOTOR_AIN2, MOTOR_PWMA, motorOffsetA, MOTOR_STBY);
Motor motor2 = Motor(MOTOR_BIN1, MOTOR_BIN2, MOTOR_PWMB, motorOffsetB, MOTOR_STBY);

int proportional, derivative, integral, previousError, PIDValue, error;
int leftSpeed, rightSpeed;
int lfSpeed = 130;
int currentSpeed = 35;

float Kp = 0.06;
float Kd = 1.5;
float Ki = 0;

int onLine = 1;
int minValues[7], maxValues[7], threshold[7], sensorValue[7];
bool brakeFlag = 0;

void setup() {
  Serial.begin(9600);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  lineThickness = constrain(lineThickness, 10, 35);
}

void loop() {
  while (digitalRead(11)) {}
  delay(1000);
  calibrate();
  while (digitalRead(12)) {}
  delay(1000);

  while (true) {
    readLine();
    if (currentSpeed < lfSpeed) currentSpeed++;
    if (onLine == 1) {
      lineFollow();
      digitalWrite(13, HIGH);
      brakeFlag = 0;
    } else {
      offLineBehavior();
    }
  }
}

void lineFollow() {
  calculateError();
  calculatePID();
  adjustMotorSpeed();
}

void calculateError() {
  if (numSensors == 7) {
    error = (3 * sensorValue[0] + 2 * sensorValue[1] + sensorValue[2] - sensorValue[4] - 2 * sensorValue[5] - 3 * sensorValue[6]);
  } else if (numSensors == 5) {
    error = (3 * sensorValue[1] + sensorValue[2] - sensorValue[4] - 3 * sensorValue[5]);
  }
  if (lineThickness > 22 || isBlackLine) {
    error = error * -1;
  }
}

void calculatePID() {  // PID control logic
  proportional = error;
  integral += error;
  derivative = error - previousError;
  PIDValue = (Kp * proportional) + (Ki * integral) + (Kd * derivative);
  previousError = error;
}

void adjustMotorSpeed() {
  leftSpeed = currentSpeed - PIDValue;
  rightSpeed = currentSpeed + PIDValue;
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  motor1.drive(leftSpeed);
  motor2.drive(rightSpeed);
}

void offLineBehavior() {
  digitalWrite(13, LOW);
  if (error > 0) {
    offLineLeft();
  } else {
    offLineRight();
  }
}

void offLineLeft() {
  if (brakeEnabled && !brakeFlag) {
    brakeMotors();
  }
  motor1.drive(-100);
  motor2.drive(150);
  brakeFlag = 1;
}

void offLineRight() {
  if (brakeEnabled && !brakeFlag) {
    brakeMotors();
  }
  motor1.drive(150);
  motor2.drive(-100);
  brakeFlag = 1;
}

void brakeMotors() {
  motor1.drive(0);
  motor2.drive(0);
  delay(30);
}

void calibrate() {
  for (int i = 0; i < 7; i++) {
    minValues[i] = analogRead(i);
    maxValues[i] = analogRead(i);
  }
  for (int i = 0; i < 10000; i++) {
    motor1.drive(50);
    motor2.drive(-50);
    for (int i = 0; i < 7; i++) {
      int readValue = analogRead(i);
      minValues[i] = min(minValues[i], readValue);
      maxValues[i] = max(maxValues[i], readValue);
    }
  }
  for (int i = 0; i < 7; i++) {
    threshold[i] = (minValues[i    + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print(" ");
  }
  Serial.println();

  motor1.drive(0);
  motor2.drive(0);
}

void readLine() {
  onLine = 0;
  for (int i = 0; i < numSensors; i++) {
    sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 0, 1000);
    sensorValue[i] = constrain(sensorValue[i], 0, 1000);
    if ((isBlackLine && sensorValue[i] > threshold[i]) || (!isBlackLine && sensorValue[i] < threshold[i])) {
      onLine = 1;
    }
  }
}

