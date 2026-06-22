#include <Wire.h>
#include "mpu6050.hpp"

// =====================
// L298N #1 - LEFT Motor Driver
// =====================

// Left-front motor - Channel A
#define LEFT_FRONT_PWM   5
#define LEFT_FRONT_IN1   7
#define LEFT_FRONT_IN2   8

// Left-rear motor - Channel B
#define LEFT_REAR_PWM    6
#define LEFT_REAR_IN1    9
#define LEFT_REAR_IN2    10


// =====================
// L298N #2 - RIGHT Motor Driver
// =====================

// Right-front motor - Channel A
#define RIGHT_FRONT_PWM  3
#define RIGHT_FRONT_IN1  2
#define RIGHT_FRONT_IN2  4

// Right-rear motor - Channel B
#define RIGHT_REAR_PWM   11
#define RIGHT_REAR_IN1   12
#define RIGHT_REAR_IN2   13


// =====================
// Direction Calibration
// Your current working calibration
// =====================

#define LEFT_FRONT_DIR   1
#define LEFT_REAR_DIR    1
#define RIGHT_FRONT_DIR -1
#define RIGHT_REAR_DIR  -1


// =====================
// PID Settings
// =====================

float Kp = 3.0;
float Ki = 0.0;
float Kd = 0.4;

// If robot corrects in the wrong direction, change this to -1
int PID_SIGN = 1;

float targetYaw = 0.0;
float currentYaw = 0.0;

float gyroZOffset = 0.0;

float pidIntegral = 0.0;
float previousError = 0.0;

unsigned long previousTime = 0;

#define BASE_SPEED       140
#define MAX_CORRECTION   70

#define MOVE_TIME        5000
#define STOP_TIME        3000


// =====================
// MPU6050 Data
// =====================

MPU6050_Data_t rawData;
MPU6050_Scaled_t scaledData;


// =====================
// Setup
// =====================

void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(LEFT_FRONT_PWM, OUTPUT);
  pinMode(LEFT_FRONT_IN1, OUTPUT);
  pinMode(LEFT_FRONT_IN2, OUTPUT);

  pinMode(LEFT_REAR_PWM, OUTPUT);
  pinMode(LEFT_REAR_IN1, OUTPUT);
  pinMode(LEFT_REAR_IN2, OUTPUT);

  pinMode(RIGHT_FRONT_PWM, OUTPUT);
  pinMode(RIGHT_FRONT_IN1, OUTPUT);
  pinMode(RIGHT_FRONT_IN2, OUTPUT);

  pinMode(RIGHT_REAR_PWM, OUTPUT);
  pinMode(RIGHT_REAR_IN1, OUTPUT);
  pinMode(RIGHT_REAR_IN2, OUTPUT);

  stopAllMotors();

  Serial.println("Starting MPU6050...");

  uint8_t id = MPU6050_Check();
  Serial.print("MPU6050 WHO_AM_I: ");
  Serial.println(id, HEX);

  MPU6050_Init();

  delay(1000);

  Serial.println("Keep robot still. Calibrating gyro Z...");
  calibrateGyroZ();

  Serial.print("Gyro Z Offset: ");
  Serial.println(gyroZOffset);

  currentYaw = 0.0;
  targetYaw = 0.0;

  previousTime = millis();

  Serial.println("PID heading hold test started.");
}


// =====================
// Main Loop
// =====================

void loop() {
  Serial.println("Forward with IMU PID");

  resetPID();
  currentYaw = 0.0;
  targetYaw = 0.0;
  previousTime = millis();

  unsigned long startTime = millis();

  while (millis() - startTime < MOVE_TIME) {
    updateYaw();
    moveForwardPID(BASE_SPEED);
    delay(20);
  }

  Serial.println("Stop");
  stopAllMotors();
  delay(STOP_TIME);
}


// =====================
// IMU / Yaw Functions
// =====================

void calibrateGyroZ() {
  const int samples = 500;
  float sum = 0.0;

  for (int i = 0; i < samples; i++) {
    MPU6050_Read_All(&rawData);
    MPU6050_Get_Scaled(&rawData, &scaledData);

    sum += scaledData.gz;
    delay(5);
  }

  gyroZOffset = sum / samples;
}


void updateYaw() {
  unsigned long now = millis();
  float dt = (now - previousTime) / 1000.0;
  previousTime = now;

  if (dt <= 0.0 || dt > 0.2) {
    return;
  }

  MPU6050_Read_All(&rawData);
  MPU6050_Get_Scaled(&rawData, &scaledData);

  float correctedGz = scaledData.gz - gyroZOffset;

  currentYaw += correctedGz * dt;

  currentYaw = wrapAngle(currentYaw);
}


float wrapAngle(float angle) {
  while (angle > 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}


float angleError(float target, float current) {
  float error = target - current;
  return wrapAngle(error);
}


// =====================
// PID Function
// =====================

float computePID() {
  float error = angleError(targetYaw, currentYaw);

  pidIntegral += error * 0.02;
  pidIntegral = constrain(pidIntegral, -50.0, 50.0);

  float derivative = (error - previousError) / 0.02;
  previousError = error;

  float output = (Kp * error) + (Ki * pidIntegral) + (Kd * derivative);

  output = output * PID_SIGN;
  output = constrain(output, -MAX_CORRECTION, MAX_CORRECTION);

  Serial.print("Yaw: ");
  Serial.print(currentYaw);
  Serial.print(" | Error: ");
  Serial.print(error);
  Serial.print(" | Correction: ");
  Serial.println(output);

  return output;
}


void resetPID() {
  pidIntegral = 0.0;
  previousError = 0.0;
}


// =====================
// Motor Control
// =====================

void setMotor(int pwmPin, int in1Pin, int in2Pin, int speedValue) {
  speedValue = constrain(speedValue, -255, 255);

  if (speedValue > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, speedValue);
  }
  else if (speedValue < 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    analogWrite(pwmPin, -speedValue);
  }
  else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, 0);
  }
}


// =====================
// Calibrated Individual Motors
// =====================

void setLeftFront(int speedValue) {
  setMotor(LEFT_FRONT_PWM, LEFT_FRONT_IN1, LEFT_FRONT_IN2, speedValue * LEFT_FRONT_DIR);
}

void setLeftRear(int speedValue) {
  setMotor(LEFT_REAR_PWM, LEFT_REAR_IN1, LEFT_REAR_IN2, speedValue * LEFT_REAR_DIR);
}

void setRightFront(int speedValue) {
  setMotor(RIGHT_FRONT_PWM, RIGHT_FRONT_IN1, RIGHT_FRONT_IN2, speedValue * RIGHT_FRONT_DIR);
}

void setRightRear(int speedValue) {
  setMotor(RIGHT_REAR_PWM, RIGHT_REAR_IN1, RIGHT_REAR_IN2, speedValue * RIGHT_REAR_DIR);
}


// =====================
// PID Movement
// =====================

void moveForwardPID(int baseSpeed) {
  float correction = computePID();

  int frontSpeed = baseSpeed - correction;
  int rearSpeed  = baseSpeed + correction;

  frontSpeed = constrain(frontSpeed, -255, 255);
  rearSpeed  = constrain(rearSpeed, -255, 255);

  setLeftFront(frontSpeed);
  setRightFront(frontSpeed);

  setLeftRear(rearSpeed);
  setRightRear(rearSpeed);
}


// =====================
// Normal Movement Functions
// =====================

void moveForward(int speedValue) {
  setLeftFront(speedValue);
  setLeftRear(speedValue);
  setRightFront(speedValue);
  setRightRear(speedValue);
}

void moveBackward(int speedValue) {
  setLeftFront(-speedValue);
  setLeftRear(-speedValue);
  setRightFront(-speedValue);
  setRightRear(-speedValue);
}

void turnLeft(int speedValue) {
  setLeftFront(-speedValue);
  setLeftRear(speedValue);

  setRightFront(-speedValue);
  setRightRear(speedValue);
}

void turnRight(int speedValue) {
  setLeftFront(speedValue);
  setLeftRear(-speedValue);

  setRightFront(speedValue);
  setRightRear(-speedValue);
}

void stopAllMotors() {
  setLeftFront(0);
  setLeftRear(0);
  setRightFront(0);
  setRightRear(0);
}
