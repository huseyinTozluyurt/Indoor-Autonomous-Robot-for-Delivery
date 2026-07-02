// ======================================================
// DeliveryBot Yaw Threshold Recovery Code
// Arduino Uno + 2x L298N + 4 DC Motors + MPU6050
//
// IMPORTANT STRATEGY:
//   1) Normal forward/backward movement uses your proven trim-only logic.
//   2) MPU6050 does NOT continuously change PWM.
//   3) MPU6050 only watches accumulated heading/yaw error.
//   4) If yaw error breaches threshold, robot stops for 2 seconds,
//      turns slightly left/right based on yaw sign, pauses,
//      then continues the remaining forward/backward time.
//
// Commands:
//   F left_speed right_speed duration_ms
//   B left_speed right_speed duration_ms
//   L left_speed right_speed duration_ms
//   R left_speed right_speed duration_ms
//   S
//
// Example:
//   F 200 200 20000
//
// For a 20-second forward command, correction-turn time does NOT count
// as forward time. If correction happens at 12s, robot later completes
// the remaining 8s of forward movement.
// ======================================================

#include <Wire.h>
#include "mpu6050.hpp"

DeliveryBotMPU6050 imu;

// =====================
// Physical LEFT Side L298N
// =====================
#define LEFT_A_PWM   5
#define LEFT_A_IN1   7
#define LEFT_A_IN2   8

#define LEFT_B_PWM   3
#define LEFT_B_IN1   2
#define LEFT_B_IN2   4

// =====================
// Physical RIGHT Side L298N
// =====================
#define RIGHT_A_PWM  6
#define RIGHT_A_IN1  9
#define RIGHT_A_IN2  10

#define RIGHT_B_PWM  11
#define RIGHT_B_IN1  12
#define RIGHT_B_IN2  13

// =====================
// Direction Calibration
// =====================
#define LEFT_A_DIR    1
#define LEFT_B_DIR   -1

#define RIGHT_A_DIR   1
#define RIGHT_B_DIR  -1

// =====================
// Straight Movement Calibration
// =====================
// KEEP THESE VALUES. These are your working trim-only values.
#define STRAIGHT_LEFT_TRIM    20
#define STRAIGHT_RIGHT_TRIM   -10

// =====================
// Turning Calibration
// =====================
// KEEP THESE VALUES. Your current turns are already close/equal.
#define TURN_LEFT_LEFT_TRIM       0
#define TURN_LEFT_RIGHT_TRIM      0

#define TURN_RIGHT_LEFT_TRIM    -38
#define TURN_RIGHT_RIGHT_TRIM   -38

// =====================
// Segmented Yaw Correction Settings
// =====================
#define USE_SEGMENTED_YAW_CORRECTION 1

// Set this after hand yaw-direction test:
//   If turning robot LEFT by hand makes yaw INCREASE, keep +1.
//   If turning robot LEFT by hand makes yaw DECREASE, set -1.
// After this multiplication, positive headingError means:
//   robot has turned LEFT too much -> correction should turn RIGHT.
#define YAW_LEFT_POSITIVE_SIGN 1

// Do not correct small/noisy yaw changes.
#define YAW_CORRECTION_THRESHOLD_DEG  5.0f

// Do not allow correction immediately after command start.
#define MIN_DRIVE_TIME_BEFORE_CORRECTION_MS 1500UL

// Do not allow repeated correction too quickly.
#define MIN_TIME_BETWEEN_CORRECTIONS_MS 3000UL

// Small stop/pause times around correction.
#define CORRECTION_STOP_BEFORE_MS 2000UL
#define CORRECTION_PAUSE_AFTER_MS 300UL

// Correction turn parameters.
#define CORRECTION_TURN_PWM        200
#define CORRECTION_TURN_MIN_MS     160UL
#define CORRECTION_TURN_MAX_MS     450UL
#define CORRECTION_TURN_MS_PER_DEG 30.0f

// Safety: avoid endless stop-turn-stop behavior.
#define MAX_CORRECTIONS_PER_COMMAND 5

// Debug: set 1 only while diagnosing. Keep 0 for accurate timing.
#define ENABLE_CORRECTION_DEBUG 1
#define DEBUG_PRINT_PERIOD_MS 500UL

// =====================
// Command State
// =====================
enum RobotCommand {
  CMD_STOP = 0,
  CMD_FORWARD,
  CMD_BACKWARD,
  CMD_LEFT,
  CMD_RIGHT
};

enum MotionState {
  MOTION_IDLE = 0,
  MOTION_DRIVE,
  MOTION_CORRECTION_STOP_BEFORE,
  MOTION_CORRECTION_TURN,
  MOTION_CORRECTION_PAUSE_AFTER
};

RobotCommand activeCommand = CMD_STOP;
MotionState motionState = MOTION_IDLE;

int commandLeftSpeed = 0;
int commandRightSpeed = 0;
unsigned long commandDuration = 0;

// For simple open-loop turns L/R.
unsigned long commandStartTime = 0;

// For segmented F/B movement.
unsigned long completedDriveMs = 0;
unsigned long driveSegmentStartMs = 0;
unsigned long stateStartMs = 0;
unsigned long lastCorrectionEndMs = 0;
unsigned long correctionTurnDurationMs = 0;
int correctionTurnDirection = 0; // -1 = left, +1 = right
int correctionCount = 0;
float targetYawDeg = 0.0f;
float correctionTriggerErrorDeg = 0.0f;

unsigned long lastDebugPrintMs = 0;

// =====================
// Function Prototypes
// =====================
void handleSerialCommand();
void updateActiveCommand();
void updateStraightSegmentedMotion();
void updateSimpleTimedTurn();

void startCommand(RobotCommand cmd, int leftSpeedValue, int rightSpeedValue, unsigned long durationValue);
void resetSegmentedMotion();
void startCorrectionTurn(float headingErrorDeg);

float normalizeAngleDeg(float angleDeg);
float getHeadingErrorDeg();
unsigned long computeCorrectionTurnDuration(float absHeadingErrorDeg);

void setMotor(int pwmPin, int in1Pin, int in2Pin, int speedValue);

void setLeftA(int speedValue);
void setLeftB(int speedValue);
void setRightA(int speedValue);
void setRightB(int speedValue);

void setPhysicalLeftSide(int speedValue);
void setPhysicalRightSide(int speedValue);

int applyTrim(int speedValue, int trimValue);
int applySignedTrim(int speedValue, int trimValue);

void moveForwardTrimOnly(int leftSpeed, int rightSpeed);
void moveBackwardTrimOnly(int leftSpeed, int rightSpeed);
void turnLeft(int leftSpeed, int rightSpeed);
void turnRight(int leftSpeed, int rightSpeed);
void stopAllMotors();

// =====================
// Setup
// =====================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(30);

  pinMode(LEFT_A_PWM, OUTPUT);
  pinMode(LEFT_A_IN1, OUTPUT);
  pinMode(LEFT_A_IN2, OUTPUT);

  pinMode(LEFT_B_PWM, OUTPUT);
  pinMode(LEFT_B_IN1, OUTPUT);
  pinMode(LEFT_B_IN2, OUTPUT);

  pinMode(RIGHT_A_PWM, OUTPUT);
  pinMode(RIGHT_A_IN1, OUTPUT);
  pinMode(RIGHT_A_IN2, OUTPUT);

  pinMode(RIGHT_B_PWM, OUTPUT);
  pinMode(RIGHT_B_IN1, OUTPUT);
  pinMode(RIGHT_B_IN2, OUTPUT);

  stopAllMotors();

  Wire.begin();

  Serial.println("DELIVERYBOT_YAW_THRESHOLD_RECOVERY_BOOT");
  Serial.println("TRIM_ONLY_MOVEMENT_PRESERVED");
  Serial.println("MPU6050_USED_AS_YAW_THRESHOLD_RECOVERY_TRIGGER_ONLY");

  DeliveryBotMPU6050::InitStatus imuStatus = imu.begin();
  if (imuStatus != DeliveryBotMPU6050::INIT_OK) {
    Serial.print("IMU_INIT_FAILED_STATUS_");
    Serial.println((int)imuStatus);
    Serial.println("CHECK_WIRING: VCC GND SDA=A4 SCL=A5");
    while (true) {
      stopAllMotors();
      delay(1000);
    }
  }

  Serial.print("WHO_AM_I: 0x");
  Serial.println(imu.getWhoAmI(), HEX);
  Serial.println("Keep robot completely still. Calibrating gyro...");

  if (!imu.calibrateGyro(1000, 2)) {
    Serial.println("GYRO_CALIBRATION_FAILED");
    while (true) {
      stopAllMotors();
      delay(1000);
    }
  }

  imu.resetYaw(0.0f);

  Serial.println("GYRO_CALIBRATION_DONE");
  Serial.println("STRAIGHT_LEFT_TRIM_20_RIGHT_TRIM_MINUS_10");
  Serial.println("RIGHT_TURN_SLOWED_BY_38_PWM");
  Serial.println("READY");
  Serial.println("Commands: F/B/L/R left_speed right_speed duration_ms | S");
}

// =====================
// Main Loop
// =====================
void loop() {
  imu.update();
  handleSerialCommand();
  updateActiveCommand();
}

// =====================
// Serial Command Handling
// =====================
void handleSerialCommand() {
  if (!Serial.available()) {
    return;
  }

  String line = Serial.readStringUntil('\n');
  line.trim();

  if (line.length() == 0) {
    return;
  }

  char cmd = line.charAt(0);

  if (cmd == 'S' || cmd == 's') {
    startCommand(CMD_STOP, 0, 0, 0);
    Serial.println("ACK STOP");
    return;
  }

  int leftSpeedValue = 0;
  int rightSpeedValue = 0;
  unsigned long durationValue = 0;

  int parsed = sscanf(line.c_str(), "%c %d %d %lu", &cmd, &leftSpeedValue, &rightSpeedValue, &durationValue);

  if (parsed < 4) {
    Serial.print("ERR BAD_COMMAND: ");
    Serial.println(line);
    return;
  }

  leftSpeedValue = constrain(leftSpeedValue, 0, 255);
  rightSpeedValue = constrain(rightSpeedValue, 0, 255);

  switch (cmd) {
    case 'F':
    case 'f':
      startCommand(CMD_FORWARD, leftSpeedValue, rightSpeedValue, durationValue);
      Serial.println("ACK FORWARD_SEGMENTED_CORRECTION");
      break;

    case 'B':
    case 'b':
      startCommand(CMD_BACKWARD, leftSpeedValue, rightSpeedValue, durationValue);
      Serial.println("ACK BACKWARD_SEGMENTED_CORRECTION");
      break;

    case 'L':
    case 'l':
      startCommand(CMD_LEFT, leftSpeedValue, rightSpeedValue, durationValue);
      Serial.println("ACK LEFT_OPEN_LOOP");
      break;

    case 'R':
    case 'r':
      startCommand(CMD_RIGHT, leftSpeedValue, rightSpeedValue, durationValue);
      Serial.println("ACK RIGHT_OPEN_LOOP");
      break;

    default:
      Serial.print("ERR UNKNOWN_COMMAND: ");
      Serial.println(line);
      break;
  }
}

void startCommand(RobotCommand cmd, int leftSpeedValue, int rightSpeedValue, unsigned long durationValue) {
  stopAllMotors();

  activeCommand = cmd;
  commandLeftSpeed = leftSpeedValue;
  commandRightSpeed = rightSpeedValue;
  commandDuration = durationValue;
  commandStartTime = millis();

  if (cmd == CMD_FORWARD || cmd == CMD_BACKWARD) {
    resetSegmentedMotion();
  }
  else if (cmd == CMD_LEFT || cmd == CMD_RIGHT) {
    motionState = MOTION_DRIVE;
  }
  else {
    motionState = MOTION_IDLE;
  }
}

void resetSegmentedMotion() {
  imu.update();
  targetYawDeg = imu.getYawDeg();

  completedDriveMs = 0;
  driveSegmentStartMs = millis();
  stateStartMs = millis();
  lastCorrectionEndMs = 0;
  correctionTurnDurationMs = 0;
  correctionTurnDirection = 0;
  correctionCount = 0;
  correctionTriggerErrorDeg = 0.0f;
  motionState = MOTION_DRIVE;

#if ENABLE_CORRECTION_DEBUG
  Serial.print("TARGET_YAW_SET: ");
  Serial.println(targetYawDeg, 2);
#endif
}

// =====================
// Active Command Execution
// =====================
void updateActiveCommand() {
  if (activeCommand == CMD_STOP) {
    motionState = MOTION_IDLE;
    stopAllMotors();
    return;
  }

  if (activeCommand == CMD_FORWARD || activeCommand == CMD_BACKWARD) {
    updateStraightSegmentedMotion();
    return;
  }

  if (activeCommand == CMD_LEFT || activeCommand == CMD_RIGHT) {
    updateSimpleTimedTurn();
    return;
  }
}

void updateSimpleTimedTurn() {
  if (commandDuration > 0 && millis() - commandStartTime >= commandDuration) {
    activeCommand = CMD_STOP;
    motionState = MOTION_IDLE;
    stopAllMotors();
    Serial.println("DONE");
    return;
  }

  if (activeCommand == CMD_LEFT) {
    turnLeft(commandLeftSpeed, commandRightSpeed);
  }
  else if (activeCommand == CMD_RIGHT) {
    turnRight(commandLeftSpeed, commandRightSpeed);
  }
}

void updateStraightSegmentedMotion() {
  unsigned long now = millis();

  switch (motionState) {
    case MOTION_DRIVE: {
      unsigned long segmentElapsedMs = now - driveSegmentStartMs;

      if (commandDuration > 0 && completedDriveMs + segmentElapsedMs >= commandDuration) {
        activeCommand = CMD_STOP;
        motionState = MOTION_IDLE;
        stopAllMotors();
        Serial.println("DONE");
        return;
      }

      if (activeCommand == CMD_FORWARD) {
        moveForwardTrimOnly(commandLeftSpeed, commandRightSpeed);
      }
      else if (activeCommand == CMD_BACKWARD) {
        moveBackwardTrimOnly(commandLeftSpeed, commandRightSpeed);
      }

#if USE_SEGMENTED_YAW_CORRECTION
      float headingErrorDeg = getHeadingErrorDeg();
      float absError = fabs(headingErrorDeg);

#if ENABLE_CORRECTION_DEBUG
      if (now - lastDebugPrintMs >= DEBUG_PRINT_PERIOD_MS) {
        lastDebugPrintMs = now;
        Serial.print("DRIVE | Target:");
        Serial.print(targetYawDeg, 2);
        Serial.print(" | Yaw:");
        Serial.print(imu.getYawDeg(), 2);
        Serial.print(" | Error:");
        Serial.print(headingErrorDeg, 2);
        Serial.print(" | AbsErr:");
        Serial.print(absError, 2);
        Serial.print(" | Threshold:");
        Serial.print(YAW_CORRECTION_THRESHOLD_DEG, 2);
        Serial.print(" | DoneMs:");
        Serial.print(completedDriveMs + segmentElapsedMs);
        Serial.print("/");
        Serial.println(commandDuration);
      }
#endif

      bool enoughTimeFromStart = (completedDriveMs + segmentElapsedMs) >= MIN_DRIVE_TIME_BEFORE_CORRECTION_MS;
      bool enoughTimeSinceCorrection = (lastCorrectionEndMs == 0) || ((now - lastCorrectionEndMs) >= MIN_TIME_BETWEEN_CORRECTIONS_MS);
      bool correctionLimitOk = correctionCount < MAX_CORRECTIONS_PER_COMMAND;
      bool yawThresholdBreached = absError >= YAW_CORRECTION_THRESHOLD_DEG;

      // Main non-PID correction condition:
      // Keep trim-only movement until yaw breaches the threshold.
      // Then stop for 2 seconds, turn opposite direction, pause, and resume remaining movement.
      if (yawThresholdBreached && enoughTimeFromStart && enoughTimeSinceCorrection && correctionLimitOk) {
        completedDriveMs += segmentElapsedMs;
        startCorrectionTurn(headingErrorDeg);
        return;
      }
#endif
      break;
    }

    case MOTION_CORRECTION_STOP_BEFORE:
      stopAllMotors();
      if (now - stateStartMs >= CORRECTION_STOP_BEFORE_MS) {
        motionState = MOTION_CORRECTION_TURN;
        stateStartMs = now;
      }
      break;

    case MOTION_CORRECTION_TURN:
      if (correctionTurnDirection < 0) {
        turnLeft(CORRECTION_TURN_PWM, CORRECTION_TURN_PWM);
      }
      else if (correctionTurnDirection > 0) {
        turnRight(CORRECTION_TURN_PWM, CORRECTION_TURN_PWM);
      }
      else {
        stopAllMotors();
      }

      if (now - stateStartMs >= correctionTurnDurationMs) {
        stopAllMotors();
        motionState = MOTION_CORRECTION_PAUSE_AFTER;
        stateStartMs = now;
      }
      break;

    case MOTION_CORRECTION_PAUSE_AFTER:
      stopAllMotors();
      if (now - stateStartMs >= CORRECTION_PAUSE_AFTER_MS) {
        // Keep original targetYawDeg. This prevents the robot from accepting a bad heading.
        motionState = MOTION_DRIVE;
        driveSegmentStartMs = now;
        lastCorrectionEndMs = now;

#if ENABLE_CORRECTION_DEBUG
        Serial.print("CORRECTION_DONE | NewYaw:");
        Serial.print(imu.getYawDeg(), 2);
        Serial.print(" | RemainingDriveMs:");
        if (commandDuration > completedDriveMs) {
          Serial.println(commandDuration - completedDriveMs);
        } else {
          Serial.println(0);
        }
#endif
      }
      break;

    case MOTION_IDLE:
    default:
      stopAllMotors();
      break;
  }
}

void startCorrectionTurn(float headingErrorDeg) {
  stopAllMotors();

  correctionTriggerErrorDeg = headingErrorDeg;
  correctionTurnDurationMs = computeCorrectionTurnDuration(fabs(headingErrorDeg));

  // After YAW_LEFT_POSITIVE_SIGN normalization:
  //   positive error = robot turned left too much  -> turn right
  //   negative error = robot turned right too much -> turn left
  if (headingErrorDeg > 0.0f) {
    correctionTurnDirection = +1; // right
  }
  else {
    correctionTurnDirection = -1; // left
  }

  correctionCount++;
  motionState = MOTION_CORRECTION_STOP_BEFORE;
  stateStartMs = millis();

#if ENABLE_CORRECTION_DEBUG
  Serial.print("CORRECTION_TRIGGER | Error:");
  Serial.print(headingErrorDeg, 2);
  Serial.print(" | Turn:");
  Serial.print(correctionTurnDirection > 0 ? "RIGHT" : "LEFT");
  Serial.print(" | TurnMs:");
  Serial.print(correctionTurnDurationMs);
  Serial.print(" | CompletedDriveMs:");
  Serial.print(completedDriveMs);
  Serial.print(" | StopWaitMs:");
  Serial.println(CORRECTION_STOP_BEFORE_MS);
#endif
}

float getHeadingErrorDeg() {
  float rawError = normalizeAngleDeg(imu.getYawDeg() - targetYawDeg);
  return rawError * (float)YAW_LEFT_POSITIVE_SIGN;
}

float normalizeAngleDeg(float angleDeg) {
  while (angleDeg > 180.0f) {
    angleDeg -= 360.0f;
  }
  while (angleDeg < -180.0f) {
    angleDeg += 360.0f;
  }
  return angleDeg;
}

unsigned long computeCorrectionTurnDuration(float absHeadingErrorDeg) {
  float durationFloat = absHeadingErrorDeg * CORRECTION_TURN_MS_PER_DEG;
  unsigned long durationMs = (unsigned long)durationFloat;
  durationMs = constrain(durationMs, CORRECTION_TURN_MIN_MS, CORRECTION_TURN_MAX_MS);
  return durationMs;
}

// =====================
// Low-Level Motor Control
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
// Individual Motor Functions
// =====================
void setLeftA(int speedValue) {
  setMotor(LEFT_A_PWM, LEFT_A_IN1, LEFT_A_IN2, speedValue * LEFT_A_DIR);
}

void setLeftB(int speedValue) {
  setMotor(LEFT_B_PWM, LEFT_B_IN1, LEFT_B_IN2, speedValue * LEFT_B_DIR);
}

void setRightA(int speedValue) {
  setMotor(RIGHT_A_PWM, RIGHT_A_IN1, RIGHT_A_IN2, speedValue * RIGHT_A_DIR);
}

void setRightB(int speedValue) {
  setMotor(RIGHT_B_PWM, RIGHT_B_IN1, RIGHT_B_IN2, speedValue * RIGHT_B_DIR);
}

// =====================
// Physical Side Control
// =====================
void setPhysicalLeftSide(int speedValue) {
  setLeftA(speedValue);
  setLeftB(speedValue);
}

void setPhysicalRightSide(int speedValue) {
  setRightA(speedValue);
  setRightB(speedValue);
}

// =====================
// Trim Helpers
// =====================
int applyTrim(int speedValue, int trimValue) {
  return constrain(speedValue + trimValue, 0, 255);
}

int applySignedTrim(int speedValue, int trimValue) {
  if (speedValue > 0) {
    return constrain(speedValue + trimValue, 0, 255);
  }
  else if (speedValue < 0) {
    int magnitude = -speedValue;
    magnitude = constrain(magnitude + trimValue, 0, 255);
    return -magnitude;
  }
  else {
    return 0;
  }
}

// =====================
// Movement Functions
// =====================
void moveForwardTrimOnly(int leftSpeed, int rightSpeed) {
  int correctedLeftSpeed = applyTrim(leftSpeed, STRAIGHT_LEFT_TRIM);
  int correctedRightSpeed = applyTrim(rightSpeed, STRAIGHT_RIGHT_TRIM);

  setPhysicalLeftSide(correctedLeftSpeed);
  setPhysicalRightSide(correctedRightSpeed);
}

void moveBackwardTrimOnly(int leftSpeed, int rightSpeed) {
  int correctedLeftSpeed = applyTrim(leftSpeed, STRAIGHT_LEFT_TRIM);
  int correctedRightSpeed = applyTrim(rightSpeed, STRAIGHT_RIGHT_TRIM);

  setPhysicalLeftSide(-correctedLeftSpeed);
  setPhysicalRightSide(-correctedRightSpeed);
}

void turnLeft(int leftSpeed, int rightSpeed) {
  int correctedLeftSpeed = applySignedTrim(-leftSpeed, TURN_LEFT_LEFT_TRIM);
  int correctedRightSpeed = applySignedTrim(rightSpeed, TURN_LEFT_RIGHT_TRIM);

  setPhysicalLeftSide(correctedLeftSpeed);
  setPhysicalRightSide(correctedRightSpeed);
}

void turnRight(int leftSpeed, int rightSpeed) {
  int correctedLeftSpeed = applySignedTrim(leftSpeed, TURN_RIGHT_LEFT_TRIM);
  int correctedRightSpeed = applySignedTrim(-rightSpeed, TURN_RIGHT_RIGHT_TRIM);

  setPhysicalLeftSide(correctedLeftSpeed);
  setPhysicalRightSide(correctedRightSpeed);
}

void stopAllMotors() {
  setPhysicalLeftSide(0);
  setPhysicalRightSide(0);
}