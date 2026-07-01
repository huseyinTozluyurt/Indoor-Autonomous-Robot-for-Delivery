// ======================================================
// DeliveryBot IMU-Free Motor Test Code
// Arduino Uno + 2x L298N + 4 DC Motors
//
// Physical LEFT side motors:
//   LEFT_A  = pins 5, 7, 8
//   LEFT_B  = pins 3, 2, 4
//
// Physical RIGHT side motors:
//   RIGHT_A = pins 6, 9, 10
//   RIGHT_B = pins 11, 12, 13
//
// Straight movement calibration:
//   Forward/backward already moves straight with:
//   left side  = base + 20
//   right side = base - 10
//
// Turning calibration:
//   Right turn was faster than left turn.
//   So right turn is slowed down separately.
//
// Commands:
//   F left_speed right_speed duration_ms
//   B left_speed right_speed duration_ms
//   L left_speed right_speed duration_ms
//   R left_speed right_speed duration_ms
//   S
//
// Example:
//   F 150 150 1000
//   L 150 150 1000
//   R 150 150 1000
// ======================================================


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
// These values make each physical side move correctly.
#define LEFT_A_DIR    1
#define LEFT_B_DIR   -1

#define RIGHT_A_DIR   1
#define RIGHT_B_DIR  -1


// =====================
// Straight Movement Calibration
// =====================
// This is already working well for forward/backward.
// Do not use this trim for turning.
#define STRAIGHT_LEFT_TRIM    20
#define STRAIGHT_RIGHT_TRIM   -10


// =====================
// Turning Calibration
// =====================
// Your observation:
//   Right turn is faster.
//   Left turn is slower.
//
// Strategy:
//   Keep left turn as reference.
//   Slow down right turn.
//
// Tune these values:
//   If right turn is still faster: use -25 or -30.
//   If right turn becomes too slow: use -15 or -10.
#define TURN_LEFT_LEFT_TRIM       0
#define TURN_LEFT_RIGHT_TRIM      0

#define TURN_RIGHT_LEFT_TRIM    -38
#define TURN_RIGHT_RIGHT_TRIM   -38


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

RobotCommand activeCommand = CMD_STOP;

int commandLeftSpeed = 0;
int commandRightSpeed = 0;
unsigned long commandDuration = 0;
unsigned long commandStartTime = 0;


// =====================
// Function Prototypes
// =====================
void handleSerialCommand();
void updateActiveCommand();

void startCommand(
  RobotCommand cmd,
  int leftSpeedValue,
  int rightSpeedValue,
  unsigned long durationValue
);

void setMotor(int pwmPin, int in1Pin, int in2Pin, int speedValue);

void setLeftA(int speedValue);
void setLeftB(int speedValue);
void setRightA(int speedValue);
void setRightB(int speedValue);

void setPhysicalLeftSide(int speedValue);
void setPhysicalRightSide(int speedValue);

int applyTrim(int speedValue, int trimValue);
int applySignedTrim(int speedValue, int trimValue);

void moveForward(int leftSpeed, int rightSpeed);
void moveBackward(int leftSpeed, int rightSpeed);
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

  Serial.println("ARDUINO_MOTOR_TEST_BOOT");
  Serial.println("IMU_DISABLED");
  Serial.println("PHYSICAL_SIDE_MAPPING_ENABLED");
  Serial.println("STRAIGHT_TRIM_ENABLED");
  Serial.println("TURN_TRIM_ENABLED");
  Serial.println("STRAIGHT_LEFT_TRIM_20_RIGHT_TRIM_MINUS_10");
  Serial.println("RIGHT_TURN_SLOWED_BY_20_PWM");
  Serial.println("READY");
  Serial.println("Commands: F/B/L/R left_speed right_speed duration_ms | S");
}


// =====================
// Main Loop
// =====================
void loop() {
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

  int parsed = sscanf(
    line.c_str(),
    "%c %d %d %lu",
    &cmd,
    &leftSpeedValue,
    &rightSpeedValue,
    &durationValue
  );

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
      Serial.println("ACK FORWARD");
      break;

    case 'B':
    case 'b':
      startCommand(CMD_BACKWARD, leftSpeedValue, rightSpeedValue, durationValue);
      Serial.println("ACK BACKWARD");
      break;

    case 'L':
    case 'l':
      startCommand(CMD_LEFT, leftSpeedValue, rightSpeedValue, durationValue);
      Serial.println("ACK LEFT");
      break;

    case 'R':
    case 'r':
      startCommand(CMD_RIGHT, leftSpeedValue, rightSpeedValue, durationValue);
      Serial.println("ACK RIGHT");
      break;

    default:
      Serial.print("ERR UNKNOWN_COMMAND: ");
      Serial.println(line);
      break;
  }
}


void startCommand(
  RobotCommand cmd,
  int leftSpeedValue,
  int rightSpeedValue,
  unsigned long durationValue
) {
  activeCommand = cmd;
  commandLeftSpeed = leftSpeedValue;
  commandRightSpeed = rightSpeedValue;
  commandDuration = durationValue;
  commandStartTime = millis();

  if (cmd == CMD_STOP) {
    stopAllMotors();
  }
}


// =====================
// Active Command Execution
// =====================
void updateActiveCommand() {
  if (activeCommand == CMD_STOP) {
    stopAllMotors();
    return;
  }

  if (commandDuration > 0 && millis() - commandStartTime >= commandDuration) {
    activeCommand = CMD_STOP;
    stopAllMotors();
    Serial.println("DONE");
    return;
  }

  switch (activeCommand) {
    case CMD_FORWARD:
      moveForward(commandLeftSpeed, commandRightSpeed);
      break;

    case CMD_BACKWARD:
      moveBackward(commandLeftSpeed, commandRightSpeed);
      break;

    case CMD_LEFT:
      turnLeft(commandLeftSpeed, commandRightSpeed);
      break;

    case CMD_RIGHT:
      turnRight(commandLeftSpeed, commandRightSpeed);
      break;

    case CMD_STOP:
    default:
      stopAllMotors();
      break;
  }
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
  setMotor(
    LEFT_A_PWM,
    LEFT_A_IN1,
    LEFT_A_IN2,
    speedValue * LEFT_A_DIR
  );
}

void setLeftB(int speedValue) {
  setMotor(
    LEFT_B_PWM,
    LEFT_B_IN1,
    LEFT_B_IN2,
    speedValue * LEFT_B_DIR
  );
}

void setRightA(int speedValue) {
  setMotor(
    RIGHT_A_PWM,
    RIGHT_A_IN1,
    RIGHT_A_IN2,
    speedValue * RIGHT_A_DIR
  );
}

void setRightB(int speedValue) {
  setMotor(
    RIGHT_B_PWM,
    RIGHT_B_IN1,
    RIGHT_B_IN2,
    speedValue * RIGHT_B_DIR
  );
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

// For forward/backward positive speed values.
int applyTrim(int speedValue, int trimValue) {
  return constrain(speedValue + trimValue, 0, 255);
}


// For turning, because turning uses positive and negative speeds.
// Example:
//   +150 with -20 trim becomes +130.
//   -150 with -20 trim becomes -130.
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

void moveForward(int leftSpeed, int rightSpeed) {
  int correctedLeftSpeed = applyTrim(leftSpeed, STRAIGHT_LEFT_TRIM);
  int correctedRightSpeed = applyTrim(rightSpeed, STRAIGHT_RIGHT_TRIM);

  setPhysicalLeftSide(correctedLeftSpeed);
  setPhysicalRightSide(correctedRightSpeed);
}


void moveBackward(int leftSpeed, int rightSpeed) {
  int correctedLeftSpeed = applyTrim(leftSpeed, STRAIGHT_LEFT_TRIM);
  int correctedRightSpeed = applyTrim(rightSpeed, STRAIGHT_RIGHT_TRIM);

  setPhysicalLeftSide(-correctedLeftSpeed);
  setPhysicalRightSide(-correctedRightSpeed);
}


// Classic differential/tank turn with separate turning calibration.
// Left side goes backward, right side goes forward.
void turnLeft(int leftSpeed, int rightSpeed) {
  int correctedLeftSpeed = applySignedTrim(-leftSpeed, TURN_LEFT_LEFT_TRIM);
  int correctedRightSpeed = applySignedTrim(rightSpeed, TURN_LEFT_RIGHT_TRIM);

  setPhysicalLeftSide(correctedLeftSpeed);
  setPhysicalRightSide(correctedRightSpeed);
}


// Classic differential/tank turn with separate turning calibration.
// Left side goes forward, right side goes backward.
//
// Right turn was faster, so this function slows both sides during right turn.
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