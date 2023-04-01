#include "SimpleRSLK.h"

// IR Beacon Initialization Values
#define green GREEN_LED
#define red RED_LED
int IRbeaconLeft = 2;
int IRbeaconMid = 3;
int IRbeaconRight = 4;
int prev1Left = 1;
int prev2Left = 1;
int prev3Left = 1;
int prev4Left = 1;

int prev1Mid = 1;
int prev2Mid = 1;
int prev3Mid = 1;
int prev4Mid = 1;

int prev1Right = 1;
int prev2Right = 1;
int prev3Right = 1;
int prev4Right = 1;


uint16_t rightSpeed = 5;
uint16_t leftSpeed = 5;
uint16_t normalEncSpeed = 4;
uint16_t fastEncSpeed = 6;
uint32_t left_enc_curr;
uint32_t right_enc_curr;
uint32_t left_enc_prev;
uint32_t right_enc_prev;
uint32_t left_enc_speed;
uint32_t right_enc_speed;
uint32_t leftEncTurn;
uint32_t rightEncTurn;
int encoder_t = 15;
int prev_t;
bool calibrated = false;
bool leftCorrect = false;
int correctT = 0;
int turningT = 0;
int startT = 0;
bool turnLeft = false;
bool turnRight = false;

uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS];
uint16_t sensorMinVal[LS_NUM_SENSORS];
int intersection_counter = 0;

uint8_t lineColor = DARK_LINE;

enum decisionStates {
  STARTUP,
  DETECTING,
  SHOOTING_LEFT,
  SHOOTING_MID,
  SHOOTING_RIGHT
};

decisionStates currDecState = DETECTING;

void ninety_right() {  // needs to be tuned for a 90 degree turn through either time or encoder
  getMotorSpeed(normalEncSpeed, normalEncSpeed);
  if ((getEncoderLeftCnt() - leftEncTurn) < 180) {
    enableMotor(LEFT_MOTOR);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorSpeed(LEFT_MOTOR, leftSpeed);
  } else {
    disableMotor(LEFT_MOTOR);
  }
  if ((getEncoderRightCnt() - rightEncTurn) < 180) {
    enableMotor(RIGHT_MOTOR);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorSpeed(RIGHT_MOTOR, rightSpeed);
  } else {
    disableMotor(RIGHT_MOTOR);
  }
  if (((getEncoderRightCnt() - rightEncTurn) > 180) && ((getEncoderLeftCnt() - leftEncTurn) > 180)) {
    turnRight = false;
  }
}

void ninety_left() {  // needs to be tuned for a 90 degree turn through either time or encoder
  getMotorSpeed(normalEncSpeed, normalEncSpeed);
  if ((getEncoderLeftCnt() - leftEncTurn) < 180) {
    enableMotor(LEFT_MOTOR);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorSpeed(LEFT_MOTOR, leftSpeed);
  } else {
    disableMotor(LEFT_MOTOR);
  }
  if ((getEncoderRightCnt() - rightEncTurn) < 180) {
    enableMotor(RIGHT_MOTOR);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorSpeed(RIGHT_MOTOR, rightSpeed);
  } else {
    disableMotor(RIGHT_MOTOR);
  }
  if (((getEncoderRightCnt() - rightEncTurn) > 180) && ((getEncoderLeftCnt() - leftEncTurn) > 180)) {
    turnLeft = false;
  }
  Serial.println(getEncoderLeftCnt() - leftEncTurn);
  Serial.println(getEncoderRightCnt() - rightEncTurn);
}

void face_left_basket() {
  getMotorSpeed(normalEncSpeed, normalEncSpeed);
  if ((getEncoderLeftCnt() - leftEncTurn) < 90) {
    enableMotor(LEFT_MOTOR);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorSpeed(LEFT_MOTOR, leftSpeed);
  } else {
    disableMotor(LEFT_MOTOR);
  }
  if ((getEncoderRightCnt() - rightEncTurn) < 90) {
    enableMotor(RIGHT_MOTOR);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorSpeed(RIGHT_MOTOR, rightSpeed);
  } else {
    disableMotor(RIGHT_MOTOR);
  }
  if (((getEncoderRightCnt() - rightEncTurn) > 180) && ((getEncoderLeftCnt() - leftEncTurn) > 180)) {
    turnLeft = false;
  }
  Serial.println(getEncoderLeftCnt() - leftEncTurn);
  Serial.println(getEncoderRightCnt() - rightEncTurn);
}

void face_right_basket() {
  getMotorSpeed(normalEncSpeed, normalEncSpeed);
  if ((getEncoderLeftCnt() - leftEncTurn) < 90) {
    enableMotor(LEFT_MOTOR);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorSpeed(LEFT_MOTOR, leftSpeed);
  } else {
    disableMotor(LEFT_MOTOR);
  }

  if ((getEncoderRightCnt() - rightEncTurn) < 90) {
    enableMotor(RIGHT_MOTOR);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorSpeed(RIGHT_MOTOR, rightSpeed);
  } else {
    disableMotor(RIGHT_MOTOR);
  }
  if (((getEncoderRightCnt() - rightEncTurn) > 90) && ((getEncoderLeftCnt() - leftEncTurn) > 133)) {
    turnRight = false;
  }
}

void stopper() {
  disableMotor(BOTH_MOTORS);
}

void turnTest() {
  int start = millis();
  int curr = millis();
  int rightEncStart = getEncoderRightCnt();
  int leftEncStart = getEncoderLeftCnt();
  while ((curr - start) < 700) {
    getMotorSpeed(normalEncSpeed, normalEncSpeed);
    enableMotor(BOTH_MOTORS);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorSpeed(RIGHT_MOTOR, rightSpeed);
    setMotorSpeed(LEFT_MOTOR, leftSpeed);
    curr = millis();
  }
  Serial.println(getEncoderLeftCnt() - leftEncStart);
  Serial.println(getEncoderRightCnt() - rightEncStart);
}

void getMotorSpeed(uint16_t leftEncSpeed, uint16_t rightEncSpeed) {
  if ((millis() - prev_t) > encoder_t) {
    left_enc_curr = getEncoderLeftCnt();
    right_enc_curr = getEncoderRightCnt();
    left_enc_speed = left_enc_curr - left_enc_prev;
    right_enc_speed = right_enc_curr - right_enc_prev;

    prev_t = millis();
    left_enc_prev = left_enc_curr;
    right_enc_prev = right_enc_curr;

    if (left_enc_speed < leftEncSpeed) {
      leftSpeed = leftSpeed + 1;
    } else if (left_enc_speed > leftEncSpeed) {
      leftSpeed = leftSpeed - 1;
    }
    if (right_enc_speed < rightEncSpeed) {
      rightSpeed = rightSpeed + 1;
    } else if (right_enc_speed > rightEncSpeed) {
      rightSpeed = rightSpeed - 1;
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  setupRSLK();
  setupEncoder(72, 12, 56, 13);
  prev_t = millis();

  setupWaitBtn(LP_LEFT_BTN);
  /* Red led in rgb led */
  setupLed(RED_LED);
  clearMinMax(sensorMinVal, sensorMaxVal);
  pinMode(IRbeacon1, INPUT_PULLUP);  // NOTE: because this is a pullup, a 1 indicates no beacon detected, 0 is yes beacon detected
}

int counter = 0;

void loop() {
 
   
}
