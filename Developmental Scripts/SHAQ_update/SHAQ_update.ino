#include "SimpleRSLK.h"
#include <hcrs04.h>

#define PINTRIG 36
#define PINECHO 37
#define green GREEN_LED
#define red RED_LED

hcrs04 mySensor(PINTRIG, PINECHO);
float KpLine = 0.1;
float KiLine = 0.0;
float KdLine = 0.0;
float LineError = 0.0;
float lastError = 0.0;
float normalSpeed = 25.0;
float fastSpeed = 30.0;
float setPointPID = 3250.0;


// variables that are responsible for High PWM to overcome static friction that is then lowered
bool getMotorFirst = true;
uint16_t FrictionOvercomeSpeed = 30;

// variables for tuning shooting
double shootingEncLeftRight = 4200;
double shootingEncMid = 4200;
double shootingTurnLeft = 93;
double shootingTurnRight = 90;
double shootingReturnRight = 91; // returning to pointing straight from the right basket
double shootingReturnLeft = 99; //returning to pointing straight from the left basket
double PWMShotMid = 100.0;
double PWMShotSide = 150.0; 


//

uint16_t rightSpeed = 5;
uint16_t leftSpeed = 5;
uint16_t slowEncSpeed = 4; // this number controls how fast the motor spins when it rotates
uint16_t normalEncSpeed = 6;
uint16_t fastEncSpeed = 8;
uint32_t left_enc_curr;
uint32_t right_enc_curr;
uint32_t left_enc_prev;
uint32_t right_enc_prev;
uint32_t left_enc_speed;
uint32_t right_enc_speed;
uint32_t leftEncTurn;
uint32_t rightEncTurn;
uint32_t pingEncVal;
int encoder_t = 20;
int prev_t;
bool calibrated = false;
bool leftCorrect = false;
int correctT = 0;
int turningT = 0;
int intT = 0;
int startT = 0;
bool turnLeft = false;
bool turnRight = false;
bool turnCenter = false;
bool right45 = false;
bool left45 = false;
bool correctRight = false;
bool correctLeft = false;
bool shotDone = false;
bool nudge = false;
bool scanning = false;
bool leavingInt = false;
bool reversing = false;
bool startRight;
float distArray[145];
float minDistVal;
int minDistIdx;
int centerIdx;
int basketEncDist = 350;
int shots = 0;

int IRbeaconLeft = 14;
int IRStateLeft;
int IRbeaconMid = 15;
int IRStateMid;
int IRbeaconRight = 17;
int IRStateRight;
int PWM_IN = 18;
int PWM_OUT = 19;
int PWM_SPEED = 38;
const int encoder0PinA = 32;
const int encoder1PinA = 33;
const int encoder0PinB = 34;
const int encoder1PinB = 35;
volatile signed int DCencoderPos = 0;
signed int DCencoderPosLast = 0;

int detectStartTime = 0;
int detectEndTime = 0;
int leftBeaconCounter = 0;
int midBeaconCounter = 0;
int rightBeaconCounter = 0;

uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS];
uint16_t sensorMinVal[LS_NUM_SENSORS];
int intersection_counter = 0;

uint8_t lineColor = DARK_LINE;

enum lineStates {
  SHOT_CENTERED,
  CENTERED,
  LEFT_OF_LINE,
  RIGHT_OF_LINE,
  CORRECT_LEFT,
  CORRECT_RIGHT,
  TURN_LEFT,
  TURN_RIGHT,
  BACKWARDS,
  ALIGN_INTERSECTION,
  AT_INTERSECTION,
  INTERSECTION_DECISION,
  NO_LINE
};

lineStates currLineState;
lineStates prevLineState;

enum decisionStates {
  STARTUP,
  DETECTING,
  SHOOTING_LEFT,
  SHOOTING_MID,
  SHOOTING_RIGHT,
  REALIGN
};

decisionStates currDecState;
decisionStates prevDecState;
decisionStates prevDecState1;

void doEncoderA(){
  if (digitalRead(encoder0PinA) == HIGH) {   // found a low-to-high on channel A
    if (digitalRead(encoder0PinB) == LOW) {  // check channel B to see which way
                                             // encoder is turning
      DCencoderPos = DCencoderPos + 1;         // CCW
    } 
    else {
      DCencoderPos = DCencoderPos - 1;         // CW
}
  }
  else                                        // found a high-to-low on channel A
  { 
    if (digitalRead(encoder0PinB) == LOW) {   // check channel B to see which way
                                              // encoder is turning  
      DCencoderPos = DCencoderPos - 1;          // CW
    } 
    else {
      DCencoderPos = DCencoderPos + 1;          // CCW
    }
  }
}

void doEncoderB(){
  if (digitalRead(encoder0PinB) == HIGH) {   // found a low-to-high on channel A
    if (digitalRead(encoder0PinA) == LOW) {  // check channel B to see which way
                                             // encoder is turning
      DCencoderPos = DCencoderPos - 1;         // CCW
    } 
    else {
      DCencoderPos = DCencoderPos + 1;         // CW
}
  }
  else                                        // found a high-to-low on channel A
  { 
    if (digitalRead(encoder0PinA) == LOW) {   // check channel B to see which way
                                              // encoder is turning  
      DCencoderPos = DCencoderPos + 1;          // CW
    } 
    else {
      DCencoderPos = DCencoderPos - 1;          // CCW
    }
  }
}

void test_driving() {
  if ((((getEncoderLeftCnt() - leftEncTurn) < basketEncDist) && ((getEncoderRightCnt() - rightEncTurn) < basketEncDist))) {
    getMotorSpeed(normalEncSpeed, normalEncSpeed);
    enableMotor(BOTH_MOTORS);
    setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD);
    setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
    setMotorSpeed(RIGHT_MOTOR, rightSpeed);
    setMotorSpeed(LEFT_MOTOR, leftSpeed);
  }
  else {
    disableMotor(BOTH_MOTORS);
  }
}

void keep_driving() {
  enableMotor(BOTH_MOTORS);
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
  getMotorSpeed(normalEncSpeed, normalEncSpeed);
  setMotorSpeed(RIGHT_MOTOR, rightSpeed);
  setMotorSpeed(LEFT_MOTOR, leftSpeed);
}
void keep_driving1() {
  enableMotor(BOTH_MOTORS);
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
  setMotorSpeed(BOTH_MOTORS, normalSpeed);
}

void turn_right() {
  getMotorSpeed(fastEncSpeed, normalEncSpeed);
  enableMotor(BOTH_MOTORS);
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
  setMotorSpeed(RIGHT_MOTOR, rightSpeed);
  setMotorSpeed(LEFT_MOTOR, leftSpeed);
}
void turn_right1() {
  enableMotor(BOTH_MOTORS);
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
  setMotorSpeed(RIGHT_MOTOR, fastSpeed);
  setMotorSpeed(LEFT_MOTOR, normalSpeed);
}

void turn_left() {
  getMotorSpeed(normalEncSpeed, fastEncSpeed);
  enableMotor(BOTH_MOTORS);
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
  setMotorSpeed(LEFT_MOTOR, leftSpeed);
  setMotorSpeed(RIGHT_MOTOR, rightSpeed);
}
void turn_left1() {
  enableMotor(BOTH_MOTORS);
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
  setMotorSpeed(RIGHT_MOTOR, normalSpeed);
  setMotorSpeed(LEFT_MOTOR, fastSpeed);
}

void ninety_right() {  // needs to be tuned for a 90 degree turn through either time or encoder
  getMotorSpeed(slowEncSpeed, slowEncSpeed);
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
    stopper();
  }
}

void ninety_left() {  // needs to be tuned for a 90 degree turn through either time or encoder
  getMotorSpeed(slowEncSpeed, slowEncSpeed);
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
    stopper();
  }
}

void turnLeftBasket(){ // copied the turn ninety left, will most likely turn 45 left but will maybe need to tune
  getMotorSpeed(slowEncSpeed, slowEncSpeed);
  if ((getEncoderLeftCnt() - leftEncTurn) < shootingTurnLeft) {
    enableMotor(LEFT_MOTOR);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorSpeed(LEFT_MOTOR, leftSpeed);
  } else {
    disableMotor(LEFT_MOTOR);
  }
  if ((getEncoderRightCnt() - rightEncTurn) < shootingTurnLeft) {
    enableMotor(RIGHT_MOTOR);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorSpeed(RIGHT_MOTOR, rightSpeed);
  } else {
    disableMotor(RIGHT_MOTOR);
  }
  if (((getEncoderRightCnt() - rightEncTurn) > shootingTurnLeft) && ((getEncoderLeftCnt() - leftEncTurn) > shootingTurnLeft)) {
    left45 = false;
    stopper();
    Serial.println("STOP TURN");
  }
}

void turnRightBasket(){ // copied the turn ninety right, will most likely turn 45 left but will maybe need to tune
  getMotorSpeed(slowEncSpeed, slowEncSpeed);
  if ((getEncoderLeftCnt() - leftEncTurn) < shootingTurnRight) {
    enableMotor(LEFT_MOTOR);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorSpeed(LEFT_MOTOR, leftSpeed);
  } else {
    disableMotor(LEFT_MOTOR);
  }
  if ((getEncoderRightCnt() - rightEncTurn) < shootingTurnRight) {
    enableMotor(RIGHT_MOTOR);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorSpeed(RIGHT_MOTOR, rightSpeed);
  } else {
    disableMotor(RIGHT_MOTOR);
  }
  if (((getEncoderRightCnt() - rightEncTurn) > shootingTurnRight) && ((getEncoderLeftCnt() - leftEncTurn) > shootingTurnRight)) {
    right45 = false;
    stopper();
  }
}

void leaveLeftBasket(){ // copied the turn ninety left, will most likely turn 45 left but will maybe need to tune
  getMotorSpeed(slowEncSpeed, slowEncSpeed);
  if ((getEncoderLeftCnt() - leftEncTurn) < shootingReturnLeft) {
    enableMotor(LEFT_MOTOR);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorSpeed(LEFT_MOTOR, leftSpeed);
  } else {
    disableMotor(LEFT_MOTOR);
  }
  if ((getEncoderRightCnt() - rightEncTurn) < shootingReturnLeft) {
    enableMotor(RIGHT_MOTOR);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorSpeed(RIGHT_MOTOR, rightSpeed);
  } else {
    disableMotor(RIGHT_MOTOR);
  }
  if (((getEncoderRightCnt() - rightEncTurn) > shootingReturnLeft) && ((getEncoderLeftCnt() - leftEncTurn) > shootingReturnLeft)) {
    correctLeft = false;
    stopper();
    Serial.println("STOP TURN");
  }
}

void leaveRightBasket(){ // copied the turn ninety right, will most likely turn 45 left but will maybe need to tune
  getMotorSpeed(slowEncSpeed, slowEncSpeed);
  if ((getEncoderLeftCnt() - leftEncTurn) < shootingReturnRight) {
    enableMotor(LEFT_MOTOR);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorSpeed(LEFT_MOTOR, leftSpeed);
  } else {
    disableMotor(LEFT_MOTOR);
  }
  if ((getEncoderRightCnt() - rightEncTurn) < shootingReturnRight) {
    enableMotor(RIGHT_MOTOR);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorSpeed(RIGHT_MOTOR, rightSpeed);
  } else {
    disableMotor(RIGHT_MOTOR);
  }
  if (((getEncoderRightCnt() - rightEncTurn) > shootingReturnRight) && ((getEncoderLeftCnt() - leftEncTurn) > shootingReturnRight)) {
    correctRight = false;
    stopper();
  }
}

void turn180() {  // needs to be tuned for a 90 degree turn through either time or encoder
  getMotorSpeed(slowEncSpeed, slowEncSpeed);
  if ((getEncoderLeftCnt() - leftEncTurn) < 360) {
    enableMotor(LEFT_MOTOR);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorSpeed(LEFT_MOTOR, leftSpeed);
  } else {
    disableMotor(LEFT_MOTOR);
  }
  if ((getEncoderRightCnt() - rightEncTurn) < 360) {
    enableMotor(RIGHT_MOTOR);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorSpeed(RIGHT_MOTOR, rightSpeed);
  } else {
    disableMotor(RIGHT_MOTOR);
  }
  if (((getEncoderRightCnt() - rightEncTurn) > 360) && ((getEncoderLeftCnt() - leftEncTurn) > 360)) {
    reversing = false;
    stopper();
  }
}

void nudgeForward() {
  getMotorSpeed(slowEncSpeed, slowEncSpeed);
  if ((getEncoderLeftCnt() - leftEncTurn) < 350) {
    enableMotor(LEFT_MOTOR);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorSpeed(LEFT_MOTOR, leftSpeed);
  } else {
    disableMotor(LEFT_MOTOR);
  }
  if ((getEncoderRightCnt() - rightEncTurn) < 350) {
    enableMotor(RIGHT_MOTOR);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorSpeed(RIGHT_MOTOR, rightSpeed);
  } else {
    disableMotor(RIGHT_MOTOR);
  }
  if (((getEncoderRightCnt() - rightEncTurn) > 350) && ((getEncoderLeftCnt() - leftEncTurn) > 350)) {
    nudge = false;
    currDecState = SHOOTING_MID;
    stopper();
  }
}

void stopper() {
  disableMotor(BOTH_MOTORS);
  getMotorFirst = true;
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
}

void getMotorSpeed(uint16_t leftEncSpeed, uint16_t rightEncSpeed) {
  if(getMotorFirst){ // this applies a high PWM value to overcome static friction then allows the lower operationg PWM for the rest of the motion
    leftSpeed = FrictionOvercomeSpeed;
    rightSpeed = FrictionOvercomeSpeed;
    getMotorFirst = false;
  }
  else if ((millis() - prev_t) > encoder_t) {
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
void PIDLineFollow(){
  readLineSensor(sensorVal);
  readCalLineSensor(sensorVal,
                    sensorCalVal,
                    sensorMinVal,
                    sensorMaxVal,
                    lineColor);


  uint32_t linePos = getLinePosition(sensorCalVal, lineColor);
  LineError = linePos - setPointPID; //what is the current position?
  float P = LineError;
  float I = I + LineError;
  float D = LineError - lastError;
  lastError = LineError;
  float motorspeed = P*KpLine + I*KiLine + D*KdLine;
  if (motorspeed>5){


  }
  float motorspeedRight = normalSpeed - motorspeed;
  float motorspeedLeft = normalSpeed + motorspeed;
  if (motorspeedRight > normalSpeed+2){
    motorspeedRight = normalSpeed+2;
  }
  if (motorspeedLeft>normalSpeed+2){
    motorspeedLeft = normalSpeed+2;
  }
  enableMotor(BOTH_MOTORS);
  setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
  setMotorSpeed(RIGHT_MOTOR, motorspeedRight);
  setMotorSpeed(LEFT_MOTOR, motorspeedLeft);
}

void followLine() {
  readLineSensor(sensorVal);
  readCalLineSensor(sensorVal,
                    sensorCalVal,
                    sensorMinVal,
                    sensorMaxVal,
                    lineColor);

  uint32_t linePos = getLinePosition(sensorCalVal, lineColor);

  bool atInt = AtIntersection();
  
  if (atInt) {
    currLineState = AT_INTERSECTION;
    Serial.println("At Intersection");
  }
  else if (!atInt && prevLineState == AT_INTERSECTION && (millis() - intT) > 200) {
    currLineState = INTERSECTION_DECISION;
    intersection_counter = intersection_counter + 1;
    Serial.println("Deciding Intersection");
    Serial.println(intersection_counter);
    intT = millis(); 
  }
  
  else if (CheckIntersection()) {
    currLineState = ALIGN_INTERSECTION;
    Serial.println("Aligning with Intersection");
    
  }
  else if (linePos < 1500) {
    currLineState = NO_LINE;
    Serial.println("No Line");
    
  }
  else if (currDecState == SHOOTING_MID) {
    currLineState = SHOT_CENTERED;
    
  }
  else if (linePos >= 1500 && linePos < setPointPID-500) {  // linePos spits out a weighted average of sensorVal where below 3000 is the sensors on the left seeing darker
    if (prevLineState != RIGHT_OF_LINE) {
      startT = millis();
    }
    currLineState = RIGHT_OF_LINE;
    Serial.println("Right of Line");
    
  } 
  else if (linePos > setPointPID+500) {  // and the weighted value above 3500 are the sensors on the left seeing darker
    if (prevLineState != LEFT_OF_LINE) {
      startT = millis();
    }
    currLineState = LEFT_OF_LINE;
    Serial.println("Left of Line");
  } 
  else if (linePos >= setPointPID-500 && linePos <= setPointPID+500) {  // between 3000 and 3500 is considered centered
    if (prevLineState == LEFT_OF_LINE) {
      leftCorrect = true;
      correctT = (millis() - startT) * 0.5;
      startT = millis();
    } else if (prevLineState == RIGHT_OF_LINE) {
      leftCorrect = false;
      correctT = (millis() - startT) * 0.5;
      startT = millis();
    }
    if (leftCorrect && (millis() - startT) < correctT) {
      currLineState = CORRECT_LEFT;
      Serial.println("Correct Left");
    } else if (!leftCorrect && (millis() - startT) < correctT) {
      currLineState = CORRECT_RIGHT;
      Serial.println("Correct Right");
    } else {
      currLineState = CENTERED;
      Serial.println("Centered");
    }
  }
  switch (currLineState) {
    case SHOT_CENTERED:
      keep_driving();
      break;
    case CENTERED:
      PIDLineFollow();
      break;
    case LEFT_OF_LINE:
      turn_right1();
      break;
    case RIGHT_OF_LINE:
      turn_left1();
      break;
    case CORRECT_LEFT:
      turn_left1();
      break;
    case CORRECT_RIGHT:
      turn_right1();
      break;
    case AT_INTERSECTION:
      keep_driving();
      break;
    case ALIGN_INTERSECTION:
      AlignAtIntersection();
      break;
    case INTERSECTION_DECISION:
      intersectionDecision();
      break;
    case NO_LINE:
      driveOffLine();
      break;
  }

  prevLineState = currLineState;
}

void floorCalibration() {
/* Place Robot On Floor (no line) */
  delay(2000);
  String btnMsg = "Push left button on Launchpad to begin calibration.\n";
  btnMsg += "Make sure the robot is on the floor away from the line.\n";
  /* Wait until button is pressed to start robot */
  waitBtnPressed(LP_LEFT_BTN, btnMsg, RED_LED);

  delay(1000);

  Serial.println("Running calibration on floor");

  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
  /* Enable both motors */
  enableMotor(BOTH_MOTORS);
  /* Set both motors speed 20 */
  setMotorSpeed(BOTH_MOTORS, 20);

  for (int x = 0; x < 100; x++) {
    readLineSensor(sensorVal);
    setSensorMinMax(sensorVal, sensorMinVal, sensorMaxVal);
  }

  /* Disable both motors */
  disableMotor(BOTH_MOTORS);

  Serial.println("Reading floor values complete");

  btnMsg = "Push left button on Launchpad to begin line following.\n";
  btnMsg += "Make sure the robot is on the line.\n";
  /* Wait until button is pressed to start robot */

  // waitBtnPressed(LP_LEFT_BTN, btnMsg, RED_LED); //commented out to allow for only 1 button press
  delay(1000);

  leftEncTurn = getEncoderLeftCnt();
  rightEncTurn = getEncoderRightCnt();
  enableMotor(BOTH_MOTORS);
}

// This works by taking the two outer most line sensors and checks if the value is over 2000 w/ a max of 2500.
// This informs us that even though variable LinePos says that it is centered or to the left, we are able to index the exact sensor value and see that they are all saturated
// It returns a TRUE FALSE for if the robot reads an intersection
bool CheckIntersection() {
  if (sensorVal[0] > 2000 || sensorVal[7] > 2000) {
    return TRUE;
  } else {
    return FALSE;
  }
}

bool AtIntersection() {
  if (sensorVal[0] > 2000 && sensorVal[7] > 2000) {
    return TRUE;
  } else {
    return FALSE;
  }
}

// This function ensures that the robots line sensors are aligned in parrellel with the intersection
void AlignAtIntersection() {  // indicates the left edge of robot hit the intersection first
  stopper();
  getMotorSpeed(slowEncSpeed, slowEncSpeed);
  if (sensorVal[0] > 2000) {
    if (sensorVal[7] > 2000) {
      keep_driving();
    } else {
      enableMotor(RIGHT_MOTOR);  // turns on right motor to bring the right side of the robot even with the left side
      setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
      setMotorSpeed(RIGHT_MOTOR, rightSpeed);  // update speed in future to be non hardcoded
    }
  } else if (sensorVal[7] > 2000) {
    enableMotor(LEFT_MOTOR);  // turns on right motor to bring the right side of the robot even with the left side
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorSpeed(LEFT_MOTOR, leftSpeed);  // update speed in future to be non hardcoded
  }
}

void intersectionDecision() {
  if (currDecState == STARTUP) {
    if (intersection_counter == 1) {
      if (startRight) {
        leftEncTurn = getEncoderLeftCnt();
        rightEncTurn = getEncoderRightCnt();
        turnRight = true;
      }
      else {
        leftEncTurn = getEncoderLeftCnt();
        rightEncTurn = getEncoderRightCnt();
        turnLeft = true;
      }
    }
    else if (intersection_counter == 2) {
      keep_driving();
    }
    else {
      stopper();
      leftEncTurn = getEncoderLeftCnt();
      rightEncTurn = getEncoderRightCnt();
      nudge = true;
      intersection_counter = 0;
    }
  }
  else if (currDecState == REALIGN) {
    if (intersection_counter == 1) {
      keep_driving();
    }
    else if (intersection_counter == 2) {
      leftEncTurn = getEncoderLeftCnt();
      rightEncTurn = getEncoderRightCnt();
      reversing = true;
    }
    else {
      stopper();
      leftEncTurn = getEncoderLeftCnt();
      rightEncTurn = getEncoderRightCnt();
      nudge = true;
      intersection_counter = 0;
    }
  }
}

void driveOffLine() {
  if (currDecState == STARTUP) {
    keep_driving();
  }
}

void driving() {
  if (turnLeft) {
    ninety_left();
  } 
  else if (turnRight) {
    ninety_right();
  }
  else if (right45) {
    turnRightBasket();
  } 
  else if (left45) {
    turnLeftBasket();
  }
  else if (reversing) {
    turn180();
  }
  else if (nudge) {
    nudgeForward();
  }
  else {
    followLine();
  }
}

void shooting() {
  // turn on motor
  if (right45) {
    turnRightBasket();
  }
  else if (left45) {
    turnLeftBasket();
  }
  else if (correctRight) {
    leaveRightBasket();
  }
  else if (correctLeft) {
    leaveLeftBasket();
  }
  else if (shotDone) {
    currDecState = DETECTING;
    shotDone = false;
    shots = shots + 1;
    delay(1000);
  }
  else {
    DCencoderPos = 0;
    stopper();
    digitalWrite(PWM_OUT, LOW);
    digitalWrite(PWM_IN, HIGH);
    Serial.println("SHOOTING");
    
    if (currDecState == SHOOTING_MID){
      analogWrite(PWM_SPEED,PWMShotMid);
      while (DCencoderPos < shootingEncMid ){
      }
    }
    else {
      analogWrite(PWM_SPEED,PWMShotSide);
      while (DCencoderPos < shootingEncLeftRight){
      }
    }
    digitalWrite(PWM_IN, LOW);
    if (currDecState == SHOOTING_LEFT) {
      correctLeft = true;
      Serial.println("turn right");
    }
    if (currDecState == SHOOTING_RIGHT) {
      Serial.print("Turn left");
      correctRight = true;
    }
    leftEncTurn = getEncoderLeftCnt();
    rightEncTurn = getEncoderRightCnt();
    shotDone = true;
  }
}

void startup() {
  if (scanning) {
    initialMap();
  }
  else if (turnCenter) {
    faceCenter();
  }
  else if (distArray[0] == 0) {
    leftEncTurn = getEncoderLeftCnt();
    rightEncTurn = getEncoderRightCnt();
    pingEncVal = 0;
    scanning = true;
  }
  else if (minDistVal == 10000) {
    for (int i = 0; i < 144; i = i+1) {
      if (distArray[i] < minDistVal) {
        minDistVal = distArray[i];
        minDistIdx = i;
      }
    }
    if (intersection_counter == 0) {
      int leftIdx = minDistIdx + 36;
      if (leftIdx > 144) {
        leftIdx = leftIdx - 144;
      }
      int rightIdx = minDistIdx - 36;
      if (rightIdx < 0) {
        rightIdx = rightIdx + 144;
      }
      if (distArray[leftIdx] < distArray[rightIdx]) {
        centerIdx = rightIdx + 10;
        startRight = false;
      }
      else {
        centerIdx = leftIdx + 10;
        startRight = true;
      }
    }
    else {
      centerIdx = minDistIdx;
    }
    turnCenter = true; 
    leftEncTurn = getEncoderLeftCnt();
    rightEncTurn = getEncoderRightCnt();
  }
  else {
    driving();  }
}

void faceCenter() {
  getMotorSpeed(normalEncSpeed, normalEncSpeed);
  uint32_t leftEncVal = getEncoderLeftCnt();
  uint32_t rightEncVal = getEncoderRightCnt();
  if (centerIdx < 82) {
    if ((leftEncVal - leftEncTurn) < (centerIdx*5)) {
      enableMotor(LEFT_MOTOR);
      setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
      setMotorSpeed(LEFT_MOTOR, leftSpeed);
    }
    else {
      disableMotor(LEFT_MOTOR);
    }
    if ((rightEncVal - rightEncTurn) < (centerIdx*5)) {
      enableMotor(RIGHT_MOTOR);
      setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
      setMotorSpeed(RIGHT_MOTOR, rightSpeed);
    }
    else {
      disableMotor(RIGHT_MOTOR);
    }
    if (((rightEncVal - rightEncTurn) > (centerIdx*5)) && ((leftEncVal - leftEncTurn) > (centerIdx*5))) {
      turnCenter = false;
    }
  }
  else {
    if ((leftEncVal - leftEncTurn) < (770-(centerIdx*5))) {
      enableMotor(LEFT_MOTOR);
      setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
      setMotorSpeed(LEFT_MOTOR, leftSpeed);
    } 
    else {
      disableMotor(LEFT_MOTOR);
    }
    if ((rightEncVal - rightEncTurn) < (770-centerIdx*5)) {
      enableMotor(RIGHT_MOTOR);
      setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
      setMotorSpeed(RIGHT_MOTOR, rightSpeed);
    } 
    else {
      disableMotor(RIGHT_MOTOR);
    }
    if (((rightEncVal - rightEncTurn) > (770-centerIdx*5)) && ((leftEncVal - leftEncTurn) > (770-centerIdx*5))) {
      turnCenter = false;
    }
  }
}

void initialMap() {
  getMotorSpeed(slowEncSpeed, slowEncSpeed);
  uint32_t leftEncVal = getEncoderLeftCnt();
  uint32_t rightEncVal = getEncoderRightCnt();
  if ((leftEncVal - leftEncTurn) < 720) {
    enableMotor(LEFT_MOTOR);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorSpeed(LEFT_MOTOR, leftSpeed);
    if ((leftEncVal - leftEncTurn) > pingEncVal*5) {
      distArray[pingEncVal] = mySensor.read();
      pingEncVal = pingEncVal + 1;
    }
  } else {
    disableMotor(LEFT_MOTOR);
  }
  if ((rightEncVal - rightEncTurn) < 720) {
    enableMotor(RIGHT_MOTOR);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorSpeed(RIGHT_MOTOR, rightSpeed);
  } else {
    disableMotor(RIGHT_MOTOR);
  }
  if (((rightEncVal - rightEncTurn) > 720) && ((leftEncVal - leftEncTurn) > 720)) {
    scanning = false;
  }
}

void detecting() {
  if (prevDecState != currDecState) {
    Serial.println("START DETECTING");
    detectEndTime = millis() + 1000;
    leftBeaconCounter = 0;
    midBeaconCounter = 0;
    rightBeaconCounter = 0;
  }
  if (millis() < detectEndTime) {
    IRStateLeft = digitalRead(IRbeaconLeft);
    IRStateMid = digitalRead(IRbeaconMid);
    IRStateRight = digitalRead(IRbeaconRight);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(GREEN_LED, LOW);

    if (IRStateLeft == 0) {
      leftBeaconCounter++;
      digitalWrite(GREEN_LED, HIGH);
    }
    if (IRStateMid == 0) {
      midBeaconCounter++;
    }
    if (IRStateRight == 0) {
      rightBeaconCounter++;
      digitalWrite(GREEN_LED, HIGH);
    }
  } 
  
  else {
    //set new currDecState Variable here
    if (leftBeaconCounter > midBeaconCounter && leftBeaconCounter > rightBeaconCounter) {
      currDecState = SHOOTING_LEFT;
      leftEncTurn = getEncoderLeftCnt();
      rightEncTurn = getEncoderRightCnt();
      left45 = true;
    } 
    else if (midBeaconCounter > leftBeaconCounter && midBeaconCounter > rightBeaconCounter) {
      currDecState = SHOOTING_MID;
    } 
    else if (rightBeaconCounter > leftBeaconCounter && rightBeaconCounter > midBeaconCounter) {
      currDecState = SHOOTING_RIGHT;
      leftEncTurn = getEncoderLeftCnt();
      rightEncTurn = getEncoderRightCnt();
      right45 = true;
    }
    else {
      detectEndTime = millis() + 1000;
      leftBeaconCounter = 0;
      midBeaconCounter = 0;
      rightBeaconCounter = 0;
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  setupRSLK();
  setupEncoder(72, 12, 56, 13);
  prev_t = millis();
  currDecState = STARTUP;
  prevDecState1 = STARTUP;
  

  //currDecState = SHOOTING_RIGHT;
  pinMode(IRbeaconLeft, INPUT_PULLUP);  // NOTE: because this is a pullup, a 1 indicates no beacon detected, 0 is yes beacon detected
  pinMode(IRbeaconMid, INPUT_PULLUP);
  pinMode(IRbeaconRight, INPUT_PULLUP);

  pinMode(PWM_IN, OUTPUT);
  pinMode(PWM_OUT, OUTPUT);
  pinMode(PWM_SPEED, OUTPUT);

 pinMode(encoder0PinA, INPUT_PULLUP);
 pinMode(encoder0PinB, INPUT_PULLUP);
 pinMode(encoder1PinA, INPUT_PULLUP);
 pinMode(encoder1PinB, INPUT_PULLUP);
 attachInterrupt(encoder0PinA, doEncoderA, RISING); // Interrupt is fired whenever button is pressed
 attachInterrupt(encoder1PinA, doEncoderA, FALLING);
 attachInterrupt(encoder0PinB, doEncoderB, RISING);
 attachInterrupt(encoder1PinB, doEncoderB, FALLING);


  setupWaitBtn(LP_LEFT_BTN);
  /* Red led in rgb led */
  setupLed(RED_LED);
  setupLed(GREEN_LED);
  clearMinMax(sensorMinVal, sensorMaxVal);

  mySensor.begin(); /* Initialize the sensor */
  distArray[0] = 0;
  minDistVal = 10000;
}

void loop() {
  // put your main code here, to run repeatedly:
  if (!calibrated) {
    floorCalibration();
    calibrated = true;
  }

  switch (currDecState) {
    case STARTUP:
      startup();
      break;
    case DETECTING:
      detecting();
      break;
    case REALIGN:
      driving();
      break;
    case SHOOTING_RIGHT:
      shooting();
      break;
    case SHOOTING_LEFT:
      shooting();
      break;
    case SHOOTING_MID:
      shooting();
      break;
  }

//  if (shots > 2) {
//    currDecState = REALIGN;
//    shots = 0;
//    reversing = true;
//  }

  prevDecState = prevDecState1;
  prevDecState1 = currDecState;
}
