#include "SimpleRSLK.h"

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

enum lineStates {
  CENTERED,
  LEFT_OF_LINE,
  RIGHT_OF_LINE,
  CORRECT_LEFT,
  CORRECT_RIGHT,
  TURN_LEFT,
  TURN_RIGHT,
  ALIGN_INTERSECTION,
  AT_INTERSECTION,
  INTERSECTION_DECISION,
  NO_LINE
};

lineStates currLineState = NO_LINE;
lineStates prevLineState;

enum decisionStates {
  STARTUP,
  DETECTING,
  SHOOTING_LEFT,
  SHOOTING_MID,
  SHOOTING_RIGHT
};

decisionStates currDecState = STARTUP;

void keep_driving(){

  getMotorSpeed(normalEncSpeed, normalEncSpeed);

  enableMotor(BOTH_MOTORS);
  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
  setMotorSpeed(RIGHT_MOTOR,rightSpeed);
  setMotorSpeed(LEFT_MOTOR,leftSpeed);
}

void turn_right(){

  getMotorSpeed(fastEncSpeed, normalEncSpeed);
  
  enableMotor(BOTH_MOTORS);
  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
  setMotorSpeed(RIGHT_MOTOR,rightSpeed);
  setMotorSpeed(LEFT_MOTOR,leftSpeed);
}

void turn_left(){

  getMotorSpeed(normalEncSpeed, fastEncSpeed);
  
  enableMotor(BOTH_MOTORS);
  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
  setMotorSpeed(LEFT_MOTOR,leftSpeed);
  setMotorSpeed(RIGHT_MOTOR,rightSpeed);
}

void ninety_right(){// needs to be tuned for a 90 degree turn through either time or encoder
  getMotorSpeed(normalEncSpeed, normalEncSpeed);
  if ((getEncoderLeftCnt() - leftEncTurn) < 180) {
    enableMotor(LEFT_MOTOR);
    setMotorDirection(LEFT_MOTOR,MOTOR_DIR_FORWARD);
    setMotorSpeed(LEFT_MOTOR,leftSpeed);
  }
  else {
    disableMotor(LEFT_MOTOR);
  }
  if ((getEncoderRightCnt() - rightEncTurn) < 180) {
    enableMotor(RIGHT_MOTOR);
    setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_BACKWARD);
    setMotorSpeed(RIGHT_MOTOR,rightSpeed);
  }
  else{
    disableMotor(RIGHT_MOTOR);
  }
  if (((getEncoderRightCnt() - rightEncTurn) > 180) && ((getEncoderLeftCnt() - leftEncTurn) > 133)) {
    turnRight = false;
  }
}

void ninety_left(){// needs to be tuned for a 90 degree turn through either time or encoder
  getMotorSpeed(normalEncSpeed, normalEncSpeed);
  if ((getEncoderLeftCnt() - leftEncTurn) < 180) {
    enableMotor(LEFT_MOTOR);
    setMotorDirection(LEFT_MOTOR,MOTOR_DIR_BACKWARD);
    setMotorSpeed(LEFT_MOTOR,leftSpeed);
  }
  else {
    disableMotor(LEFT_MOTOR);
  }
  if ((getEncoderRightCnt() - rightEncTurn) < 180) {
    enableMotor(RIGHT_MOTOR);
    setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_FORWARD);
    setMotorSpeed(RIGHT_MOTOR,rightSpeed);
  }
  else{
    disableMotor(RIGHT_MOTOR);
  }
  if (((getEncoderRightCnt() - rightEncTurn) > 180) && ((getEncoderLeftCnt() - leftEncTurn) > 180)) {
    turnLeft = false;
  }
  Serial.println(getEncoderLeftCnt()-leftEncTurn);
  Serial.println(getEncoderRightCnt()-rightEncTurn);
}
void stopper(){
  disableMotor(BOTH_MOTORS);
}

void turnTest() {
  int start= millis();
  int curr = millis();
  int rightEncStart = getEncoderRightCnt();
  int leftEncStart = getEncoderLeftCnt();
  while ((curr - start) < 700) {
    getMotorSpeed(normalEncSpeed, normalEncSpeed);
    enableMotor(BOTH_MOTORS);
    setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_BACKWARD);
    setMotorDirection(LEFT_MOTOR,MOTOR_DIR_FORWARD);
    setMotorSpeed(RIGHT_MOTOR,rightSpeed);
    setMotorSpeed(LEFT_MOTOR,leftSpeed);
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
      leftSpeed = leftSpeed+1;
    }
    else if (left_enc_speed > leftEncSpeed) {
      leftSpeed = leftSpeed-1;
    }
    if (right_enc_speed < rightEncSpeed) {
      rightSpeed = rightSpeed+1;
    }
    else if (right_enc_speed > rightEncSpeed) {
      rightSpeed = rightSpeed-1;
    }
  }
}

void followLine() {
  readLineSensor(sensorVal);
  readCalLineSensor(sensorVal,
                    sensorCalVal,
                    sensorMinVal,
                    sensorMaxVal,
                    lineColor);

  uint32_t linePos = getLinePosition(sensorCalVal, lineColor);

  if (AtIntersection()) {
    currLineState = AT_INTERSECTION;
    Serial.println("At Intersection");
  }

  else if (!AtIntersection() && prevLineState == AT_INTERSECTION) {
    currLineState = INTERSECTION_DECISION;
    intersection_counter = intersection_counter + 1;
    Serial.println("Deciding Intersection");
  }
//  
//  else if (CheckIntersection()) {
//    currLineState = ALIGN_INTERSECTION;
//    Serial.println("Aligning with Intersection");
//  }

  else if (linePos >= 0 && linePos < 3000) { // linePos spits out a weighted average of sensorVal where below 3000 is the sensors on the left seeing darker
    if (prevLineState != RIGHT_OF_LINE) {
      startT = millis();
    }
    currLineState = RIGHT_OF_LINE;
    Serial.println("Right of Line");
  } 
  else if (linePos > 3500) {// and the weighted value above 3500 are the sensors on the left seeing darker
    if (prevLineState != LEFT_OF_LINE) {
      startT = millis();
    }
    currLineState = LEFT_OF_LINE;
    Serial.println("Left of Line");
  } 
  else if (linePos >= 3000 && linePos <= 3500) { // between 3000 and 3500 is considered centered
    if (prevLineState == LEFT_OF_LINE) {
      leftCorrect = true;
      correctT = (millis() - startT)*0.5;
      startT = millis();
    }
    else if (prevLineState == RIGHT_OF_LINE) {
      leftCorrect = false;
      correctT = (millis() - startT)*0.5;
      startT = millis();
    }
    if (leftCorrect && (millis()-startT) < correctT) {
      currLineState=CORRECT_LEFT;
      Serial.println("Correct Left");
    }
    else if (!leftCorrect && (millis()-startT) < correctT) {
      currLineState=CORRECT_RIGHT;
      Serial.println("Correct Right");
    }
    else {
      currLineState=CENTERED;
      Serial.println("Centered");
    }
  }
  else {
    currLineState=NO_LINE;
    Serial.println("No Line");
  }
  switch (currLineState) {
    case CENTERED:
      keep_driving();
      break;
    case LEFT_OF_LINE:
      turn_right();
      break;
    case RIGHT_OF_LINE:
      turn_left();
      break;
    case CORRECT_LEFT:
      turn_left();
      break;
    case CORRECT_RIGHT:
      turn_right();
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
      stopper();
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
  waitBtnPressed(LP_LEFT_BTN, btnMsg, RED_LED);
  delay(1000);

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
void AlignAtIntersection(){ // indicates the left edge of robot hit the intersection first
  stopper();
  getMotorSpeed(normalEncSpeed, normalEncSpeed);
  if (sensorVal[0]>2000){
    if (sensorVal[7]>2000){
      stopper();
    }
    else {
      enableMotor(RIGHT_MOTOR); // turns on right motor to bring the right side of the robot even with the left side
      setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_FORWARD);
      setMotorSpeed(RIGHT_MOTOR,rightSpeed); // update speed in future to be non hardcoded
    }
  }
  else if(sensorVal[7]>2000){
    enableMotor(LEFT_MOTOR); // turns on right motor to bring the right side of the robot even with the left side
    setMotorDirection(LEFT_MOTOR,MOTOR_DIR_FORWARD);
    setMotorSpeed(LEFT_MOTOR,leftSpeed); // update speed in future to be non hardcoded
  }
}

void intersectionDecision() {
  if (currDecState == STARTUP) {
    if (intersection_counter == 1) {
      keep_driving();
    }
    else if (intersection_counter == 2) {
      leftEncTurn = getEncoderLeftCnt();
      rightEncTurn = getEncoderRightCnt();
      turnLeft = true;
    }
    else {
      keep_driving();
    }
  }
  Serial.println(intersection_counter);
}

void driving() {
  if (turnLeft) {
    ninety_left();
  }
  else if (turnRight) {
    ninety_right();
  }
  else {
    followLine();
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

  
}

void loop() {
  // put your main code here, to run repeatedly: 
  if (!calibrated) {
    floorCalibration();
    calibrated = true;
  }

  driving();
  
}
