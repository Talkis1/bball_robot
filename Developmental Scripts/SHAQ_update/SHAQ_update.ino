#include "SimpleRSLK.h"
#include <hcrs04.h>

#define PINTRIG 36
#define PINECHO 37
#define green GREEN_LED
#define red RED_LED

hcrs04 mySensor(PINTRIG, PINECHO);
// variables that are responsible for High PWM to overcome static friction that is then lowered
bool getMotorFirst = true;
uint16_t FrictionOvercomeSpeed = 20;

uint16_t rightSpeed = 5;
uint16_t leftSpeed = 5;
uint16_t slowEncSpeed = 3; // this number controls how fast the motor spins when it rotates
uint16_t normalEncSpeed = 5;
uint16_t fastEncSpeed = 7;
uint32_t left_enc_curr;
uint32_t right_enc_curr;
uint32_t left_enc_prev;
uint32_t right_enc_prev;
uint32_t left_enc_speed;
uint32_t right_enc_speed;
uint32_t leftEncTurn;
uint32_t rightEncTurn;
uint32_t pingEncVal;
int encoder_t = 15;
int prev_t;
bool calibrated = false;
bool leftCorrect = false;
int correctT = 0;
int turningT = 0;
int startT = 0;
bool turnLeft = false;
bool turnRight = false;
bool turnCenter = false;
bool right45 = false;
bool left45 = false;
bool shotDone = false;
bool scanning = false;
bool offLine = false;
bool leavingInt = false;
bool backwards = false;
bool startRight;
float distArray[153];
float minDistVal;
int minDistIdx;
int centerIdx;
int basketEncDist = 350;

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
  SHOOTING_RIGHT
};

decisionStates currDecState;
decisionStates prevDecState;
decisionStates prevDecState1 ;

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
  if ((((getEncoderLeftCnt() - leftEncTurn) < basketEncDist) && ((getEncoderRightCnt() - rightEncTurn) < basketEncDist)) || backwards) {
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
  if ((currDecState == SHOOTING_MID) || ((currDecState == SHOOTING_LEFT || currDecState == SHOOTING_RIGHT) && intersection_counter == 1)) {
    leftEncTurn = getEncoderLeftCnt();
    rightEncTurn = getEncoderRightCnt();
    detectEndTime = millis() + 500;
    midBeaconCounter = 0;
    offLine = true;
  }
  else {
    enableMotor(BOTH_MOTORS);
    if (backwards) {
      setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD);
      getMotorSpeed(slowEncSpeed, slowEncSpeed);
    }
    else {
      setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
      getMotorSpeed(normalEncSpeed, normalEncSpeed);
    }
    setMotorSpeed(RIGHT_MOTOR, rightSpeed);
    setMotorSpeed(LEFT_MOTOR, leftSpeed);
  }
}

void turn_right() {

  getMotorSpeed(fastEncSpeed, normalEncSpeed);

  enableMotor(BOTH_MOTORS);
  if (backwards) {
    setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD);
  }
  else {
    setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
  }
  setMotorSpeed(RIGHT_MOTOR, rightSpeed);
  setMotorSpeed(LEFT_MOTOR, leftSpeed);
}

void turn_left() {

  getMotorSpeed(normalEncSpeed, fastEncSpeed);

  enableMotor(BOTH_MOTORS);
  if (backwards) {
    setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD);
  }
  else {
    setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
  }
  setMotorSpeed(LEFT_MOTOR, leftSpeed);
  setMotorSpeed(RIGHT_MOTOR, rightSpeed);
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
  if ((getEncoderLeftCnt() - leftEncTurn) < 190) {
    enableMotor(LEFT_MOTOR);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorSpeed(LEFT_MOTOR, leftSpeed);
  } else {
    disableMotor(LEFT_MOTOR);
  }
  if ((getEncoderRightCnt() - rightEncTurn) < 190) {
    enableMotor(RIGHT_MOTOR);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorSpeed(RIGHT_MOTOR, rightSpeed);
  } else {
    disableMotor(RIGHT_MOTOR);
  }
  if (((getEncoderRightCnt() - rightEncTurn) > 190) && ((getEncoderLeftCnt() - leftEncTurn) > 190)) {
    turnLeft = false;
    stopper();
  }
}

void turnLeftBasket(){ // copied the turn ninety left, will most likely turn 45 left but will maybe need to tune
  getMotorSpeed(slowEncSpeed, slowEncSpeed);
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
  if (((getEncoderRightCnt() - rightEncTurn) > 90) && ((getEncoderLeftCnt() - leftEncTurn) > 90)) {
    left45 = false;
    stopper();
    Serial.println("STOP TURN");
  }
}

void turnRightBasket(){ // copied the turn ninety right, will most likely turn 45 left but will maybe need to tune
  getMotorSpeed(slowEncSpeed, slowEncSpeed);
  if ((getEncoderLeftCnt() - leftEncTurn) < 80) {
    enableMotor(LEFT_MOTOR);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorSpeed(LEFT_MOTOR, leftSpeed);
  } else {
    disableMotor(LEFT_MOTOR);
  }
  if ((getEncoderRightCnt() - rightEncTurn) < 80) {
    enableMotor(RIGHT_MOTOR);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorSpeed(RIGHT_MOTOR, rightSpeed);
  } else {
    disableMotor(RIGHT_MOTOR);
  }
  if (((getEncoderRightCnt() - rightEncTurn) > 80) && ((getEncoderLeftCnt() - leftEncTurn) > 80)) {
    right45 = false;
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

void followLine() {
  readLineSensor(sensorVal);
  readCalLineSensor(sensorVal,
                    sensorCalVal,
                    sensorMinVal,
                    sensorMaxVal,
                    lineColor);

  uint32_t linePos = getLinePosition(sensorCalVal, lineColor);

  if (AtIntersection()) {
    if (backwards) {
      currLineState = INTERSECTION_DECISION;
      intersection_counter = intersection_counter + 1;
      Serial.println("Deciding Intersection");
      backwards = false;
    }
    else {
      if (!leavingInt) {
        startT = millis();
        leavingInt = true;
      }
      currLineState = AT_INTERSECTION;
      Serial.println("At Intersection");
    }
  }
  else if (!AtIntersection() && prevLineState == AT_INTERSECTION && (millis()-startT) > 50s0) {
    currLineState = INTERSECTION_DECISION;
    intersection_counter = intersection_counter + 1;
    Serial.println("Deciding Intersection");
    Serial.println(intersection_counter);
    leavingInt = false;
  }
  
  else if (CheckIntersection() && !leavingInt) {
    currLineState = ALIGN_INTERSECTION;
    Serial.println("Aligning with Intersection");
  }

  else if (offLine || linePos < 1500) {
    currLineState = NO_LINE;
    Serial.println("No Line");
  }

  else if (currDecState == SHOOTING_MID) {
    currLineState = CENTERED;
  }
  
  else if (linePos >= 1500 && linePos < 3000) {  // linePos spits out a weighted average of sensorVal where below 3000 is the sensors on the left seeing darker
    if (prevLineState != RIGHT_OF_LINE) {
      startT = millis();
    }
    currLineState = RIGHT_OF_LINE;
    Serial.println("Right of Line");
  } 
  else if (linePos > 3500) {  // and the weighted value above 3500 are the sensors on the left seeing darker
    if (prevLineState != LEFT_OF_LINE) {
      startT = millis();
    }
    currLineState = LEFT_OF_LINE;
    Serial.println("Left of Line");
  } 
  else if (linePos >= 3000 && linePos <= 3500) {  // between 3000 and 3500 is considered centered
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
  waitBtnPressed(LP_LEFT_BTN, btnMsg, RED_LED);
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
  getMotorSpeed(normalEncSpeed, normalEncSpeed);
  if (sensorVal[0] > 2000) {
    if (sensorVal[7] > 2000) {
      stopper();
    } else {
      enableMotor(RIGHT_MOTOR);  // turns on right motor to bring the right side of the robot even with the left side
      if (backwards) {
        setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
      }
      else {
        setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
      }
      setMotorSpeed(RIGHT_MOTOR, rightSpeed);  // update speed in future to be non hardcoded
    }
  } else if (sensorVal[7] > 2000) {
    enableMotor(LEFT_MOTOR);  // turns on right motor to bring the right side of the robot even with the left side
    if (backwards) {
      setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
    }
    else {
      setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    }
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
      offLine = false;
    }
    else if (intersection_counter == 2) {
      keep_driving();
    }
    else {
      stopper();
      currDecState = DETECTING;
      ;
    }
  }
  else if (currDecState == SHOOTING_LEFT) {
    if (intersection_counter == 1) {
      // turns right at the first intersection after starting the path to the left hoop
      // this turns the robot to face the basket
      leftEncTurn = getEncoderLeftCnt();
      rightEncTurn = getEncoderRightCnt();
      turnRight = true;
    } 
    else if (intersection_counter == 2) {
      // turns left to face the detection location after returning 
      //from the basket and detecting the line perpendicular to path
      leftEncTurn = getEncoderLeftCnt();
      rightEncTurn = getEncoderRightCnt();
      turnRight = true;
      offLine = false;
    }
    else if (intersection_counter == 3) {
      leftEncTurn = getEncoderLeftCnt();
      rightEncTurn = getEncoderRightCnt();
      turnLeft = true;
    }
    else {
      stopper();
    }
  }
  
  else if (currDecState == SHOOTING_MID) {
    currDecState = DETECTING;
    stopper();
    offLine = false;
  }
  
  else if (currDecState == SHOOTING_RIGHT) {
    if (intersection_counter == 1) {
      // turns left at the first intersection after starting the path to the right hoop
      // this turns the robot to face the basket
      leftEncTurn = getEncoderLeftCnt();
      rightEncTurn = getEncoderRightCnt();
      turnLeft = true;
    } 
    else if (intersection_counter == 2) {
      // turns right to face the detection location after returning 
      //from the basket and detecting the line perpendicular to path
      leftEncTurn = getEncoderLeftCnt();
      rightEncTurn = getEncoderRightCnt();
      turnLeft = true;
      offLine = false;
    }
    else if (intersection_counter == 3) {
      leftEncTurn = getEncoderLeftCnt();
      rightEncTurn = getEncoderRightCnt();
      turnRight = true;
    }
    else {
      stopper();
    }
  }
}

void driveOffLine() {
  if (currDecState == STARTUP) {
    keep_driving();
  }
  else {
    if ((((getEncoderLeftCnt() - leftEncTurn) < basketEncDist) && ((getEncoderRightCnt() - rightEncTurn) < basketEncDist)) || backwards) {
      enableMotor(BOTH_MOTORS);
      if (backwards) {
        setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD);
        getMotorSpeed(slowEncSpeed, slowEncSpeed);
      }
      else {
        setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
        getMotorSpeed(normalEncSpeed, normalEncSpeed);getMotorSpeed(normalEncSpeed, normalEncSpeed);
      }
      setMotorSpeed(RIGHT_MOTOR, rightSpeed);
      setMotorSpeed(LEFT_MOTOR, leftSpeed);
//      if (millis() < detectEndTime) {
//        IRStateMid = digitalRead(IRbeaconMid);
//        if (IRStateMid == 0) {
//          midBeaconCounter++;
//        }
//      }
//      else {
//        if (midBeaconCounter == 0) {
//          backwards = true;
//        }
//        else {
//          detectEndTime = millis() + 500;
//        }
//      }
    }
    else {
      shooting();
      Serial.println("Shooting");
    }
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
  else if (shotDone) {
    currDecState = DETECTING;
    shotDone = false;
  }
  else {
    DCencoderPos = 0;
    stopper();
    digitalWrite(PWM_OUT, LOW);
    digitalWrite(PWM_IN, HIGH);
    Serial.println("SHOOTING");
    
    if (currDecState == SHOOTING_MID){
      analogWrite(PWM_SPEED,abs(135));
      while (DCencoderPos < 4380 ){
      }
    }
    else {
      analogWrite(PWM_SPEED,abs(175));
      while (DCencoderPos < 4380){
      }
    }
    digitalWrite(PWM_IN, LOW);
    if (currDecState == SHOOTING_LEFT) {
      right45 = true;
      Serial.println("turn right");
    }
    if (currDecState == SHOOTING_RIGHT) {
      Serial.print("Turn left");
      left45 = true;
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
    for (int i = 0; i < 152; i = i+1) {
      if (distArray[i] < minDistVal) {
        minDistVal = distArray[i];
        minDistIdx = i;
      }
    }
    if (intersection_counter == 0) {
      int leftIdx = minDistIdx + 38;
      if (leftIdx > 152) {
        leftIdx = leftIdx - 144;
      }
      int rightIdx = minDistIdx - 38;
      if (rightIdx < 0) {
        rightIdx = rightIdx + 152;
      }
      if (distArray[leftIdx] < distArray[rightIdx]) {
        centerIdx = rightIdx;
        startRight = true;
      }
      else {
        centerIdx = leftIdx;
        startRight = false;
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
  if (centerIdx < 76) {
    if ((leftEncVal - leftEncTurn) < (centerIdx*5)) {
      enableMotor(LEFT_MOTOR);
      setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
      setMotorSpeed(LEFT_MOTOR, leftSpeed);
    }
    else {
      disableMotor(LEFT_MOTOR);
    }
    if ((rightEncVal - rightEncTurn) < (centerIdx*5)) {
      enableMotor(RIGHT_MOTOR);
      setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
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
    if ((leftEncVal - leftEncTurn) < (760-(centerIdx*5))) {
      Serial.println("Turning");
      enableMotor(LEFT_MOTOR);
      setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
      setMotorSpeed(LEFT_MOTOR, leftSpeed);
    } 
    else {
      disableMotor(LEFT_MOTOR);
    }
    if ((rightEncVal - rightEncTurn) < (760-centerIdx*5)) {
      enableMotor(RIGHT_MOTOR);
      setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
      setMotorSpeed(RIGHT_MOTOR, rightSpeed);
    } 
    else {
      disableMotor(RIGHT_MOTOR);
    }
    if (((rightEncVal - rightEncTurn) > (760-centerIdx*5)) && ((leftEncVal - leftEncTurn) > (760-centerIdx*5))) {
      turnCenter = false;
//      if (intersection_counter == 3) {
//        currDecState = DETECTING;
//      }
    }
  }
}

void initialMap() {
  getMotorSpeed(slowEncSpeed, slowEncSpeed);
  uint32_t leftEncVal = getEncoderLeftCnt();
  uint32_t rightEncVal = getEncoderRightCnt();
  if ((leftEncVal - leftEncTurn) < 760) {
    enableMotor(LEFT_MOTOR);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorSpeed(LEFT_MOTOR, leftSpeed);
    if ((leftEncVal - leftEncTurn) > pingEncVal*5) {
      distArray[pingEncVal] = mySensor.read();
      pingEncVal = pingEncVal + 1;
    }
  } else {
    disableMotor(LEFT_MOTOR);
  }
  if ((rightEncVal - rightEncTurn) < 760) {
    enableMotor(RIGHT_MOTOR);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorSpeed(RIGHT_MOTOR, rightSpeed);
  } else {
    disableMotor(RIGHT_MOTOR);
  }
  if (((rightEncVal - rightEncTurn) > 760) && ((leftEncVal - leftEncTurn) > 760)) {
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
    prevDecState = prevDecState1;
    prevDecState1 = currDecState;
}
