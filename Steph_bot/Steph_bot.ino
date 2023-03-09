#include "SimpleRSLK.h"
#define green GREEN_LED
#define red RED_LED

// Defines variables associated with the IR sensor
// int IRbeacon1 = 10; will need to make sure we pick a pin that isn't used by the platform
int prev1 = 1;
int prev2 = 1;
int prev3 = 1;
int prev4 = 1;

uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS];
uint16_t sensorMinVal[LS_NUM_SENSORS];
int intersection_counter = 0;

enum states {
  CENTERED,
  LEFT_OF_LINE,
  RIGHT_OF_LINE,
  AT_INTERSECTION,
  NO_LINE
};

states curr_state = NO_LINE;
states prev_state;

uint16_t normalSpeed = 20;
uint16_t fastSpeed = 25;

// Below are the state functions for all possible translation and rotation motions
void keep_driving(){
  enableMotor(BOTH_MOTORS);
  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
  setMotorSpeed(BOTH_MOTORS,normalSpeed);
}

void turn_right(){
  enableMotor(BOTH_MOTORS);
  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
  setMotorSpeed(RIGHT_MOTOR,fastSpeed);
  setMotorSpeed(LEFT_MOTOR,normalSpeed);
}

void turn_left(){
  enableMotor(BOTH_MOTORS);
  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
  setMotorSpeed(LEFT_MOTOR,fastSpeed);
  setMotorSpeed(RIGHT_MOTOR,normalSpeed);
}

void ninety_right(){// needs to be tuned for a 90 degree turn through either time or encoder
  int start= millis();
  int curr = millis();
  while ((curr - start) < 700) {
    enableMotor(BOTH_MOTORS);
    setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_BACKWARD);
    setMotorDirection(LEFT_MOTOR,MOTOR_DIR_FORWARD);
    setMotorSpeed(RIGHT_MOTOR,normalSpeed);
    setMotorSpeed(LEFT_MOTOR,normalSpeed);
    curr = millis();
  }
}

void ninety_left(){// needs to be tuned for a 90 degree turn through either time or encoder
  int start= millis();
  int curr = millis();
  while ((curr - start) < 700) {
    enableMotor(BOTH_MOTORS);
    setMotorDirection(LEFT_MOTOR,MOTOR_DIR_BACKWARD);
    setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_FORWARD);
    setMotorSpeed(LEFT_MOTOR,normalSpeed);
    setMotorSpeed(RIGHT_MOTOR,normalSpeed);
    curr = millis();
  }
}
void stopper(){
  disableMotor(BOTH_MOTORS);
}

// This works by taking the two outer most line sensors and checks if the value is over 2000 w/ a max of 2500. 
// This informs us that even though variable LinePos says that it is centered or to the left, we are able to index the exact sensor value and see that they are all saturated
// It returns a TRUE FALSE for if the robot reads an intersection
bool CheckIntersection() { 
  if (sensorVal[0] > 2000 && sensorVal[7] > 2000) {
    return TRUE;
  } else {
    return FALSE;
  }
}

// These functions handle the detection of an IR beacon
bool CheckIR() {
  int IRState = digitalRead(IRbeacon1);
  int IRComp = (IRState + prev1 + prev2 + prev3 + prev4)/5;
  if (IRComp < 1) {
    digitalWrite(green, HIGH);
    digitalWrite(red, LOW);
    return TRUE
  }
  else {
    digitalWrite(green, LOW);
    digitalWrite(red, HIGH);
    return FALSE
  }
  prev1 = IRState;
  prev2 = prev1; 
  prev3 = prev2;
  prev4 = prev3;
}

void setup() {
  Serial.begin(115200);

  setupRSLK();
}

void loop() {
   //Get sensor array data and compute position
  readLineSensor(sensorVal);
  readCalLineSensor(sensorVal,
                    sensorCalVal,
                    sensorMinVal,
                    sensorMaxVal,
                    lineColor);

  uint32_t linePos = getLinePosition(sensorCalVal, lineColor);

  bool atIntersection = CheckIntersection(); // Gives a true false if it detects a intersection
  if (atIntersection == TRUE && prev_state != AT_INTERSECTION) { //makes sure that the same intersection cannot add to the counter more than once
    intersection_counter += 1;
    curr_state = AT_INTERSECTION;
    Serial.println("At Intersection");

  }
  else if (atIntersection == TRUE && prev_state == AT_INTERSECTION) {//really is only for the first intersection in the function intersection_decision which tells it to keep going through it
    curr_state = AT_INTERSECTION;
    Serial.println("At Intersection");
  }
  else if (linePos >= 0 && linePos < 3000) { // linePos spits out a weighted average of sensorVal where below 3000 is the sensors on the left seeing darker
    curr_state = LEFT_OF_LINE;
    Serial.println("Right of Line");
  } 
  else if (linePos > 3500) {// and the weighted value above 3500 are the sensors on the left seeing darker
    curr_state = RIGHT_OF_LINE;
    Serial.println("Left of Line");
  } 
  else if (linePos >= 3000 && linePos <= 3500) { // between 3000 and 3500 is considered centered
    curr_state=CENTERED;
    Serial.println("Centered");
  }

  else {
    curr_state=NO_LINE;
    Serial.println("No Line");
  }
  
  switch (curr_state) {
    case CENTERED:
      keep_driving();
      break;
    case LEFT_OF_LINE:
      turn_right();
      break;
    case RIGHT_OF_LINE:
      turn_left();
      break;
    case AT_INTERSECTION:
      intersection_decision();
      break;
    case NO_LINE:
      stopper();
      break;
  }
  prev_state = curr_state; // Way of keeping track of the previous state for passing through the intersections.

}
