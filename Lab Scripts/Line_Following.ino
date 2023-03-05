/*
   Energia Robot Library for Texas Instruments' Robot System Learning Kit (RSLK)
   Line Following Example

   Summary:
   This example has the TI Robotic System Learning Kit (TI RSLK) follow a line
   using a basic line following algorithm. This example works on a dark floor with
   a white line or a light floor with a dark line. The robot first needs to be calibrated
   Then place the robot on the hit the left button again to begin the line following.

   How to run:
   1) Push left button on Launchpad to have the robot perform calibration.
   2) Robot will drive forwards and backwards by a predefined distance.
   3) Place the robot center on the line you want it to follow.
   4) Push left button again to have the robot begin to follow the line.

   Parts Info:
   o Black eletrical tape or white electrical tape. Masking tape does not work well
     with IR sensors.

   Learn more about the classes, variables and functions used in this library by going to:
   https://fcooper.github.io/Robot-Library/

   Learn more about the TI RSLK by going to http://www.ti.com/rslk

   created by Franklin Cooper Jr.
   Edited by Perry Scott

   This example code is in the public domain.
*/

#include "SimpleRSLK.h"

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

uint16_t normalSpeed = 20;
uint16_t fastSpeed = 25;

states curr_state = NO_LINE;
states prev_state;

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
void setup(){
  Serial.begin(115200);

  setupRSLK();
  /* Left button on Launchpad */
  setupWaitBtn(LP_LEFT_BTN);
  /* Red led in rgb led */
  setupLed(RED_LED);
  clearMinMax(sensorMinVal, sensorMaxVal);

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
  simpleCalibrate();
  Serial.println("Reading floor values complete");

  btnMsg = "Push left button on Launchpad to begin line following.\n";
  btnMsg += "Make sure the robot is on the line.\n";
  /* Wait until button is pressed to start robot */
  waitBtnPressed(LP_LEFT_BTN, btnMsg, RED_LED);
  delay(1000);

  enableMotor(BOTH_MOTORS);
}

void simpleCalibrate() {
  /* Set both motors direction forward */
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

// Makes a decision based on the number of intersection it has come across. This can be customizable to a desired path.
void intersection_decision() {
  
  if (intersection_counter == 1) {
    keep_driving();
  }
  else if (intersection_counter == 2) {
    // need to turn 90 degrees
    ninety_right();
  }
  else if (intersection_counter == 3) {
    // need to turn 90 degrees
    ninety_left();
  }
}

bool isCalibrationComplete = false;
void loop(){

  /* Valid values are either:
      DARK_LINE  if your floor is lighter than your line
      LIGHT_LINE if your floor is darker than your line
  */
  
  uint8_t lineColor = DARK_LINE;

  /* Run this setup only once */
  if (isCalibrationComplete == false) {
    floorCalibration();
    isCalibrationComplete = true;
  }

  //Get sensor array data and compute position
  readLineSensor(sensorVal);
  readCalLineSensor(sensorVal,
                    sensorCalVal,
                    sensorMinVal,
                    sensorMaxVal,
                    lineColor);

  uint32_t linePos = getLinePosition(sensorCalVal, lineColor);


  /*Uncomment the Serial Print code block below to observe the analog values for each of
     the 8 individual embedded IR sensors on the robot. You cann index into the sensor array
     this way.
  */
  Serial.println(linePos);
//  Serial.println("---------------");
  Serial.println(sensorVal[0]); //the left-most sensor if facing same direction as robot
//  Serial.println(sensorVal[1]);
//  Serial.println(sensorVal[2]);
//  Serial.println(sensorVal[3]);
//  Serial.println(sensorVal[4]);
//  Serial.println(sensorVal[5]);
//  Serial.println(sensorVal[6]);
  Serial.println(sensorVal[7]); //the right-most sensor if facing same direction as robot
//  Serial.println("---------------");

  bool atIntersection = CheckIntersection(); // Gives a true false if it detects a intersection

  if (intersection_counter > 2) {
    curr_state = NO_LINE;
  }
  else if (atIntersection == TRUE && prev_state != AT_INTERSECTION) { //makes sure that the same intersection cannot add to the counter more than once
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
