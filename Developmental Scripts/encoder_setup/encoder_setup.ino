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
int encoder_t = 20;
int prev_t;
bool calibrated = false;
bool leftCorrect = false;
int correctT = 0;
int turningT = 0;
int startT = 0;
bool aligned;

uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS];
uint16_t sensorMinVal[LS_NUM_SENSORS];
int intersection_counter = 0;

uint8_t lineColor = DARK_LINE;

enum states {
  CENTERED,
  LEFT_OF_LINE,
  RIGHT_OF_LINE,
  CORRECT_LEFT,
  CORRECT_RIGHT,
  AT_INTERSECTION,
  NO_LINE
};

states curr_state = NO_LINE;
states prev_state;

void keep_driving() {

  uint32_t* values = getMotorSpeed(normalEncSpeed, leftSpeed, normalEncSpeed, rightSpeed);
  leftSpeed = values[0];
  rightSpeed = values[1];

  enableMotor(BOTH_MOTORS);
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
  setMotorSpeed(RIGHT_MOTOR, rightSpeed);
  setMotorSpeed(LEFT_MOTOR, leftSpeed);
}

void turn_right() {

  uint32_t* values = getMotorSpeed(fastEncSpeed, leftSpeed, normalEncSpeed, rightSpeed);
  leftSpeed = values[0];
  rightSpeed = values[1];

  enableMotor(BOTH_MOTORS);
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
  setMotorSpeed(RIGHT_MOTOR, rightSpeed);
  setMotorSpeed(LEFT_MOTOR, leftSpeed);
}

void turn_left() {

  uint32_t* values = getMotorSpeed(normalEncSpeed, leftSpeed, fastEncSpeed, rightSpeed);
  leftSpeed = values[0];
  rightSpeed = values[1];

  enableMotor(BOTH_MOTORS);
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
  setMotorSpeed(LEFT_MOTOR, leftSpeed);
  setMotorSpeed(RIGHT_MOTOR, rightSpeed);
}

void ninety_right() { // needs to be tuned for a 90 degree turn through either time or encoder
  int start = millis();
  int curr = millis();
  while ((curr - start) < 700) {
    enableMotor(BOTH_MOTORS);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorSpeed(RIGHT_MOTOR, normalEncSpeed);
    setMotorSpeed(LEFT_MOTOR, normalEncSpeed);
    curr = millis();
  }
}

void ninety_left() { // needs to be tuned for a 90 degree turn through either time or encoder
  int start = millis();
  int curr = millis();
  while ((curr - start) < 700) {
    enableMotor(BOTH_MOTORS);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorSpeed(LEFT_MOTOR, normalEncSpeed);
    setMotorSpeed(RIGHT_MOTOR, normalEncSpeed);
    curr = millis();
  }
}
void stopper() {
  disableMotor(BOTH_MOTORS);
}

uint32_t* getMotorSpeed(uint16_t leftEncSpeed, uint16_t leftMotorSpeed, uint16_t rightEncSpeed, uint16_t rightMotorSpeed) {
  if ((millis() - prev_t) > encoder_t) {
    left_enc_curr = getEncoderLeftCnt();
    right_enc_curr = getEncoderRightCnt();
    left_enc_speed = left_enc_curr - left_enc_prev;
    right_enc_speed = right_enc_curr - right_enc_prev;

    prev_t = millis();
    left_enc_prev = left_enc_curr;
    right_enc_prev = right_enc_curr;

    if (left_enc_speed < leftEncSpeed) {
      leftMotorSpeed = leftMotorSpeed + 1;
    }
    else if (left_enc_speed > leftEncSpeed) {
      leftMotorSpeed = leftMotorSpeed - 1;
    }
    if (right_enc_speed < rightEncSpeed) {
      rightMotorSpeed = rightMotorSpeed + 1;
    }
    else if (right_enc_speed > rightEncSpeed) {
      rightMotorSpeed = rightMotorSpeed - 1;
    }
  }

  static uint32_t values[2];
  values[0] = leftMotorSpeed;
  values[1] = rightMotorSpeed;

  return values;
}

void followLine() {
  readLineSensor(sensorVal);
  readCalLineSensor(sensorVal,
                    sensorCalVal,
                    sensorMinVal,
                    sensorMaxVal,
                    lineColor);

  uint32_t linePos = getLinePosition(sensorCalVal, lineColor);

  if (CheckIntersection()) {
    if (prev_state != AT_INTERSECTION) {
      aligned = false;
    }
    curr_state = AT_INTERSECTION;
    Serial.println("At Intersection");

  }
  else if (linePos >= 0 && linePos < 3000) { // linePos spits out a weighted average of sensorVal where below 3000 is the sensors on the left seeing darker
    if (prev_state == CENTERED) {
      startT = millis();
    }
    curr_state = RIGHT_OF_LINE;
    Serial.println("Right of Line");
  }
  else if (linePos > 3500) {// and the weighted value above 3500 are the sensors on the left seeing darker
    if (prev_state == CENTERED) {
      startT = millis();
    }
    curr_state = LEFT_OF_LINE;
    Serial.println("Left of Line");
  }
  else if (linePos >= 3000 && linePos <= 3500) { // between 3000 and 3500 is considered centered
    if (prev_state == LEFT_OF_LINE) {
      leftCorrect = true;
      correctT = millis() - startT;
      startT = millis();
    }
    else if (prev_state == RIGHT_OF_LINE) {
      leftCorrect = false;
      correctT = (millis() - startT) / 2;
      startT = millis();
    }
    if (leftCorrect && (millis() - startT) < correctT) {
      curr_state = CORRECT_LEFT;
      Serial.println("Correct Left");
    }
    else if (!leftCorrect && (millis() - startT) < correctT) {
      curr_state = CORRECT_RIGHT;
      Serial.println("Correct Right");
    }
    else {
      curr_state = CENTERED;
      Serial.println("Centered");
    }
  }
  else {
    curr_state = NO_LINE;
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
    case CORRECT_LEFT:
      turn_left();
      break;
    case CORRECT_RIGHT:
      turn_right();
      break;
    case AT_INTERSECTION:
      if (!aligned) {
        aligned = AlignAtIntersection();
        break;
      } else {

      }
      break;
    case NO_LINE:
      stopper();
      break;
  }

  prev_state = curr_state;

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

// This function ensures that the robots line sensors are aligned in parrellel with the intersection
bool AlignAtIntersection() { // indicates the left edge of robot hit the intersection first
  uint32_t* speed = getMotorSpeed(normalEncSpeed, leftSpeed, normalEncSpeed, rightSpeed);

  if (sensorVal[0] > 2000) {
    if (sensorVal[7] > 2000) {
      stopper();
      return true;
    } else {
      //enableMotor(RIGHT_MOTOR); // turns on right motor to bring the right side of the robot even with the left side
      //disableMotor(LEFT_MOTOR);
      enableMotor(BOTH_MOTORS);
      setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
      setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
      setMotorSpeed(LEFT_MOTOR, speed[0]);
      setMotorSpeed(RIGHT_MOTOR, speed[1]); // update speed in future to be non hardcoded
      return false;
    }
  } else if (sensorVal[7] > 2000) {
    if (sensorVal[0] > 2000) {
      stopper();
      return true;
    } else {
      //enableMotor(LEFT_MOTOR); // turns on right motor to bring the right side of the robot even with the left side
      //disableMotor(RIGHT_MOTOR);
      enableMotor(BOTH_MOTORS);
      setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
      setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
      setMotorSpeed(LEFT_MOTOR, speed[0]); // update speed in future to be non hardcoded
      setMotorSpeed(RIGHT_MOTOR,speed[1]);
      return false;
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


}

void loop() {
  // put your main code here, to run repeatedly:
  if (!calibrated) {
    floorCalibration();
    calibrated = true;
  }
  followLine();
}
