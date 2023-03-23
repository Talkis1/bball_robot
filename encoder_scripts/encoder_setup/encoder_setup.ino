#include "SimpleRSLK.h"

uint16_t right_speed = 5;
uint16_t left_speed = 5;
uint32_t left_enc_curr;
uint32_t right_enc_curr;
uint32_t left_enc_prev;
uint32_t right_enc_prev;
uint32_t left_enc_speed;
uint32_t right_enc_speed;
int encoder_t = 20;
int prev_t;

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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  setupRSLK();
  setupEncoder(72, 12, 56, 13);
  prev_t = millis();
}

void loop() {
  // put your main code here, to run repeatedly: 
  enableMotor(BOTH_MOTORS);
  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
  setMotorSpeed(RIGHT_MOTOR,right_speed);
  setMotorSpeed(LEFT_MOTOR,left_speed);
  left_enc_curr = getEncoderLeftCnt();
  right_enc_curr = getEncoderRightCnt();
  if ((millis() - prev_t) > encoder_t) {
    left_enc_curr = getEncoderLeftCnt();
    right_enc_curr = getEncoderRightCnt();
    left_enc_speed = left_enc_curr - left_enc_prev;
    right_enc_speed = right_enc_curr - right_enc_prev;

    prev_t = millis();
    left_enc_prev = left_enc_curr;
    right_enc_prev = right_enc_curr;
    
    if (left_enc_speed < 8) {
      left_speed = left_speed+1;
    }
    else if (left_enc_speed > 8) {
      left_speed = left_speed - 1;
    }
    if (right_enc_speed < 8) {
      right_speed = right_speed+1;
    }
    else if (right_enc_speed > 8) {
      right_speed = right_speed-1;
    }
    Serial.print(right_speed);
    Serial.print("\n");
    Serial.print(left_speed);
    Serial.print("\n");
  }
  
}
