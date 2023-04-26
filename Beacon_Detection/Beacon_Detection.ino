#include "SimpleRSLK.h"

// IR Beacon Initialization Values
#define green GREEN_LED
#define red RED_LED
int IRbeaconLeft = 14;
int IRStateLeft;
int IRbeaconMid = 15;
int IRStateMid;
int IRbeaconRight = 17;
int IRStateRight;

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

enum decisionStates {
  STARTUP,
  DETECTING,
  SHOOTING_LEFT,
  SHOOTING_MID,
  SHOOTING_RIGHT
};

decisionStates currDecState = DETECTING;
decisionStates prevDecState = STARTUP;

void detecting() {
  if (currDecState == DETECTING) {
    if (prevDecState != currDecState) {
      detectStartTime = millis();
      detectEndTime = detectStartTime + 500000 ;
      leftBeaconCounter = 0;
      midBeaconCounter = 0;
      rightBeaconCounter = 0;
      digitalWrite(red, LOW);
      digitalWrite(green, LOW);
    }
    if (millis() < detectEndTime) {
      IRStateLeft = digitalRead(IRbeaconLeft);
      IRStateMid = digitalRead(IRbeaconMid);
      IRStateRight = digitalRead(IRbeaconRight);

      if (IRStateLeft == 0) {
        leftBeaconCounter++;
        Serial.print("LEFT: ");
        Serial.println(leftBeaconCounter);
      }
      if (IRStateMid == 0) {
        midBeaconCounter++;
        Serial.print("MID: ");
        Serial.println(midBeaconCounter);
      }
      if (IRStateRight == 0) {
        rightBeaconCounter++;
        Serial.print("RIGHT: ");
        Serial.println(rightBeaconCounter);
      }

      delay(1);


    } else {
      //set new currDecState Variable here
      if (leftBeaconCounter > midBeaconCounter && leftBeaconCounter > rightBeaconCounter) {
        currDecState = SHOOTING_LEFT;
        digitalWrite(red, HIGH);
      } else if (midBeaconCounter > leftBeaconCounter && midBeaconCounter > rightBeaconCounter) {
        currDecState = SHOOTING_MID;
        digitalWrite(green, HIGH);
        Serial.println("shoot mid");
      } else if (rightBeaconCounter > leftBeaconCounter && rightBeaconCounter > midBeaconCounter) {
        currDecState = SHOOTING_RIGHT;
        Serial.println("shoot right");
      }
      else {
        Serial.println("NONE DETECTED");
        detectStartTime = millis();
        detectEndTime = detectStartTime + 5000;
      }
    }

  }
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  setupRSLK();
  setupLed(RED_LED);
  setupLed(GREEN_LED);
  pinMode(IRbeaconLeft, INPUT_PULLUP);  // NOTE: because this is a pullup, a 1 indicates no beacon detected, 0 is yes beacon detected
  pinMode(IRbeaconMid, INPUT_PULLUP);
  pinMode(IRbeaconRight, INPUT_PULLUP);
}


void loop() {
  detecting();
  prevDecState = currDecState;

}
