#include "SimpleRSLK.h"

// IR Beacon Initialization Values
#define green GREEN_LED
#define red RED_LED
int IRbeaconLeft = 14;
int IRStateLeft = 1;
int IRbeaconMid = 18;
int IRStateMid = 1;
int IRbeaconRight = 17;
int IRStateRight = 1;

int detectStartTime = 0;
int detectEndTime = 0;
int leftBeaconCounter = 0;
int midBeaconCounter = 0;
int rightBeaconCounter = 0;
uint16_t LED_ShootingRight = 9;


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
  Serial.println("detecting");
  if (currDecState == DETECTING) {
    if (prevDecState != currDecState) {
      detectStartTime = millis();
      detectEndTime = detectStartTime + 5000;
      leftBeaconCounter = 0;
      midBeaconCounter = 0;
      rightBeaconCounter = 0;
      digitalWrite(LED_ShootingRight, LOW);
      digitalWrite(red, LOW);
      digitalWrite(green, LOW);
    }
    if (millis() < detectEndTime) {
      Serial.println("reading");
      IRStateLeft = digitalRead(IRbeaconLeft);
      IRStateMid = digitalRead(IRbeaconMid);
      IRStateRight = digitalRead(IRbeaconRight);

      if (IRStateLeft == 0) {
        leftBeaconCounter++;
        Serial.print("leftBeaconCounter:");
        Serial.println(leftBeaconCounter);
      }
      if (IRStateMid == 0) {
        midBeaconCounter++;
        Serial.print("midBeaconCounter:");
        Serial.println(midBeaconCounter);
      }
      if (IRStateRight == 0) {
        rightBeaconCounter++;
        Serial.print("rightBeaconCounter:");
        Serial.println(rightBeaconCounter);
      }

      delay(1);


    } else {
      //set new currDecState Variable here
      if (leftBeaconCounter > midBeaconCounter && leftBeaconCounter > rightBeaconCounter) {
        currDecState = SHOOTING_LEFT;
        digitalWrite(red, HIGH);
        Serial.println("shoot left");
      } else if (midBeaconCounter > leftBeaconCounter && midBeaconCounter > rightBeaconCounter) {
        currDecState = SHOOTING_MID;
        digitalWrite(green, HIGH);
        Serial.println("shoot mid");
      } else if (rightBeaconCounter > leftBeaconCounter && rightBeaconCounter > midBeaconCounter) {
        currDecState = SHOOTING_RIGHT;
        digitalWrite(LED_ShootingRight, HIGH);
        Serial.println("shoot right");
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
