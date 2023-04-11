#define IN1 2
#define IN2 3
#define IN3 4
#define IN4 5
int Steps = 0;
boolean Direction = true;
void setup() {
Serial.begin(9600);
pinMode(IN1, OUTPUT);
pinMode(IN2, OUTPUT);
pinMode(IN3, OUTPUT);
pinMode(IN4, OUTPUT);
}
void loop() {
  // Turn by input angle using full stepping
//  Serial.println(1);
  if (Serial.available() != 0){
    int angle = Serial.parseInt();
    int steps = angle/0.135;
    if (steps < 0) {
      steps = steps * -1;
      Direction = false;
    }
    else {
      Direction = true;
    }
    for(int i=0; i<steps; i++){
      stepperFULL(1);
      delayMicroseconds(1500); // Don't change this number - see if you can find it on the scope!
    }
  }
    // Half step control
//  for(int i=0; i<4096; i++){ // What is 4096? You'll have to change things here to do rotational control
//    stepperHALF(1);
//    delayMicroseconds(1500); // Don't change this number - see if you can find it on the scope! 
//  }
//  Direction = !Direction;  // What does this do? 
}

void stepperFULL(int xw) {
for (int x = 0; x < xw; x++) {
switch (Steps) {
case 0:
digitalWrite(IN1, LOW);
digitalWrite(IN2, LOW);
digitalWrite(IN3, LOW);
digitalWrite(IN4, HIGH);
break;
case 1:
digitalWrite(IN1, LOW);
digitalWrite(IN2, LOW);
digitalWrite(IN3, HIGH);
digitalWrite(IN4, LOW);
break;
case 2:
digitalWrite(IN1, LOW);
digitalWrite(IN2, HIGH);
digitalWrite(IN3, LOW);
digitalWrite(IN4, LOW);
break;
case 3:
digitalWrite(IN1, HIGH);
digitalWrite(IN2, LOW);
digitalWrite(IN3, LOW);
digitalWrite(IN4, LOW);
break;
default:
digitalWrite(IN1, LOW);
digitalWrite(IN2, LOW);
digitalWrite(IN3, LOW);
digitalWrite(IN4, LOW);
break;
}
SetDirectionFULL();
}
} // END StepperFull() 

void SetDirectionFULL() {
if (Direction == 1) {
Steps++;
}
if (Direction == 0) {
Steps--;
}
if (Steps > 4) {
Steps = 0;
}
if (Steps < 0) {
Steps = 4;
}
}

/* FILL OUT YOUR CODE HERE FOR HALF STEPS!!*/

void stepperHALF(int xw) {
for (int x = 0; x < xw; x++) {
switch (Steps) {
case 0:
digitalWrite(IN1, LOW);
digitalWrite(IN2, LOW);
digitalWrite(IN3, LOW);
digitalWrite(IN4, HIGH);
break;
case 1:
digitalWrite(IN1, LOW);
digitalWrite(IN2, LOW);
digitalWrite(IN3, HIGH);
digitalWrite(IN4, HIGH);
break;
case 2:
digitalWrite(IN1, LOW);
digitalWrite(IN2, LOW);
digitalWrite(IN3, HIGH);
digitalWrite(IN4, LOW);
break;
case 3:
digitalWrite(IN1, LOW);
digitalWrite(IN2, HIGH);
digitalWrite(IN3, HIGH);
digitalWrite(IN4, LOW);
break;
case 4:
digitalWrite(IN1, LOW);
digitalWrite(IN2, HIGH);
digitalWrite(IN3, LOW);
digitalWrite(IN4, LOW);
break;
case 5:
digitalWrite(IN1, HIGH);
digitalWrite(IN2, HIGH);
digitalWrite(IN3, LOW);
digitalWrite(IN4, LOW);
break;
case 6:
digitalWrite(IN1, HIGH);
digitalWrite(IN2, LOW);
digitalWrite(IN3, LOW);
digitalWrite(IN4, LOW);
break;
case 7:
digitalWrite(IN1, HIGH);
digitalWrite(IN2, LOW);
digitalWrite(IN3, LOW);
digitalWrite(IN4, HIGH);
break;
default:
digitalWrite(IN1, LOW);
digitalWrite(IN2, LOW);
digitalWrite(IN3, LOW);
digitalWrite(IN4, LOW);
break;
}
SetDirectionHALF();
}
}
void SetDirectionHALF() {
if (Direction == 1) {
Steps++;
}
if (Direction == 0) {
Steps--;
}
if (Steps > 7) {
Steps = 0;
}
if (Steps < 0) {
Steps = 7;
}
}

void stepperTORQUE(int xw) {
for (int x = 0; x < xw; x++) {
switch (Steps) {
case 0:
digitalWrite(IN1, HIGH);
digitalWrite(IN2, LOW);
digitalWrite(IN3, LOW);
digitalWrite(IN4, HIGH);
break;
case 1:
digitalWrite(IN1, LOW);
digitalWrite(IN2, LOW);
digitalWrite(IN3, HIGH);
digitalWrite(IN4, HIGH);
break;
case 2:
digitalWrite(IN1, LOW);
digitalWrite(IN2, HIGH);
digitalWrite(IN3, HIGH);
digitalWrite(IN4, LOW);
break;
case 3:
digitalWrite(IN1, HIGH);
digitalWrite(IN2, HIGH);
digitalWrite(IN3, LOW);
digitalWrite(IN4, LOW);
break;
default:
digitalWrite(IN1, LOW);
digitalWrite(IN2, LOW);
digitalWrite(IN3, LOW);
digitalWrite(IN4, LOW);
break;
}
SetDirectionFULL();
}
} // END StepperFull() 
