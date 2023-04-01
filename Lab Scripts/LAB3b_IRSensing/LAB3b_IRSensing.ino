#define green GREEN_LED
#define red RED_LED

int IRbeacon1 = 10;
int prev1 = 1;
int prev2 = 1;
int prev3 = 1;
int prev4 = 1;


// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600); // msp430g2231 must use 4800
  // make the on-board pushbutton's pin an input pullup:
  pinMode(IRbeacon1, INPUT_PULLUP); // NOTE: because this is a pullup, a 1 indicates no beacon detected, 0 is yes beacon detected
}

// the loop routine runs over and over again forever:
/*NOTE: This is not good code. It's blocking code due to the delay. Turn this into 
        non-blocking code by either defining a timer as in BlinkWithoutDelay or 
        attaching an interrupt. If you want an interrupt, you can use any unused
        (white) P4.x pins. That information is found in both the base pin map and User Guide.
        Software guidance for using an interrupt can be found here: 
        https://energia.nu/reference/en/language/functions/external-interrupts/attachinterrupt/
        */
void loop() {
  // read the input pin:
  int IRState = digitalRead(IRbeacon1);
  int IRComp = (IRState + prev1 + prev2 + prev3 + prev4)/5;
  // print out the state of the button:
  Serial.println(IRState); // Note - it's 0 if beacon is ON, 1 if beacon is OFF. Do some if or case statements to print out more meaningful information.
  delay(1);        // delay in between reads for stability.

  
  if (IRComp < 1) {
    digitalWrite(green, HIGH);
    digitalWrite(red, LOW);
  }
  else {
    digitalWrite(green, LOW);
    digitalWrite(red, HIGH);
  }

  prev1 = IRState;
  prev2 = prev1; 
  prev3 = prev2;
  prev4 = prev3;
}
