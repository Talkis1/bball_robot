/*
 * DC Motor Control Template
 * Written By: Ann Majewicz Fey, Perry Scott, and inspired by online resources.
 * 
 * In this code, you will first convert the encoder reading to revolutions per second
 * and then you will take that variable as an input to you PID controller (already programmed
 * as a function you can use). First, you will need to use Zeigler Nicholas and brute force 
 * to tune the gains of your PID controller. Notice how your motor response is affected by the
 * different gain terms. Spend some time playing around with it to get a better understanding 
 * of real-time control systems! 
 */

// Constant Definitions
int countsPerRev = 48;
double maxSpeed = 66.7; //in rev/sec (this is found experimentally, you may need to confirm the value)
double delayTime = 20; // in ms

// Pin Definitions
const int PWMoutp = 19;
const int PWMoutn = 34;
const int PWMspeedPin = 38;
const int potentiometerPWMinput = 33;
const int encoder0PinA = 5;
const int encoder1PinA = 36;
const int encoder0PinB = 6;
const int encoder1PinB = 37;

// Variable Definitions
volatile signed int encoder0Pos = 0;
signed int encoderPosLast = 0;
int potPWMval = 0;
int motorPWMval = 0;

//PID constants (Use the Ziegler Nicholas Tuning Method as a first guess)
double kp = 4;
double ki = 0.04;
double kd = 10;

// PID Variables 
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastSpeedError;
double motorSpeed, newMotorSpeed, setPoint;
double cumError, rateError;
double out;

void setup()
{
  // Don't Change
 Serial.begin(115200);
 pinMode(potentiometerPWMinput,INPUT_PULLUP);
 pinMode(encoder0PinA, INPUT_PULLUP);
 pinMode(encoder0PinB, INPUT_PULLUP);
 pinMode(encoder1PinA, INPUT_PULLUP);
 pinMode(encoder1PinB, INPUT_PULLUP);
 pinMode(PWMoutp, OUTPUT);
 pinMode(PWMoutn, OUTPUT);
 pinMode(PWMspeedPin, OUTPUT);
 attachInterrupt(encoder0PinA, doEncoderA, RISING); // Interrupt is fired whenever button is pressed
 attachInterrupt(encoder1PinA, doEncoderA, FALLING);
 attachInterrupt(encoder0PinB, doEncoderB, RISING);
 attachInterrupt(encoder1PinB, doEncoderB, FALLING);

 // Pre-set the direction of the motors
 digitalWrite(PWMoutp, HIGH);
 digitalWrite(PWMoutn, LOW);

 // USE FOR PID CONTROL 
 setPoint = 0;                         //set point at zero rev/second

}

void loop()
{
  // POTENTIOMETER CONTROL]
  
  potPWMval = analogRead(potentiometerPWMinput);
  potPWMval = map(potPWMval,0,1023,-255,255);
  //potPWMval = map(potPWMval,0,1023,-maxSpeed,maxSpeed);
  setPoint = map(potPWMval, -255, 255, -140, 140);

  if(potPWMval < 0){
    digitalWrite(PWMoutp, HIGH);
    digitalWrite(PWMoutn, LOW);
  }
  else
  {
  digitalWrite(PWMoutp, LOW);
  digitalWrite(PWMoutn, HIGH);
  }


  motorSpeed = (encoder0Pos-encoderPosLast)*1000/countsPerRev/delayTime; // Here you will need to compute the motor speed in counts/rev. 
  // Hint: you just need to convert the encoder counts to Rev/Sec!

  // HERE YOU WILL WRITE YOUR CODE TO ENABLE THE PID CONTROLLER. Use the function below!!
  /* Initially, just have the PID controller follow the pre-defined
      setPoint variable. For fun, you can later consider how to tie the PID 
      controller to the potenitometer value to have an adaptively changing setPoint.*/
  out = computePID(motorSpeed);
  motorSpeed = motorSpeed + out;
  motorPWMval = map(motorSpeed, -140, 140,-255, 255);
  if(motorPWMval < 0){
    digitalWrite(PWMoutp, HIGH);
    digitalWrite(PWMoutn, LOW);
  }
  else {
    digitalWrite(PWMoutp, LOW);
    digitalWrite(PWMoutn, HIGH);
  }
  analogWrite(PWMspeedPin,abs(motorPWMval));
      
        
  //print out speed (currently is only plotting the pot value)
     //Serial.println(motorSpeed); // CHANGE THIS TO PLOT MOTOR SPEED
 
  encoderPosLast = encoder0Pos;
  delay(delayTime); // This is a delay to time the control loop. In the future, this should be a non-blocking version.

}

// **** DONT CHANGE THE FUNCTIONS BELOW ****
//================
// READ ENCODER A
//================
void doEncoderA(){
  if (digitalRead(encoder0PinA) == HIGH) {   // found a low-to-high on channel A
    if (digitalRead(encoder0PinB) == LOW) {  // check channel B to see which way
                                             // encoder is turning
      encoder0Pos = encoder0Pos - 1;         // CCW
    } 
    else {
      encoder0Pos = encoder0Pos + 1;         // CW
}
  }
  else                                        // found a high-to-low on channel A
  { 
    if (digitalRead(encoder0PinB) == LOW) {   // check channel B to see which way
                                              // encoder is turning  
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
      
  }
}
//================
// READ ENCODER B
//================
void doEncoderB(){
  if (digitalRead(encoder0PinB) == HIGH) {   // found a low-to-high on channel A
    if (digitalRead(encoder0PinA) == LOW) {  // check channel B to see which way
                                             // encoder is turning
      encoder0Pos = encoder0Pos + 1;         // CCW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CW
}
  }
  else                                        // found a high-to-low on channel A
  { 
    if (digitalRead(encoder0PinA) == LOW) {   // check channel B to see which way
                                              // encoder is turning  
      encoder0Pos = encoder0Pos - 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos + 1;          // CCW
    }
      
  }
}
//================
// PID CONTROL FUNCTION (use this!)
//================
double computePID(double inp){  
        currentTime = millis();                //get current time
        elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
        
        error = setPoint - inp;                         // determine error
        cumError += error * elapsedTime;                // compute integral
        rateError = (error - lastSpeedError)/elapsedTime;   // compute derivative

        if (cumError > 3000) {
          cumError = 3000;
        }
        if (cumError < -3000) {
          cumError = -3000;
        }
 
 
        double out = kp*error + ki*cumError + kd*rateError;  //PID output    

        if (out > 140) {
          out = 140;
        }
        if (out < -140) {
          out = -140;
        }
 
        lastSpeedError = error;                            //remember current error
        previousTime = currentTime;                        //remember current time
        Serial.println(error);
        return out;                                        //have function return the PID output
}
