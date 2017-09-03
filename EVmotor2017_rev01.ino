#include <Servo.h>
Servo steeringServo;

// assume right turn is decreasing servo angle; left turn is increasing servo angle:
const int maxServoAng = 99; // max left turn
const int minServoAng = 79; // max right turn
const int midServoAng = 90; // going straight 

// const int numStepsServoAng = 20;
const int deltaServoAng = 1;
const float k_servo = 1.0;

// Initialize
float targetDistance = 2.5, distanceRampDn = 0.0, distanceBreak = 0.0;

// pin definition
#define escPin 4
#define encoderPinL  6 //interrupt 0
#define encoderPinR  3 //interrupt 1
#define steeringServoPin 5

// define initial crusing ESC speed

// Choose ESC speed in multiples of 2.  Ramp down speed is 1/5 of initial ESC speed
int ESCSpeed = 10; // superfast if set to 25 on scale of 0 to 100
float RampDnDistance = 0.0, BreakDistance = 0.25;

volatile unsigned int encoderCountL = 0;
volatile float distanceL = 0.0;
volatile unsigned int encoderCountR = 0;
volatile float distanceR = 0.0;
float distance = 0.0;
float deltaDistance = 0.0;

// distanceRampDn + RampDnDistance = targetDistance;



// velocity in m/s
float velocity;

// define percent of total distance car travels at full speed, and percent distance car needs from breaking to complete stop
// float percentFullSpeed = 0.8, percentBreak = 0.05;

// dimensions in mm
const int radiusWheel = 30;
const int countPerRev= 38;

// function delcarlarion

// void setESCSpeed(int speed1);

void setup()
{
  Serial.begin(9600);
   
   pinMode(escPin, OUTPUT);
   pinMode(encoderPinL, INPUT);
  // digitalWrite(encoder0PinA, HIGH);       // turn on pullup resistor
  pinMode(encoderPinR, INPUT);
  // digitalWrite(encoder0PinB, HIGH);       // turn on pullup resistor
  pinMode(steeringServoPin, OUTPUT);

  setESCSpeed(0);
  delay(2000);
  setESCSpeed(5);
  delay(2000);
  setESCSpeed(0);
  
  // Left wheel encoder pin on interrupt 0 - pin 2
  attachInterrupt(0, doEncoderL, CHANGE);  
  // Right wheel encoder pin on interrupt 1 - pin 3
  attachInterrupt(1, doEncoderR, CHANGE);  
  Serial.println("MaxServo Ang, minServo Ang, deltaServoAng are:\n");
  Serial.println(maxServoAng, 8);
  Serial.println(minServoAng, 8);
  Serial.println(deltaServoAng, 8);

  steeringServo.attach(steeringServoPin);
  
  steeringServo.write(midServoAng);
  delay(2000);
  // steeringServo.write(maxServoAng);
  // delay(2000);
  // steeringServo.write(minServoAng);
  // delay(2000);
  // steeringServo.write(midServoAng);
  // divide by 1000 to convert wheel radius in mm to distance in m
  deltaDistance = (0.001/countPerRev)*2.0*3.14159*radiusWheel;
  
  Serial.println("deltaDistance is");
  Serial.println(deltaDistance, 8);
  Serial.begin (9600);
  Serial.println("start");
}



void loop()
{
 unsigned long timeInit, timeCurrent, timeInterval = 100;
 float posInit, posCurrent, velocityTarget = 0.3;
 float servoAngCmd, servoAngCorrection;
 int i, lengthString, l;
 char buffer[80];
 
 
 Serial.println("Starting in 5 sec....\n");
 delay(5000);
 
 // reset encoderPos and distance for current run
 
 encoderCountL = 0;
 encoderCountR = 0;
 distanceL = 0.0;
 distanceR = 0.0;
 
 // set steering servo so car will go straight:
 servoAngCmd = midServoAng;
 
 Serial.println("target distance choosen is: \t");
 Serial.println(targetDistance, 8);
 
 distanceRampDn = targetDistance - RampDnDistance - BreakDistance;
 Serial.println("Start ramping down when car is at: \t");
 Serial.println(distanceRampDn, 8);
 
 distanceBreak = targetDistance - BreakDistance;
 Serial.println("Start breaking when car is at: \t");
 Serial.println(distanceBreak, 8);
 
 // rampup(delayup, ESCSpeedESCSpeed, speed_step_up);
 setESCSpeed(ESCSpeed);
 
 // Measure Cruising Velocity and adjust ESC throttle to reach desired velocity
 timeInit = millis();
 timeCurrent = timeInit;
 posInit = distance;
 posCurrent = posInit;
 
 Serial.println("timeInit and posInit are");
 Serial.println(timeInit, 8);
 Serial.println(posInit, 8);
 
// distance = 0.5*distanceR + 0.5*distanceL;
 
 while (distance < targetDistance) {
  distance = 0.5*distanceR + 0.5*distanceL;
  
  /*
  Serial.println ("distance and targetDistance is");
  Serial.println (distance, 8);
  Serial.println (targetDistance, 8);
  Serial.println ("distanceL and distanceR is");
  Serial.println (distanceL, 8);
  Serial.println (distanceR, 8);
  */

   int angle = steeringServo.read();
   Serial.println("Servo Angle");
   Serial.println(angle);
    while (timeCurrent < timeInit + timeInterval){
     timeCurrent = millis();
     // Serial.println("timeCurrent is");
     // Serial.println(timeCurrent, 8);
   }
   posCurrent = distance;
  
   // time is in miliseconds, pos or distance is in meters, velocity is m/s
   velocity = 1000.0*(posCurrent - posInit)/float(timeCurrent - timeInit);
  
   timeInit = timeCurrent;
   posInit = posCurrent;
  
   // Serial.println("Current time, position, and velocity are");
   Serial.println (timeCurrent, DEC);
   Serial.println (posCurrent, 8);
    lengthString=sprintf (buffer, "Current time, position, and velocity: %u %3.8f and %3.8f", timeCurrent, posCurrent, velocity);
    
      for(int l= 0; l<=lengthString; l++){
        // Serial.print(buffer[l]);
      }
  
   // call setVelocity to adjust ESC based on measured velocity
   if (distance < distanceRampDn){
     // setVelocity(velocityTarget);
   }
  
   if (distance > distanceRampDn && distance < distanceBreak) {
     // reduce speed in two steps, 1/2, 1/4
     //for (i = 1; i<=2; i++){
     //  if ( (distance - targetDistance) > 1.0/pow(2.0, float(i)) * RampDnDistance){
         //velocityTarget = 0.5 * velocityTarget;
         //Serial.println("velocityTarget is");
         //Serial.println(velocityTarget, 8);
         //setVelocity(velocityTarget);
         ESCSpeed = ESCSpeed/5;
         setESCSpeed(ESCSpeed);
       //}
     //}
   }
   
   // Direction Control==========================================================================================================================================

   // Only start steering after wheels are moving for a combined two revolutions:
   //if ( (encoderCountL + encoderCountR)> 2*countPerRev){
   if ((encoderCountL + encoderCountR) % 76 == 0) {
      // Check if car going to the left, turn right to go straight:
      
          servoAngCorrection = (float)(k_servo*(encoderCountL - encoderCountR));
          //servoAngCmd = servoAngCmd + servoAngCorrection;
          servoAngCmd = midServoAng + servoAngCorrection;
          if ( servoAngCmd < minServoAng) { // limit servoAngCmd to be greater than min
            servoAngCmd = minServoAng;
          }
          if (servoAngCmd > maxServoAng) {
            servoAngCmd = maxServoAng;  
          }
          steeringServo.write((int)servoAngCmd);
      } 
   /*
   Serial.println("EncoderL: ");
   Serial.println(encoderCountL);
   Serial.println("EncoderR: ");
   Serial.println(encoderCountR);
   delay(500);
   */
 }// end while loop with increasing distance;
 
   // Put on break as distaneTarget has reached
   setESCSpeed(0);
 
 // allow time for the next run:
 delay(5000);
 } // end of loop()

void setESCSpeed(int speed1){
 // speed is from 0 to 100 where 0 is off and 100 is maximum speed.
 // If using comercial ESC, with servo signalling, using pulse width from 1.5ms to 2.0 ms:
 //the following maps speed values of 0-100 to angles from 0-180,
 // some speed controllers may need different values, see the ESC instructions
 // int pulseWidthMicroSec = map(speed1, 0, 100, MIN_SIGNAL, MAX_SIGNAL);
 // myESC.writeMicroseconds(pulseWidthMicroSec);   
 //
 // if using home made ESC, direct PWM:
 // 
 analogWrite(escPin, map(speed1, 0, 100, 0, 255));
} 

void doEncoderL() {
  /*
   * For more information on speeding up this process, see
   * [Reference/PortManipulation], specifically the PIND register.
   */
    encoderCountL++;
    //distance = (float)encoder0Pos/countPerRadius*2.0*3.14259*radiusSmWheel/radiusBgWheel;
    // distance in M
    distanceL = distanceL + deltaDistance;
   
  // can only print one variable value, otherwise serial COM output is truncated
  //Serial.println ("encoderCountL and distanceL calculated are:");
  //Serial.println (encoderCountL, DEC);
  //Serial.println (distanceL, 8);
  
  // delay(5);
}

void doEncoderR() {
  /*
   * For more information on speeding up this process, see
   * [Reference/PortManipulation], specifically the PIND register.
   */
    encoderCountR++;
    //distance = (float)encoder0Pos/countPerRadius*2.0*3.14259*radiusSmWheel/radiusBgWheel;
    // distance in M
    distanceR = distanceR + deltaDistance;
   
  // can only print one variable value, otherwise serial COM output is truncated
  //Serial.println ("encoderCountR and distanceR calculated are:");
  //Serial.println (encoderCountR, DEC);
  //Serial.println (distanceR, 8);
 
  // delay(5);
}


