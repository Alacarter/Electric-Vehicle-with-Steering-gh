#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
Servo steeringServo;

#define OUTPUT_READABLE_YAWPITCHROLL

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

//===|||||||||||||||||||||||||||||||||||IMPORTANT VARIABLES||||||||||||||||||||||||||||||||||||||||||||===

float targetDistance = 9.0, distanceRampDn = 2.5, distanceBreak = 0.0; // distanceBreak = how many m before finish line it starts to break
float midSegment = 1.0; // distance of middle segment of the run (through the cans)
float horizOffset = 0.8; // Car will ultimately be 0.8 meters right of the imaginary center line at the mid point on the track.

// Choose ESC speed in multiples of 2.  Ramp down speed is 1/5 of initial ESC speed
int midESCSpeed = 15; // superfast if set to 25 on scale of 0 to 100
float RampDnDistance = 0.0; // at what place, 0-12m, to start ramping down.
float BreakDistance = 0.25; //Set to 8m

float targetVelocityStart = 2.0; // in m/s
float targetVelocityMid = 2.0; // in m/s
float targetVelocityFinal = 0.1;

float k_p = 0.5; // 1.0 seems to work best. Maybe lower would be good as well.
const float k_i = 0.0;
const float k_d = 0.0;

//velocity constants for velocity PID
const float v_p = 1.0;
const float v_i = 1.0;
const float v_d = 0.0;

// assume right turn is decreasing servo angle; left turn is increasing servo angle:
const int maxServoAng = 92; // max left turn
const int minServoAng = 82; // max right turn
const int midServoAng = 87; // going straight 

int adjTimeSteering = 5; // 100 = around 1 second; adjustment time for steering
int adjTimeVelocity = 100;
//===||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||===

bool blinkState = false;
bool ready = false;
bool doneY_0 = false;
bool blinkMega;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// velocity in m/s
float velocity;
float distanceI, distanceF, distanceTraveled; // variables for calculating velocity
int timeI, timeF;
float velocity_N, velocity_NMinus1, deltaVelocity;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yaw;
float yawF, yawI;
float yawStart;
float Y_0;
float turnAng; // angle to turn the car left, twice during the run.
float yaw_N, yaw_NMinus1, deltaYaw;

float beginningSegment; // Distance of the first segment (starting point to where the car starts to turn)
float finalTurnDirection;



// const int numStepsServoAng = 20;
const int deltaServoAng = 1;




// pin definition
#define escPin 4
#define encoderPinL  18 //interrupt 0
#define encoderPinR  3 //interrupt 1
#define steeringServoPin 5

#define gyroPin 2 // If you switch the pin, it doesn't work.
// define initial crusing ESC speed



volatile unsigned int encoderCountL = 0;
volatile float distanceL = 0.0;
volatile unsigned int encoderCountR = 0;
volatile float distanceR = 0.0;
float distance = 0.0;
float deltaDistance = 0.0;
float distance_pre = 0.0;

//variables from loop
 unsigned long timeInit, timeCurrent, timeInterval = 100;
 float posInit, posCurrent; //velocityTarget = 0.3;
 float servoAngCmd, servoAngProportionalCorrection, servoAngIntegralError, integralSSError, servoAngIntegralCorrection, servoAngDerivativeCorrection;
 int i, lengthString, l;
 char buffer[80];
//end variables from loop

//variables for speed PID
float ESCSpeedCmd, maxESCSpeed = 20;
const int maxTauVelocity = 10;
int integralTauVelocity = 3;
float ESCSpeedProportionalCorrection, ESCSpeedIntegralError, integralSSVelocityError, ESCSpeedIntegralCorrection, ESCSpeedDerivativeCorrection;
float ESCSpeedCorrectionError[maxTauVelocity];


const int maxTau = 10; //number of readings to keep to calculate integral term
int tau = 0; // tau counter for the array servoAngCorrectionError[]
int integralTau = 6; //number of readings to summation
float servoAngCorrectionError[maxTau];


// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// distanceRampDn + RampDnDistance = targetDistance;





// define percent of total distance car travels at full speed, and percent distance car needs from breaking to complete stop
// float percentFullSpeed = 0.8, percentBreak = 0.05;

// dimensions in mm
const int radiusWheel = 30;
const int countPerRev= 38;

// function delcarlarion

// void setESCSpeed(int speed1);


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high


void setup()
{
  Serial.begin(115200);
   
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
  attachInterrupt(digitalPinToInterrupt(encoderPinL), doEncoderL, CHANGE);  
  // Right wheel encoder pin on interrupt 1 - pin 3
  attachInterrupt(digitalPinToInterrupt(encoderPinR), doEncoderR, CHANGE);  
  /*
  //SetServo
  attachInterrupt(digitalPinToInterrupt(steeringServoPin), setServoStart, CHANGE);
  attachInterrupt(digitalPinToInterrupt(steeringServoPin), setServoMid, CHANGE);
  attachInterrupt(digitalPinToInterrupt(steeringServoPin), setServoFinal, CHANGE);
  //Gyro Proportional
  attachInterrupt(digitalPinToInterrupt(gyroPin), doGyro, CHANGE);
  //Integral
  attachInterrupt(digitalPinToInterrupt(gyroPin), storeSSErrorStart, CHANGE);
  attachInterrupt(digitalPinToInterrupt(gyroPin), storeSSErrorMid, CHANGE);
  attachInterrupt(digitalPinToInterrupt(gyroPin), storeSSErrorFinal, CHANGE);
  attachInterrupt(digitalPinToInterrupt(gyroPin), sumSSError, CHANGE);
  //Gyro Differential
  attachInterrupt(digitalPinToInterrupt(gyroPin), calcServoAngDerivativeCorrection, CHANGE);
  attachInterrupt(digitalPinToInterrupt(gyroPin), calcDeltaYaw, CHANGE);

  attachInterrupt(digitalPinToInterrupt(escPin), calcVelocity, CHANGE);
  //SetESC
  attachInterrupt(digitalPinToInterrupt(escPin), setVelocityStart, CHANGE);
  attachInterrupt(digitalPinToInterrupt(escPin), setVelocityMid, CHANGE);
  attachInterrupt(digitalPinToInterrupt(escPin), setVelocityFinal, CHANGE);
  //Integral
  attachInterrupt(digitalPinToInterrupt(escPin), storeSSVelocityErrorStart, CHANGE);
  attachInterrupt(digitalPinToInterrupt(escPin), storeSSVelocityErrorMid, CHANGE);
  attachInterrupt(digitalPinToInterrupt(escPin), storeSSVelocityErrorFinal, CHANGE);
  attachInterrupt(digitalPinToInterrupt(escPin), sumSSVelocityError, CHANGE);
  // Differential
  attachInterrupt(digitalPinToInterrupt(escPin), calcESCSpeedDerivativeCorrection, CHANGE);
  attachInterrupt(digitalPinToInterrupt(escPin), calcDeltaVelocity, CHANGE);
  */
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

  //GYRO void setup stuff
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220); // 220
    mpu.setYGyroOffset(76); // 76
    mpu.setZGyroOffset(-85); //-85
    mpu.setZAccelOffset(1788); // 1788

    // make sure it worked (returns 0 if so)

    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    pinMode(LED_PIN, OUTPUT);
  //end of GYRO void setup stuff
  
  Serial.println("start");
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void loop()
{
  if (doneY_0 == false) {
    doGyro();
    checkY_0Consistency();
    checkY_0Consistency();
  }
  else if (doneY_0 == true) {
    //if (blinkMega == false) {
      //flashBeethovenNo5();
      //flashBeethovenNo5();
      //blinkMega = true;
    //}
    doGyro();
    checkReadyConsistency();
    if (ready == false) {
      calcBeginningSegment();
      calcTurnAng();
      Serial.print("\nCalculated turn Angle: ");
      Serial.println(turnAng);
      Serial.print("\nCalculated yawStart: ");
      Serial.println(yawStart);
      delay(500);
      Serial.print("\ncurrent yaw: ");
      Serial.println(yaw);
      delay(500);
      calcFinalTurnDirection();
      // Loop again. It'll checkReadyConsistency() again.
    }
    else if (ready == true) {
      initialEngineLoop();
      ESCSpeedCmd = midESCSpeed;
      while (distance < 0.05) {
        runMotorInput();
        calcVelocity();
      }
      while (distance < beginningSegment && distance >= 0.05) {
        runMotorInput();
        setESCSpeed((int)(ESCSpeedCmd));
              
        // Direction Control
        doGyro();
        setServoStart();
        storeSSErrorStart();
        sumSSError();
        calcDeltaYaw();
        calcServoAngDerivativeCorrection();

        
        // Velocity Control
        calcVelocity();
        setVelocityStart();
        storeSSVelocityErrorStart();
        sumSSVelocityError();
        calcDeltaVelocity();
        calcESCSpeedDerivativeCorrection();
        
        
      }// end while loop for BEGINNING SEGMENT
      while (distance < (beginningSegment + midSegment) && distance >= beginningSegment) {
        runMotorInput();
        setESCSpeed((int)(ESCSpeedCmd));

        
        // Direction Control
        doGyro();
        setServoMid();
        storeSSErrorMid();
        sumSSError();
        calcDeltaYaw();
        calcServoAngDerivativeCorrection();
        
        //Velocity Control
        calcVelocity();
        setVelocityMid();
        storeSSVelocityErrorMid();
        sumSSVelocityError();
        calcDeltaVelocity();
        calcESCSpeedDerivativeCorrection();
        
      }
      while (distance < (2*beginningSegment + midSegment - distanceBreak - distanceRampDn) && distance >= (beginningSegment + midSegment)) {
        runMotorInput();
        setESCSpeed((int)(ESCSpeedCmd));
        
        // Direction Control
        doGyro1();
        setServoFinal();
        storeSSErrorFinal();
        sumSSError();
        calcDeltaYaw();
        calcServoAngDerivativeCorrection();
        
        //Velocity Control
        calcVelocity();
        setVelocityMid();
        storeSSVelocityErrorMid();
        sumSSVelocityError();
        calcDeltaVelocity();
        calcESCSpeedDerivativeCorrection();
        
        
      }
      while (distance < (2*beginningSegment + midSegment - distanceBreak) && distance >= (2*beginningSegment + midSegment - distanceBreak - distanceRampDn)) {
        runMotorInput();
        setESCSpeed((int)(ESCSpeedCmd));
        
        // Direction Control
        doGyro();
        setServoFinal();
        storeSSErrorFinal();
        sumSSError();
        calcDeltaYaw();
        calcServoAngDerivativeCorrection();
        
        //Velocity Control
        calcVelocity();
        setVelocityFinal();
        storeSSVelocityErrorFinal();
        sumSSVelocityError();
        calcDeltaVelocity();
        calcESCSpeedDerivativeCorrection();
        
      }
    }
    // Put on break as distaneTarget has reached
    setESCSpeed(0);
    Serial.println(2*beginningSegment + midSegment - distanceBreak);
    // allow time for the next run:
    delay(5000);
  } 
}// end of loop()
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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

void initialEngineLoop() { // distance calculations, etc
  Serial.println("Starting in 1 sec....\n");
 delay(1000);
 
 // reset encoderPos and distance for current run
 
 encoderCountL = 0;
 encoderCountR = 0;
 distanceL = 0.0;
 distanceR = 0.0;
 
 // set steering servo so car will go straight:
 servoAngCmd = midServoAng;
 
 Serial.println("target distance choosen is: \t");
 Serial.println(targetDistance, 8);
 
 //distanceRampDn = targetDistance - RampDnDistance - BreakDistance;
 Serial.println("Start ramping down when car is at: \t");
 Serial.println(distanceRampDn, 8);
 
 //distanceBreak = targetDistance - BreakDistance;
 Serial.println("Start breaking when car is at: \t");
 Serial.println(distanceBreak, 8);
 
 // rampup(delayup, ESCSpeedESCSpeed, speed_step_up);
 setESCSpeed(midESCSpeed);
 
 // Measure Cruising Velocity and adjust ESC throttle to reach desired velocity
 timeInit = millis();
 timeCurrent = timeInit;
 posInit = distance;
 posCurrent = posInit;
 
 Serial.println("timeInit and posInit are");
 Serial.println(timeInit, 8);
 Serial.println(posInit, 8);
 
// distance = 0.5*distanceR + 0.5*distanceL;
}

void runMotorInput() { //makes sure motor stops at target distance
   if ((abs(distanceL-distanceR)) < 0.1*distance) {
     distance = 0.5* distanceL + 0.5* distanceR;
   }
   else {
     distance = max(distanceL, distanceR);
   }
   /*
   if (abs(distanceR-distanceL) < 0.1*distance_pre)
   {
     distance = avgDistance;
   }
   if (distanceR > distanceL)
   {
     distance = distanceR;
   }
   if (distanceL > distanceR)
   {
     distance = distanceL;
   }
   */
   int angle = steeringServo.read();
   Serial.println("Servo Angle");
   Serial.println(angle);
   // Serial.println("Current time, position, and velocity are");
   Serial.println (timeCurrent, DEC);
   Serial.println("Distance:");
   Serial.println(distance);
   // call setVelocity to adjust ESC based on measured velocity
   /*
   if ((distance > (2*beginningSegment + midSegment - distanceBreak - distanceRampDn)) && (distance < (2*beginningSegment + midSegment - distanceBreak))){
     setESCSpeed(9); //7 works when car is in the air. Slowest possible is 8.
   }
   if (distance > distanceRampDn && distance < distanceBreak) {
   }
   */
}




//GYRO STUFF ------------------------------------------------------------------------------------------------------------
void dmpDataReady() {
    mpuInterrupt = true;
}

void doGyroOnce() {
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //Jan28 2am Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;


        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            yaw = ypr[0] * 180/M_PI;
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.println(yaw);
        #endif


        // blink LED to indicate activity
        //blinkState = !blinkState;
        //digitalWrite(LED_PIN, blinkState);
    }
}

void doGyro() {
  yawI = yaw;
  for (int i = 0; i < adjTimeSteering; i++)
  {
    doGyroOnce();
  }
  yawF = yaw;
}

void doGyro1() {
  yawI = yaw;
  for (int i = 0; i < 1; i++)
  {
    doGyroOnce();
  }
  yawF = yaw;
}

void checkY_0Consistency() {
  if (abs(yawI-yawF) < 0.03)
  {
    //Serial.println("Ready to go.");
    doneY_0 = true;
    Y_0 = yaw;
  }
  else
  {
    //Serial.println("Not ready.");
    doneY_0 = false;
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
    delay(1000);
  }
  //delay(500);
}

void checkReadyConsistency() {
  if (abs(yawI-yawF) < 0.1)
  {
    //Serial.println("Ready to go.");
    if (abs(yawStart - yaw) < 0.5)
    {
      ready = true;
      yawStart = yaw;
    }
  }
  else
  {
    //Serial.println("Not ready.");
    ready = false;
  }
  //delay(500);
}

//End Gyro Stuff ------------------------------------------------------------------------------------------------------------


void calcBeginningSegment() {
  beginningSegment = sqrt(sq((targetDistance - midSegment)/2) + sq(horizOffset));
}

void calcTurnAng() {
  turnAng = atan(horizOffset/((targetDistance-midSegment)/2));
  turnAng = (turnAng * 180.0)/3.1416; // convert to degrees
  if (turnAng < 0.0) // will probably never be negative. But just in case...
  {
    turnAng = -1 * turnAng;
  }
  yawStart = Y_0 + turnAng;
  if (yawStart > 180.0)
  {
    yawStart = yawStart - 360.0;
  }
}

void calcFinalTurnDirection() {
  finalTurnDirection = Y_0 - turnAng;
  if (finalTurnDirection < -180.0)
  {
    finalTurnDirection = finalTurnDirection + 360.0;
  }
}

//couldn't create setServo or storeSSError with parameters b/c then it can't interrupt

void setServoStart() {
  servoAngCmd = midServoAng;
  if (abs(yaw-yawStart)>1)
  {
          servoAngProportionalCorrection = (float)(k_p*(yaw-yawStart));
          servoAngCmd = midServoAng + servoAngProportionalCorrection + servoAngIntegralCorrection + servoAngDerivativeCorrection; // Decreasing yaw = turning left
          if ( servoAngCmd < minServoAng) { 
            servoAngCmd = minServoAng;
          }
          if (servoAngCmd > maxServoAng) {
            servoAngCmd = maxServoAng;  
          }
          steeringServo.write((int)(servoAngCmd));
  } 
}

void setServoMid() {
  servoAngCmd = midServoAng;
  if (abs(yaw-Y_0)>1)
  {
          servoAngProportionalCorrection = (float)(k_p*(yaw-Y_0));
          servoAngCmd = midServoAng + servoAngProportionalCorrection + servoAngIntegralCorrection + servoAngDerivativeCorrection; // Decreasing yaw = turning left
          if (servoAngCmd < minServoAng) { 
            servoAngCmd = minServoAng;
          }
          if (servoAngCmd > maxServoAng) {
            servoAngCmd = maxServoAng;  
          }
          steeringServo.write((int)(servoAngCmd));
  } 
}

void setServoFinal() {
  servoAngCmd = midServoAng;
  if (abs(yaw-finalTurnDirection)>1)
  {
          servoAngProportionalCorrection = (float)(k_p*(yaw-finalTurnDirection));
          servoAngCmd = midServoAng + servoAngProportionalCorrection + servoAngIntegralCorrection + servoAngDerivativeCorrection; // Decreasing yaw = turning left
          if (servoAngCmd < minServoAng) { 
            servoAngCmd = minServoAng;
          }
          if (servoAngCmd > maxServoAng) {
            servoAngCmd = maxServoAng;  
          }
          steeringServo.write((int)(servoAngCmd));
  } 
}


void storeSSErrorStart() {
  servoAngIntegralError = (float)(yaw-yawStart);
  for (int tauStart = maxTau-2; tauStart >= 0; tauStart--)
  {
    servoAngCorrectionError[tauStart] = servoAngCorrectionError[tauStart+1];
  }
  servoAngCorrectionError[0] = servoAngIntegralError;
}

void storeSSErrorMid() {
  servoAngIntegralError = (float)(yaw-Y_0);
  for (int tauMid = maxTau-2; tauMid >= 0; tauMid--)
  {
    servoAngCorrectionError[tauMid] = servoAngCorrectionError[tauMid+1];
  }
  servoAngCorrectionError[0] = servoAngIntegralError;
}

void storeSSErrorFinal() {
  servoAngIntegralError = (float)(yaw-finalTurnDirection);
  for (int tauFinal = maxTau-2; tauFinal >= 0; tauFinal--)
  {
    servoAngCorrectionError[tauFinal] = servoAngCorrectionError[tauFinal+1];
  }
  servoAngCorrectionError[0] = servoAngIntegralError;
}

void sumSSError() {
  for (int i = 0; i<integralTau; i++)
  {
    integralSSError = integralSSError + servoAngCorrectionError[i];
  }
  servoAngIntegralCorrection = ((k_i)*(integralSSError))/(integralTau);
}

void calcDeltaYaw() {
  doGyroOnce();
  yaw_NMinus1 = yaw;
  doGyroOnce();
  yaw_N = yaw;
  deltaYaw = yaw_N - yaw_NMinus1;
  if (deltaYaw < -180.0)
  {
    deltaYaw = 360.0 - deltaYaw;
  }
}

void calcServoAngDerivativeCorrection() {
  servoAngDerivativeCorrection = k_d * deltaYaw;
}

void flashBeethovenNo5 () {
  digitalWrite(LED_PIN, HIGH);
  delay(250);
  digitalWrite(LED_PIN, LOW);
  delay(250);
  digitalWrite(LED_PIN, HIGH);
  delay(250);
  digitalWrite(LED_PIN, LOW);
  delay(2000);
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(500);
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(2000);
}

void calcDistanceTraveled() {
  if ((abs(distanceL-distanceR)) < 0.1*distance) {
     distance = 0.5* distanceL + 0.5* distanceR;
  }
  else {
    distance = max(distanceL, distanceR);
  }
  distanceTraveled = distance;
}

void calcVelocity() {
  String str1;
  float deltaTime;
  timeI = millis();
  calcDistanceTraveled();
  distanceI = distanceTraveled;
  for (int i = 0; i < adjTimeVelocity; i++)
  {
    str1 = " ";
    if (str1.substring(0) == " ")
    {
      str1 = "";
    }
    Serial.print(str1);
  }
  timeF = millis();
  calcDistanceTraveled();
  distanceF = distanceTraveled;
  deltaTime = (timeF-timeI)/1000.0;
  velocity = ((distanceF - distanceI)/deltaTime);
  Serial.println("\nvelocity: ");
  Serial.print(velocity);
  Serial.println("\n");
}

void setVelocityStart() {
  ESCSpeedCmd = midESCSpeed;
  if (abs(velocity-targetVelocityStart)>0.05)
  {
    ESCSpeedProportionalCorrection = v_p*(targetVelocityStart-velocity);
    ESCSpeedCmd = midESCSpeed + ESCSpeedProportionalCorrection + ESCSpeedIntegralCorrection + ESCSpeedDerivativeCorrection;
  }
  if (ESCSpeedCmd > maxESCSpeed) {
    ESCSpeedCmd = maxESCSpeed;
  }
  if (ESCSpeedCmd < 0.0) {
    ESCSpeedCmd = 0;
  }
  setESCSpeed((int)(ESCSpeedCmd));
}

void setVelocityMid() {
  ESCSpeedCmd = midESCSpeed;
  if (abs(velocity-targetVelocityMid)>0.05)
  {
    ESCSpeedProportionalCorrection = v_p*(targetVelocityMid-velocity);
    ESCSpeedCmd = midESCSpeed + ESCSpeedProportionalCorrection + ESCSpeedIntegralCorrection + ESCSpeedDerivativeCorrection;
  }
  if (ESCSpeedCmd > maxESCSpeed) {
    ESCSpeedCmd = maxESCSpeed;
  }
  if (ESCSpeedCmd < 0.0) {
    ESCSpeedCmd = 0;
  }
  setESCSpeed((int)(ESCSpeedCmd));
}

void setVelocityFinal() {
  ESCSpeedCmd = midESCSpeed;
  if (abs(velocity-targetVelocityFinal)>0.01)
  {
    ESCSpeedProportionalCorrection = v_p*(targetVelocityFinal-velocity);
    ESCSpeedCmd = midESCSpeed + ESCSpeedProportionalCorrection + ESCSpeedIntegralCorrection + ESCSpeedDerivativeCorrection;
  }
  if (ESCSpeedCmd > maxESCSpeed) {
    ESCSpeedCmd = maxESCSpeed;
  }
  if (ESCSpeedCmd < 0.0) {
    ESCSpeedCmd = 0;
  }
  setESCSpeed((int)(ESCSpeedCmd));
}

//Velocity Integral Control

void storeSSVelocityErrorStart() {
  ESCSpeedIntegralError = targetVelocityStart-velocity;
  for (int tauVelocityStart = maxTauVelocity-2; tauVelocityStart >= 0; tauVelocityStart--)
  {
    ESCSpeedCorrectionError[tauVelocityStart] = ESCSpeedCorrectionError[tauVelocityStart+1];
  }
  ESCSpeedCorrectionError[0] = ESCSpeedIntegralError;
}

void storeSSVelocityErrorMid() {
  ESCSpeedIntegralError = targetVelocityMid-velocity;
  for (int tauVelocityMid = maxTauVelocity-2; tauVelocityMid >= 0; tauVelocityMid--)
  {
    ESCSpeedCorrectionError[tauVelocityMid] = ESCSpeedCorrectionError[tauVelocityMid+1];
  }
  ESCSpeedCorrectionError[0] = ESCSpeedIntegralError;
}

void storeSSVelocityErrorFinal() {
  ESCSpeedIntegralError = targetVelocityFinal-velocity;
  for (int tauVelocityFinal = maxTauVelocity-2; tauVelocityFinal >= 0; tauVelocityFinal--)
  {
    ESCSpeedCorrectionError[tauVelocityFinal] = ESCSpeedCorrectionError[tauVelocityFinal+1];
  }
  ESCSpeedCorrectionError[0] = ESCSpeedIntegralError;
}

void sumSSVelocityError() {
  for (int i = 0; i<integralTauVelocity; i++)
  {
    integralSSVelocityError = integralSSVelocityError + ESCSpeedCorrectionError[i];
  }
  ESCSpeedIntegralCorrection = ((v_i)*(integralSSVelocityError))/(integralTauVelocity);
}

//calc Velocity Derivative term

void calcDeltaVelocity() {
  calcVelocity();
  velocity_NMinus1 = velocity;
  calcVelocity();
  velocity_N = velocity;
  deltaVelocity = velocity_N - velocity_NMinus1;
}

void calcESCSpeedDerivativeCorrection() {
  ESCSpeedDerivativeCorrection = v_d * deltaYaw;
}
