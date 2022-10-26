
//Learn more about some functions pertaning to stuff I commented on.

//Copyright notice: this code is adapted from https://github.com/jrowberg/i2cdevlib
/*I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.*/

//Include SD, SPI, I2C, MPU6050 libraries.
#include <SD.h>
#include <SPI.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
#include <Servo.h>
#include <math.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif 

// class default I2C adress is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69

//Create MPU6050 object
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high int pinCS = 10; //CS pin for SD card 

//Create file object for future reference.
File myFile; // class default I2C address is 0x68

//Define timer variables used in Loop to wait for program to begin after calibration/until program has had reasonable time to run.
long int timer;
bool timerBool = true;
long int timeNow;

//Set up SD card
const int chipSelect = BUILTIN_SDCARD;

//Define LED pin on Arduino, and set the blinkState to false
#define LED_PIN 13
bool blinkState = false;

//Define servo objects
Servo rollServo;
Servo pitchServo;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int16_t gx, gy, gz;

//mpu data -> torque degrees
//WAIT: REPLACE ALL OF THE COMMENTS LATER AS WE HAVE MORE DEFINED STUFF LIKE PID AND WHATNOT

//Define your Moment of Inertia. We're using g*cm^2, due to the rather small size of our rocket.
static float momentInertia = 1260.1847; //not accurate yet

//PID vars
  //Angular Acceleration. We're measuring in deg/s^2.
float rAcceleration = 15;
float pAcceleration = 15;
static float Proportional = 1;
static float Derivative = 0.4;
static float Integral = 0.1;
float data[3][20]; 
int arrayCounter = 0;
float pSum = 0;
float rSum = 0;
static float loopTime;
//float pOmega = 0;
//float rOmega = 0;

//THIS IS DUMB WE NEED A BETTER WAY TO DEFINE THE 2D ARRAY OF THE F15thrust CURVE
//Force of the Engine, measured in g*cm/s^2 (for unit continuity with the Moment of Inertia).
float F15thrust;

//Define your Radius of your "system". The Radius is defined as the distance from the Centers of Thrust/Gravity. We're measuring in cm.
static float torqueRadius = 6;

//torque degrees -> servo degrees vars
float pTorque;
float pz = 4.452;
float pr1 = 3.2;
float pr2 = 1.445;
float pd = 4.2;
float pI;
float pJ;
float pA;
float pB;
static float pOffset = -52.02;
float rTorque;
float rz = 5.443;
float rr1 = 3.7;
float rr2 = 1.445;
float rd = 3.3;
float rI;
float rJ;
float rA;
float rB;
static float rOffset = 44.05;


// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

//Interupt detection
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

//Setup
void setup() {
  rollServo.attach(6);
  pitchServo.attach(9);

    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif 
    
    // initialize serial communication
    // serial value can be up to you depending on project
    Serial.begin(115200);  
     while (!Serial); // wait for Leonardo enumeration, others continue immediately
      
    // Tests whether the SD card is connected
    Serial.println("Initializing SD card...");
    Serial.println(SD.begin(chipSelect) ? F("SD card is ready to use.") : F("SD card initialization failed")); //If SD is available, print "SD card is ready to use."; if it's not, print "SD card initialization failed"   
    
    // initialize accel/gyro
    Serial.println("Initializing accelerometer...");
    mpu.initialize();
    mpu.setSleepEnabled(false);    // verify connection
    Serial.println("Testing accelerometer connection...");
    Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");   //If MPU is connected, print "MPU6050 connection successful"; if it's not, print "MPU6050 connection failed"   

    // Doesn't start loop until a character is sent
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer          //Work on blink state to find where problem is? Understand while statement better, and perhaps put the first while loop above the println
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
   
    //Load DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(81); //Supply your own offsets. We used calibration code from Luis Rodenas, supplied in this article: https://mjwhite8119.github.io/Robots/mpu6050
    mpu.setYGyroOffset(-8);
    mpu.setZGyroOffset(-28);
    mpu.setZAccelOffset(0);

     if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 2)..."));
        attachInterrupt(2, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    pinMode(LED_PIN, OUTPUT);
    
//Timing variables
  TCCR1A = 0;
  TCCR1B |= (1 << CS12) | (1 << CS10);
  TCNT1 = 0;
  unsigned long cycles = TCNT1;
  long microsecs = (float)((cycles-1)*1024/600);
//This code measures the number of microseconds it takes to run a single loop: put this code at the beginning of setup(),
//then put the loop code in setup().  
//Place this at the end of setup().
Serial.print(microsecs);

}

void loop() {
  timer=millis();

  // if programming failed, don't try to do anything
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
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

    //timerBool is set to false at the exact point the MPU starts sending data. This sets timeNow equal to the number of milliseconds that have passed at that time.
    if(timerBool) {
      timeNow = millis();
      timerBool = false;
    }

    //Then, data is collected for 10 seconds after timeNow is set.
    if(timer<=(timeNow+10000)) {
            // display yaw/pitch/roll in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
            //display raw angular velocities for pitch and roll in degrees per second
            mpu.getRotation(&gx, &gy, &gz);
            Serial.print("roll\t");
            Serial.print(gx/131); //dividing by 131 scales the raw output (between -32768 and 32768) down to degrees per second (between -250 and 250)
            Serial.print("pitch\t");
            Serial.print(gy/131);
      // write the accelerometer values to SD card
      myFile = SD.open("GYRO.txt", FILE_WRITE);
        myFile.print("acceleration:\t");
        myFile.print(ypr[0] * 180/M_PI); myFile.print("\t");
        myFile.print(ypr[1] * 180/M_PI); myFile.print("\t");
        myFile.print(ypr[2] * 180/M_PI); myFile.print("\t"); 
        myFile.println(timer);                                
        myFile.close();                                       
      // blink LED to indicate activity
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
      //PID!
      //storing the gyro values in a 2D array. After 20 measurements, the measurements will start overwriting previous ones, because of the mod (%) operator.
      //For example, the 22nd measurement (when arrayCounter = 21) will go into the array row numbered 1, which is the second row.
      data[arrayCounter % 20][0] = ypr[0];
      data[arrayCounter % 20][1] = ypr[1];
      data[arrayCounter % 20][2] = ypr[2];
      arrayCounter = arrayCounter+1;

      //taking the sum of all pitch and roll values
      pSum = pSum + ypr[1];
      rSum = rSum + ypr[2];

      //PID magic. The proportional gain (first term) is proportional to the rocket's angle.
      //The derivative gain (second term) is proportional to the rocket's angular velocity.
      //The integral gain is proportional to the integral of the rocket's angle, which can be calculated by 
      //taking the average measurement and multiplying it by the time. After some work, the formula
      //pSum*loopTime is found.
      pAcceleration = (Proportional*ypr[1])+(Derivative*gx/131.0)+(Integral*pSum*loopTime);
      rAcceleration = (Proportional*ypr[2])+(Derivative*gy/131.0)+(Integral*rSum*loopTime);
      //An alternate solution if the gx, gy calculations don't work
      /* if (arrayCounter = 0) { 
       pOmega = 0;
       rOmega = 0;
       } else if (arrayCounter < 3) {
       pOmega = ypr[1]/(arrayCounter*loopTime);
       rOmega = ypr[2]/(arrayCounter*loopTime);
       } else {
       pOmega = (ypr[1]-data[(arrayCounter-3) % 20][1])/(3*loopTime);
       rOmega = (ypr[2]-data[(arrayCounter-3) % 20][2])/(3*loopTime);
       }
       pAcceleration = (Proportional*ypr[1])+(Derivative*pOmega)+(Integral*pSum*loopTime);
       rAcceleration = (Proportional*ypr[2])+(Derivative*rOmega)+(Integral*rSum*loopTime);
       */

      //Torque Angle (Dynamic variable): Defined as the angle from the centerline, needed to induce a correcting torque given a specific (dynamic) angular velocity (measured by MPU).
      //Torque Angle Calculation
      pTorque = (180/M_PI)*(asin((momentInertia*pAcceleration)/(F15thrust*torqueRadius)));
      rTorque = (180/M_PI)*(asin((momentInertia*rAcceleration)/(F15thrust*torqueRadius)));


      //These are silly equations that convert a torque angle into a servo angle.
      pI = pr1*cos((pTorque+pOffset)*M_PI/180);
      pJ = pr2*sin((pTorque+pOffset)*M_PI/180);
      pA = 0.5+(((pd*pd)-(pr2*pr2))/(2*(((pz-pI)*(pz-pI))+(pJ*pJ))));
      pB = sqrt(-0.25+(((pd*pd)+(pr2*pr2))/(2*(((pz-pI)*(pz-pI))+(pJ*pJ))))-((0.5-pA)*(0.5-pA)));
      pitchServo.write((180/M_PI)*atan2(((1-pA)*pJ)-(pB*(pz-pI)),(pI+(pA*(pz-pI))-(pB*pJ)-pz)));

      rI = rr1*cos((rTorque+rOffset)*M_PI/180);
      rJ = rr2*sin((rTorque+rOffset)*M_PI/180);
      rA = 0.5+(((rd*rd)-(rr2*rr2))/(2*(((rz-rI)*(rz-rI))+(rJ*rJ))));
      rB = sqrt(-0.25+(((rd*rd)+(rr2*rr2))/(2*(((rz-rI)*(rz-rI))+(rJ*rJ))))-((0.5-rA)*(0.5-rA)));
      rollServo.write((180/M_PI)*atan2(((1-rA)*rJ)+(rB*(rz-rI)),(rI+(rA*(rz-rI))+(rB*rJ)-rz)));
    }
  }
}

//Vertical: rollServo = 32, pitchServo = 97
