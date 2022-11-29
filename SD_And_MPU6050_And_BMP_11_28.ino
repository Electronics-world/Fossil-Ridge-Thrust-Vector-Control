/*Fossil Ridge TVC Rocket Sketch
 *11/17/2022 by Jackson Dryg and Quentin Perez-Wahl
 *Updates avalible at https://github.com/QuentinPerezWahl/Open-Model-Rocket-Thrust-Vector-Control
 *
 *Using the BMP280 DEV library, Servos, math, SD, and SPI libraries.
 *Also heavily using the MPU DEV library using the I2C device class from Jeff Rowberg
 *Copyright notice: this code is adapted from https://github.com/jrowberg/i2cdevlib
=================================================================================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
=================================================================================================
*/


// =================================================================
// ===                    LIBRARY DEFINITIONS                    ===
// =================================================================

//Include all libraries
#include "I2Cdev.h"
#include <MPU6050_6Axis_MotionApps20.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif 
#include <BMP280_DEV.h>
#include <helper_3dmath.h>
#include <math.h>
#include <SD.h>
#include <Servo.h>
#include <SPI.h>


// =================================================================
// ===             VARIABLE AND OBJECT DEFINITIONS               ===
// =================================================================

// class default I2C adress is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69

//Create MPU6050 object
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */

///////////////////////////////////////DO WE NEED THE DEFINE OUTPUT STUFF HERE??????????????????????

//Define the interupt pin being used
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

//Define LED pin on the board, and set the blinkState to false
#define LED_PIN 13
bool blinkState = false;

//MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

//MPU orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int16_t omega[3]; //adjusted angular velocity data

//Create BMP280 object
BMP280_DEV bmp280;    

//BMP280 variables  
float temperature, pressure, altitude;
int BMPcounter = 0;
float BMPsum = 0;
float avgPressure;
float maxAltitude;

//Create servo objects
Servo rollServo;
Servo pitchServo;
Servo parachuteServo;

//Set up SD card (BUILTIN SELECTED FOR TEENSYS)
const int chipSelect = BUILTIN_SDCARD;

//Create file object for SD writing
File myFile; // class default I2C address is 0x68

//"Flight state" string and related variables
String state = "prelaunch";
int launchDelay = 8;
int apogeeDelay = 8;

//Timer variables
double mainTimer;
float launchTime;
float thrustTime;

//Moment of Inertia; g*cm^2
static float momentInertia = 1260.1847; //not accurate yet

//Thrust; g*cm/s^2 (for unit continuity with the Moment of Inertia)
float F15thrust;
float thrustTime;
float thrustX[26] = {0.148, 0.228, 0.294, 0.353, 0.382, 0.419, 0.477, 0.520, 0.593, 0.688, 0.855, 1.037, 1.205, 1.423, 1.452, 1.503, 1.736, 1.955, 2.210, 2.494, 2.763, 3.120, 3.382, 3.404, 3.418, 3.450};
float thrustY[26] = {7.638, 12.253, 16.391, 20.210, 22.756, 25.260, 23.074, 20.845, 19.093, 17.500, 16.225, 15.427, 14.948, 14.627, 15.741, 14.785, 14.623, 14.303, 14.141, 13.819, 13.338, 13.334, 13.013, 9.352, 4.895, 0.000};

//Moment arm; cm
static float momentArm = 6;

//PID vars
//Angular Acceleration; deg/s^2
float rAcceleration = 15;
float pAcceleration = 15;
static float Proportional = 1; //depends on
static float Derivative = 0.4; //how we tune
static float Integral = 0.1; //our PID control
float data[3][20]; 
int arrayCounter = 0;
float pSum = 0;
float rSum = 0;
static float loopTime;
//float pOmega = 0;
//float rOmega = 0;

//Torque to servo angle variables
float pTorqueAngle;
float pz = 4.452;
float pr1 = 3.2;
float pr2 = 1.445;
float pd = 4.2;
float pI;
float pJ;
float pA;
float pB;
static float pOffset = -52.02;
float rTorqueAngle;
float rz = 5.443;
float rr1 = 3.7;
float rr2 = 1.445;
float rd = 3.3;
float rI;
float rJ;
float rA;
float rB;
static float rOffset = 44.05;


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     //Indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                 SENSOR READING FUNCTIONS                 ===
// ================================================================

float read_MPU_roll () {
   if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
   mpu.dmpGetQuaternion(&q, fifoBuffer);
   mpu.dmpGetGravity(&gravity, &q);
   mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
   Serial.print("roll\t");
   Serial.println(ypr[2] * 180/M_PI);
   myFile = SD.open("Data.txt", FILE_WRITE);
   myFile.print("roll\t");
   myFile.println(ypr[2] * 180/M_PI); 
   myFile.close();
   ypr[2] = ypr[2] * 180/M_PI;
  return ypr[2];     
   }      
}

float read_MPU_pitch () {
   if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
   mpu.dmpGetQuaternion(&q, fifoBuffer);
   mpu.dmpGetGravity(&gravity, &q);
   mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
   Serial.print("pitch\t");
   Serial.println(ypr[1] * 180/M_PI);
   myFile = SD.open("Data.txt", FILE_WRITE);
   myFile.print("pitch\t");
   myFile.println(ypr[1] * 180/M_PI); 
   myFile.close();
  return ypr[1];     
   }      
}

int16_t read_MPU_accel () {
   if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
   mpu.dmpGetQuaternion(&q, fifoBuffer);
   mpu.dmpGetAccel(&aa, fifoBuffer);
   mpu.dmpGetGravity(&gravity, &q);
   mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
   Serial.print("accel\t");
   Serial.println(aaReal.z);
   myFile = SD.open("Data.txt", FILE_WRITE);
   myFile.print("accel\t");
   myFile.println(aaReal.z/16384.0); 
   myFile.close();
   aaReal.z = aaReal.z/16384.0;
   return aaReal.z;
   }
}

float read_MPU_rOmega () {
   mpu.getRotation(&omega[0], &omega[1], &omega[2]);
   Serial.print("rollOmega\t");
   Serial.print(omega[2]/131.0); //dividing by 131 scales the raw output (between -32768 and 32768) down to degrees per second (between -250 and 250)
   myFile = SD.open("Data.txt", FILE_WRITE);
   myFile.print("rollOmega\t");
   myFile.println(omega[2]/131.0); 
   myFile.close();
  omega[2] = omega[2]/131.0;
  return omega[2];
}

float read_MPU_pOmega () {
   mpu.getRotation(&omega[0], &omega[1], &omega[2]);
   Serial.print("pitch\t");
   Serial.print(omega[1]/131.0); //dividing by 131 scales the raw output (between -32768 and 32768) down to degrees per second (between -250 and 250)
   myFile = SD.open("Data.txt", FILE_WRITE);
   myFile.print("pitchOmega\t");
   myFile.println(omega[1]/131.0); 
   myFile.close();
  omega[1] = omega[1]/131.0;
  return omega[1];
}

float read_BMP_altitude () {
  if (bmp280.getMeasurements(temperature, pressure, altitude)) { // Check if the measurement is complete
    Serial.print("altitude\t");
    Serial.print(altitude);
    myFile = SD.open("Data.txt", FILE_WRITE);
    myFile.print("altitude\t");
    myFile.println(altitude); 
    myFile.close();
    return altitude;
  }
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    //Join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        //HAVE WE TESTED THIS HERE??????
        //Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif 
    
    //Initialize serial communication
    // serial value can be up to you depending on project
    Serial.begin(115200);  
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    
    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.      
    
    //Test SD card connection
    Serial.println("Initializing SD card...");
    Serial.println(SD.begin(chipSelect) ? F("SD card is ready to use.") : F("SD card initialization failed")); //If SD is available, print "SD card is ready to use."; if it's not, print "SD card initialization failed"   
    
    //Initialize MPU6050
    Serial.println("Initializing MPU6050...");
    mpu.initialize();
    pinMode(2, 0);
  
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    //Load DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    //We will eventually put the MPU calibration code here.
    mpu.setXgyroOffset(81); //Supply your own offsets. We used calibration code from Luis Rodenas, supplied in this article: https://mjwhite8119.github.io/Robots/mpu6050
    mpu.setYgyroOffset(-8);
    mpu.setZgyroOffset(-28);
    mpu.setZAccelOffset(0);

    // make sure code worked (returns 0 if so)
    if (devStatus == 0) {
      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.PrintActiveOffsets();
      //!!!-- we would like a way to get offsets during the prelaunch phase, not printed to Serial
      int16_t * GetActiveOffsets();
      
      //Turn on DMP
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      //Enable Arduino interrupt detection
      Serial.println(F("Enabling interrupt detection (Arduino external interrupt 2)..."));
      Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
      Serial.println(F(")..."));
      attachInterrupt(2, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      //Set DMP Ready flag so the main loop() function knows it's usable
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;

      //Get expected DMP packet size for later comparison
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

  //Attach servos
  rollServo.attach(6);
  pitchServo.attach(9);
  parachuteServo.attach(8);

  //BMP280 setup
  bmp280.begin();                                 // Default initialisation, place the BMP280 into SLEEP_MODE 
  //bmp280.setPresOversampling(OVERSAMPLING_X4);    // Set the pressure oversampling to X4
  //bmp280.setTempOversampling(OVERSAMPLING_X1);    // Set the temperature oversampling to X1
  //bmp280.setIIRFilter(IIR_FILTER_4);              // Set the IIR filter to setting 4
  bmp280.setTimeStandby(TIME_STANDBY_05MS);     // Set the standby time to be very short
  bmp280.startNormalConversion();                 // Start BMP280 continuous conversion in NORMAL_MODE  
  
  //Finding the pressure at sea level
  for (int j = 0; j < 20; j++) {
    if (bmp280.getMeasurements(temperature, pressure, altitude)) { //Check if the measurement is complete
        Serial.print(altitude);
        Serial.print(F("m"));  
        BMPsum += pressure;
        BMPcounter += 1;
        avgPressure = BMPsum/BMPcounter;
        Serial.print(avgPressure);
        Serial.println(F("hPa"));
      }
  }
  bmp280.setSeaLevelPressure(avgPressure);
    
  //Timing vars
    TCCR1A = 0;
    TCCR1B |= (1 << CS12) | (1 << CS10);
    TCNT1 = 0;
    unsigned long cycles = TCNT1;
    long microsecs = (float)((cycles-1)*1024/600);
  //This code measures the number of microseconds it takes to run a single loop: put this code at the beginning of setup(),
  //then put the loop code in setup().  

  //Place this at the end of setup().
  Serial.print(microsecs);

  //!!!-- the loopTime variable will be set to the length of the program loop in seconds

  // configure LED for output
  pinMode(13, 1);
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  mainTimer = micros()/1000000;
  
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
 
  if (state = "prelaunch") {
    //get z acceleration, measured in G's
    aaReal.z = read_MPU_accel();
    
    //get altitude
    altitude = read_BMP_altitude();

    //If the rocket is at least 1 m off the ground and experiencing an acceleration of 1 g upward
    //(ignoring gravity)
    if (altitude > 1 && altitude < 500 && aaReal.z > 1) {
      launchDelay -= 1; 
      if (launchDelay < 0) {
      state = "launch";
      launchTime = mainTimer;
      }
    } else {
      launchDelay = 8;
    }
  } else if (state = "launch") {
   //read MPU for roll and pitch
    ypr[2] = read_MPU_roll();
    ypr[1] = read_MPU_pitch();

    //read MPU for z acceleration
    aaReal.z = read_MPU_accel();

    //read MPU for angular velocity in roll and pitch directions
    omega[2] = read_MPU_rOmega();
    omega[1] = read_MPU_pOmega();

    //read BMP for altitude
    altitude = read_BMP_altitude();

    //compute thrust
    thrustTime = mainTimer - launchTime + 0.1855;
    for (int i = 0; i < 25; i++) {
      if (thrustTime >= thrustX[i] && thrustTime < thrustX[i+1]) {
        F15thrust = 100000*(thrustX[i]+((thrustTime-thrustX[i])*(thrustY[i+1]-thrustY[i])/(thrustX[i+1]-thrustX[i])));
      }
    }
    
      //PID!
      //taking the sum of all pitch and roll values
      pSum = pSum + ypr[1];
      rSum = rSum + ypr[2];

      //PID magic. The proportional gain (first term) is proportional to the rocket's angle.
      //The derivative gain (second term) is proportional to the rocket's angular velocity.
      //The integral gain is proportional to the integral of the rocket's angle, which can be calculated by 
      //taking the average measurement and multiplying it by the time. After some work, the formula
      //pSum*loopTime is found.
      pAcceleration = (Proportional*ypr[1])+(Derivative*omega[1])+(Integral*pSum*loopTime);
      rAcceleration = (Proportional*ypr[2])+(Derivative*omega[2])+(Integral*rSum*loopTime);

      //Torque Angle (Dynamic variable): Defined as the angle from the centerline, needed to induce a correcting torque given a specific (dynamic) angular velocity (measured by MPU).
      //Torque Angle Calculation
      pTorqueAngle = (180/M_PI)*(asin((momentInertia*pAcceleration)/(F15thrust*momentArm)));
      rTorqueAngle = (180/M_PI)*(asin((momentInertia*rAcceleration)/(F15thrust*momentArm)));

      //These are silly equations that convert a torque angle into a servo angle.
      pI = pr1*cos((pTorqueAngle+pOffset)*M_PI/180);
      pJ = pr2*sin((pTorqueAngle+pOffset)*M_PI/180);
      pA = 0.5+(((pd*pd)-(pr2*pr2))/(2*(((pz-pI)*(pz-pI))+(pJ*pJ))));
      pB = sqrt(-0.25+(((pd*pd)+(pr2*pr2))/(2*(((pz-pI)*(pz-pI))+(pJ*pJ))))-((0.5-pA)*(0.5-pA)));
      pitchServo.write((180/M_PI)*atan2(((1-pA)*pJ)-(pB*(pz-pI)),(pI+(pA*(pz-pI))-(pB*pJ)-pz)));

      rI = rr1*cos((rTorqueAngle+rOffset)*M_PI/180);
      rJ = rr2*sin((rTorqueAngle+rOffset)*M_PI/180);
      rA = 0.5+(((rd*rd)-(rr2*rr2))/(2*(((rz-rI)*(rz-rI))+(rJ*rJ))));
      rB = sqrt(-0.25+(((rd*rd)+(rr2*rr2))/(2*(((rz-rI)*(rz-rI))+(rJ*rJ))))-((0.5-rA)*(0.5-rA)));
      rollServo.write((180/M_PI)*atan2(((1-rA)*rJ)-(rB*(rz-rI)),(rI+(rA*(rz-rI))-(rB*rJ)-rz)));

      
    maxAltitude = max(altitude, maxAltitude);
    if (altitude < maxAltitude) {
      apogeeDelay -= 1;
      if (apogeeDelay < 0) {
        state = "apogee";
    }
    } else {
      apogeeDelay = 8;
    }
 } else if (state = "apogee") {
    parachuteServo.write(165); //change if needed
  }                                
      // blink LED to indicate activity
      blinkState = !blinkState;
      digitalWrite(13, blinkState);
}
