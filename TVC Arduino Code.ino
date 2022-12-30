/* Open-Source Thrust Vectored Control (TVC) Rocket Arduino Sketch
 * 29/12/2022 by Quentin Perez-Wahl and Jackson Dryg; message @ <quentin.perez.wahl@gmail.com>
 * 
 *
 *
 *
 * !!!!!!!!!!      IMPORTANT      !!!!!!!!!!      !!!!!!!!!!      IMPORTANT      !!!!!!!!!!      !!!!!!!!!!      IMPORTANT      !!!!!!!!!!
 * CHECK THE GITHUB WIKI (https://github.com/QuentinPerezWahl/Open-Model-Rocket-Thrust-Vector-Control/wiki)                     !!!!!!!!!!
 * IF THERE IS SOMETHING YOU DON'T UNDERSTAND IN THE CODE OR OTHERWISE, CHECK THE ABOVE WIKI PAGE                               !!!!!!!!!!
 * !!!!!!!!!!      IMPORTANT      !!!!!!!!!!      !!!!!!!!!!      IMPORTANT      !!!!!!!!!!      !!!!!!!!!!      IMPORTANT      !!!!!!!!!!
 *
 *
 *
 * 
 * Updates avalible at https://github.com/QuentinPerezWahl/Open-Model-Rocket-Thrust-Vector-Control
 * Version 2.0
 *
 * Using the MPU6050 DEV, BMP280 DEV, Servo, Math, SD, and SPI libraries.
 * Copyright notice: this code is adapted from https://github.com/jrowberg/i2cdevlib
 * Copyright notice: this code is adapted from https://github.com/MartinL1/BMP280_DEV
 *
 *
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
=================================================================================================
MartinL1/BMP280_DEV is licensed under the MIT License

Copyright (c) 2019 MartinL1

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
=================================================================================================
*/


// ======================================================================
// ===                       LIBRARY DEFINITIONS                      ===
// ======================================================================

// Both the MPU6050dev and I2Cdev libraries can be downloaded from https://github.com/jrowberg/i2cdevlib.
#include "I2Cdev.h"
#include <MPU6050_6Axis_MotionApps20.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif 
#include <helper_3dmath.h>

// The BMP280dev library can be downloaded from https://github.com/MartinL1/BMP280_DEV.
#include <BMP280_DEV.h>

#include <math.h>
#include <SD.h>
#include <Servo.h>
#include <SPI.h>


// ======================================================================
// ===                 VARIABLE AND OBJECT DEFINITIONS                ===
// ======================================================================

// The class default I2C adress for the MPU6050 is 0x68. Specific I2C addresses may be passed as a parameter here:
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69

// Create MPU6050 object
MPU6050 mpu;
// MPU6050 mpu(0x69); // Use this line instead if you are using AD0 high

// ======================================================================
// NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
// depends on the MPU-6050's INT pin being connected to the Arduino's
// external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
// digital I/O pin 2.
// ======================================================================

// Define the interupt pin being used for the MPU6050
#define INTERRUPT_PIN 2  // Use pin 2 on Arduino Uno & most other boards

// MPU control and status variables
bool dmpReady = false;  // Set true if DMP initialization was successful
uint8_t mpuIntStatus;   // Holds the actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // Count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// MPU orientation and motion variables
Quaternion q;           // [w, x, y, z]  Quaternion container
VectorInt16 aa;         // [x, y, z]     Accelerometer sensor measurements
VectorInt16 aaReal;     // [x, y, z]     Gravity-free accelerometer sensor measurements
float verticalAcceleration;         // Vertical acceleration measured in g's, with scaled sensitivity
VectorInt16 aaWorld;    // [x, y, z]     World-frame acceleration sensor measurements
VectorFloat gravity;    // [x, y, z]     Gravity vector
float euler[3];         // [ψ, θ, φ]     Euler angle container
float ypr[3];           // [y, p, r]     Yaw/pitch/roll container and gravity vector 
                        // Measured in radians; converted in readMPU function to degrees
int16_t omega[3];       // [ẏ, ṗ, ṙ]     Angular velocity data (degrees per second)

  //LSB scale variables. See line 1822 of MPU6050.cpp for the acceleration sample scale.
  //See the following link about LSB: https://www.edaboard.com/threads/accelerometer-questions.250221/
  //Set LSB values to your desired sensitivity range, as defined in the MPU6050.cpp file.
    static float accelerationLSB = 8192;    // Sensitivity-scale value for acceleration measurements
                                            // See line 1822 of MPU6050.cpp
    static float angularLSB = 131;          // Sensitivity-scale value for rotation measurements
                                            // See line 1900 of MPU6050.cpp


// Create BMP280 object
BMP280_DEV bmp;

// BMP280 variables
float pressure;                       // Pressure measured in hPa/millibars
float altitude;                       // Altitude measured in meters
float avgPressure;                    // Average pressure calibration variable
static float pressureCalcTime = 20;   // "Time" for average pressure calculation
float maxAltitude;                    // Maximum altitude measured


// Create servo objects (parachute servo not needed if you are using another recovery system)
Servo rollServo;
Servo pitchServo;
Servo parachuteServo;

// Define servo pins being used (you can change according to your servo pins)
#define ROLL_SERVO_PIN 6
#define PITCH_SERVO_PIN 9
#define PARACHUTE_SERVO_PIN 8


// Supply SD sheild/module value to connect to SD card (we are using the TEENSY 4.1's value)
const int chipSelect = BUILTIN_SDCARD;

// Create text file that will go onto the SD card (you can name it whatever you want)
File flightData;


// Flight state variables
String flightState = "prelaunch";   // Flight state - either prelaunch, launch, or apogee
int launchDelay = 10;               // Sensitivity for computer to register launch
int apogeeDelay = 10;               // Sensitivity for computer to register apogee
int landingDelay = 100;               // Sensitivity for computer to register landing

// Timer variable
double mainTimer;   // Total runtime (measured in seconds)

// PID variables (supply your own gains)
static float Kp = 10;     // Proportional gain
static float Ki = 0.2;    // Integral gain
static float Kd = 2;      // Derivative gain
float pSum;               // Integral of pitch values
float rSum;               // Integral of roll values
float pUT;                // Pitch control variable - u(t) of pitch values
float rUT;                // Pitch control variable - u(t) of roll values

// Rocket motor angle to servo angle calculation variables (supply your own values).
// This is the hardest code to understand. Reminder to look at the wiki, linked on line 8.
float pz = 3.821;                 
float pr1 = 3.2;                  
float pr2 = 1.445;                
float pd = 2.15;                  
float pI;                         
float pJ;                         
float pA;                         
float pB;                         
static float pOffset = -41.26;    
float pValue;                     // Pitch servo angle
float rz = 3.855;                 
float rr1 = 3.421;                
float rr2 = 1.445;                
float rd = 1.75;                  
float rI;                         
float rJ;                         
float rA;                         
float rB;                         
static float rOffset = 39.8;      
float rValue;                     // Roll servo angle


// Define the LED pin on the board, and set the blinkState to false (most boards' LED is on pin 13)
#define LED_PIN 13

bool blinkState = false;


// ======================================================================
// ===                      USER DEFINED FUNCTIONS                    ===
// ======================================================================

void mpuSetup() {
    // Join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment out this line if you are having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif 
    
    // Initialize serial communication; the serial value can be up to you depending on your project
    Serial.begin(9600);  
    while (!Serial); // Wait for Leonardo enumeration, others continue immediately
    
    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.      
        
    // Initialize the MPU6050
    Serial.println("Initializing MPU6050...");
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
  
    // Verify its connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // Load DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    //Gyro and accelerometer offsets
    mpu.setXGyroOffset(90);
    mpu.setYGyroOffset(-8);
    mpu.setZGyroOffset(11);
    mpu.setZAccelOffset(894);

    // Make sure MPU6050 code worked (returns 0 if so)
    if (devStatus == 0) {
      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.PrintActiveOffsets();
      
      // Turn on DMP
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      // Enable Arduino interrupt detection
      Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
      Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
      Serial.println(F(")..."));
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // Set DMP Ready flag so the main loop() function knows it's usable
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;

      // Get expected DMP packet size for later comparison
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
}


void bmpSetup() {
  //Start BMP280
  bmp.begin();                                    // Default initialisation, place the BMP280 into SLEEP_MODE 
  //bmp280.setPresOversampling(OVERSAMPLING_X4);  // Set the pressure oversampling to X4
  //bmp280.setTempOversampling(OVERSAMPLING_X1);  // Set the temperature oversampling to X1
  //bmp280.setIIRFilter(IIR_FILTER_4);            // Set the IIR filter to setting 4
  bmp.setTimeStandby(TIME_STANDBY_05MS);          // Set the standby (non-funtioning) time to be very short
  bmp.startNormalConversion();                    // Start BMP280 continuous conversion in NORMAL_MODE 

  // Find the pressure at "sea level" (pre-launch altitude)
  for (int i = 0; i < pressureCalcTime; i++) {
    bmp.getPressure(pressure);
    avgPressure += pressure;
  }
  avgPressure /= pressureCalcTime;

  //Set pressure to sea level (so current altitude should be ~0)
  bmp.setSeaLevelPressure(avgPressure);
}


void readMPU() {
  // Read the latest packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    // Read MPU's acceleration, angular, and angular velocity data
      // Acceleration measurements
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);                  
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

      // Scale vertical acceleration by LSB sensitivity
      verticalAcceleration = aaReal.z / accelerationLSB;
      
      // Angular measurments
        // Angle (ypr) measurements
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        //Convert ypr measurments from radians to degrees
        ypr[0] *= 180/M_PI;
        ypr[1] *= 180/M_PI;
        ypr[2] *= 180/M_PI;

        // Angular velocity (ẏṗṙ) measurments
        mpu.getRotation(&omega[0], &omega[1], &omega[2]);

        // Scale angular velocity by LSB sensitivity
        omega[0] /= angularLSB;
        omega[1] /= angularLSB;
        omega[2] /= angularLSB;

    //Print necessary values to the serial monitor
    Serial.print("accel\t");
    Serial.println(verticalAcceleration);
    Serial.print("pitch\t");
    Serial.println(ypr[1]);
    Serial.print("roll\t");
    Serial.println(ypr[2]);

    //Print necessary values to the SD card
    flightData.print("accel:\t");
    flightData.println(verticalAcceleration);
    flightData.print("pitch:\t");
    flightData.println(ypr[1]);
    flightData.print("roll:\t");
    flightData.println(ypr[2]);
  }
}


void computePID() {
  // Find the "integral" of the pitch and roll values, by taking the sum of all pitch and roll values.
  pSum += ypr[1];
  rSum += ypr[2];

  // Find the control angle (otherwise represented as u(t) in PID control) needed to correct the rocket.
  pUT = -(Kp*ypr[1]) - (Ki*pSum) - (Kd*omega[1]);
  rUT = -(Kp*ypr[2]) - (Ki*rSum) - (Kd*omega[2]);

  // Check if u(t) is too large for the gimbal on both axes
    if(pUT > 5) {
      pUT = 5;
    } else if (pUT < -5) {
      pUT = -5;
    }

    if(rUT > 5) {
      rUT = 5;
    } else if (rUT < -5) {
      rUT = -5;
    }
}


void torqueToServo() {
    // Pitch servo calculation
    pI = pr1*cos((pUT+pOffset)*M_PI/180); 
    pJ = pr1*sin((pUT+pOffset)*M_PI/180);
    pA = 0.5+(((pd*pd)-(pr2*pr2))/(2*(((pz-pI)*(pz-pI))+(pJ*pJ))));
    pB = sqrt(-0.25+(((pd*pd)+(pr2*pr2))/(2*(((pz-pI)*(pz-pI))+(pJ*pJ))))-((0.5-pA)*(0.5-pA)));

    // Pitch servo angle value
    pValue = (180/M_PI)*atan2(((1-pA)*pJ)-(pB*(pz-pI)),(pI+(pA*(pz-pI))-(pB*pJ)-pz))+182-pOffset;

    // Roll servo calculation
    rI = rr1*cos((rUT+rOffset)*M_PI/180);
    rJ = rr1*sin((rUT+rOffset)*M_PI/180);
    rA = 0.5+(((rd*rd)-(rr2*rr2))/(2*(((rz-rI)*(rz-rI))+(rJ*rJ))));
    rB = sqrt(-0.25+(((rd*rd)+(rr2*rr2))/(2*(((rz-rI)*(rz-rI))+(rJ*rJ))))-((0.5-rA)*(0.5-rA)));

    // Roll servo angle value
    rValue = (180/M_PI)*atan2(((1-rA)*rJ)+(rB*(rz-rI)),(rI+(rA*(rz-rI))+(rB*rJ)-rz))+35-rOffset;
}


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // Indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // MPU6050 setup
  mpuSetup();

  // Test the SD card connection
  Serial.println("Initializing SD card...");
  Serial.println(SD.begin(chipSelect) ? F("SD card is ready to use.") : F("SD card initialization failed"));

  // Attach servos to their respective pins
  rollServo.attach(ROLL_SERVO_PIN);
  pitchServo.attach(PITCH_SERVO_PIN);
  parachuteServo.attach(PARACHUTE_SERVO_PIN);

  // BMP280 setup
  bmpSetup();

  // Open the SD card's flight data file, and set it up write to the file
  flightData = SD.open("Flight Data.txt", FILE_WRITE);

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  // If programming failed, don't try to do anything
  if (!dmpReady) return;

  // Find the time that the computer has been running for (in seconds)
  mainTimer = micros()/1000000;


  //State detection 
  if (flightState == "prelaunch") {
    // Read data from the MPU and BMP
    readMPU();
    bmp.getAltitude(altitude);

    // Check to see if the rocket has launched
      // If the rocket's altitude is above 1 meter and its acceleration is above 1 g
      if (altitude > 0.1 && verticalAcceleration > 0.1) {
        launchDelay -= 1; 
      }

      // Check to see if the launch delay has hit 0, in which case the rocket has launched
      if (launchDelay == 0) {
        flightState = "launch";
        Serial.println("LAUNCHING");
        flightData.write("LAUNCHING");
      }
  } else if (flightState == "launch") {
    // Read data from the MPU and BMP
    readMPU();
    bmp.getAltitude(altitude);

    // Do PID and servo angle calculations
    computePID();
    torqueToServo();

    // Write the calculated servo angles to the servos themselves
    pitchServo.write(pValue);
    rollServo.write(rValue);
      
    // Check to see if the rocket is falling
      // Find the maximum altitude
      maxAltitude = max(altitude, maxAltitude);

      // If the rocket's altitude is lower than the maximum altitude
      if (altitude < maxAltitude) {
        apogeeDelay -= 1;
        Serial.println("APOGEE");
        flightData.write("APOGEE");
      }

      // Check to see if the apogee delay has hit 0, in which case the rocket is falling
      if (apogeeDelay == 0) {
        flightState = "apogee";
      }

  } else if (flightState == "apogee") {
    // Read data from the MPU and BMP
    readMPU();
    bmp.getAltitude(altitude);

    // Deploy parachute
    parachuteServo.write(165);

    // If the rocket's altitude is lower than 10 meters
    if (altitude <= 10) {
      apogeeDelay--;
      Serial.println("LANDING");
      flightData.write("LANDING");
    }

    // Check to see if the landing delay has hit 0, in which case the rocket has basically landed
    if (landingDelay == 0) {
      flightState = "landed";
      flightData.close();
    }
  }

  // Blink the LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(13, blinkState);
}