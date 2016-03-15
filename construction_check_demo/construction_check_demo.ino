/* 
********************************************** 
        Anthony Sensor Wiring Pinout
**********************************************

************* Ultrasonic Sensors *************
* All pins are Digital I/O for Ultrasonic
* 
* Front Face Left (Echo = 38 Trig = 39)
* Front Face Right (Echo = 40 Trig = 41)
* Left Face Front Left (Echo = 24 Trig = 25)
* Left Face Bottom Left (Echo = 22 Trig = 23)
* Right Face Front Right (Echo = 50 Trig = 51)
* Right Face Bottom Right (Echo = 52 Trig = 53)
* 
*************   MPU 6050 Wiring  *************
* SCL = Digital I/O (Communication) Pin 21
* SDA = Digital I/O (Communication) Pin 20
* INT = Digital I/O (PWM)           Pin 2
* 
* ********************************************
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Define Address for the Two Motor Sheilds
Adafruit_MotorShield AFMSbot(0x61); // Rightmost jumper closed
Adafruit_MotorShield AFMStop(0x60); // Default address, no jumpers

// Define Motors
Adafruit_StepperMotor *myStepperFR = AFMStop.getStepper(200, 1);  //Front Right
Adafruit_StepperMotor *myStepperBR= AFMStop.getStepper(200, 2);   // Bottom Right
Adafruit_StepperMotor *myStepperFL = AFMSbot.getStepper(200, 1);  //Front Left
Adafruit_StepperMotor *myStepperBL = AFMSbot.getStepper(200, 2);  //Bottom Left


void setup() {
  while (!Serial);
  Serial.begin(9600);   // set up Serial library at 9600 bps
  AFMSbot.begin();      // Start the bottom shield
  AFMStop.begin();      // Start the top shield
  TWBR = ((F_CPU /200000l) - 16) / 2; // Change the i2c clock to 200KHz so motor can run faster (400KHz is fastest it can go)
}

int i;
void loop() {
    /*
    delay(500);
    asyncGoForward(162,0);
    delay(100);
    asyncGoLeft(162,0);
    delay(100);
    asyncGoForward(162,0);
    delay(100);
    asyncGoLeft(162,0);
    delay(100);
    asyncGoForward(162,0);
    delay(100);
    asyncGoLeft(162,0);
    delay(100);
    asyncGoForward(162,0);
    delay(100);
    asyncGoLeft(162,0);
    delay(100);*/
    asyncGoForward(162,0);
}

void asyncGoForward(int steps, int delayMs){
    i = 0;
    while(i < steps){
        myStepperFL->onestep(BACKWARD, DOUBLE);
        myStepperFR->onestep(FORWARD, DOUBLE);
        myStepperBL->onestep(BACKWARD, DOUBLE);
        myStepperBR->onestep(FORWARD, DOUBLE);
        i++;
        delay(delayMs);
    }    
}

void asyncGoLeft(int steps, int delayMs){
    i = 0;
    while(i < steps){
      myStepperFL->onestep(FORWARD, DOUBLE);
      myStepperFR->onestep(FORWARD, DOUBLE);
      myStepperBL->onestep(FORWARD, DOUBLE);
      myStepperBR->onestep(FORWARD, DOUBLE);
      i++;
      delay(delayMs);
    }
}

void asyncGoRight(int steps, int delayMs){
  i = 0;
  while(i < steps){
    myStepperFL->onestep(BACKWARD, DOUBLE);
    myStepperFR->onestep(BACKWARD, DOUBLE);
    myStepperBL->onestep(BACKWARD, DOUBLE);
    myStepperBR->onestep(BACKWARD, DOUBLE);
    delay(delayMs);
  }
  
}

void asyncGoBack(int steps, int delayMs){
    i = 0;
    while(i < steps){
    myStepperFL->onestep(FORWARD, DOUBLE);
    myStepperFR->onestep(BACKWARD, DOUBLE);
    myStepperBL->onestep(FORWARD, DOUBLE);
    myStepperBR->onestep(BACKWARD, DOUBLE);
    delay(delayMs);
  }
}

