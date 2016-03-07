/* 
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2 
---->  http://www.adafruit.com/products/1438
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Define Address for the Two Motor Sheilds
Adafruit_MotorShield AFMSbot(0x61); // Rightmost jumper closed
Adafruit_MotorShield AFMStop(0x60); // Default address, no jumpers

// Define Motors
Adafruit_StepperMotor *myStepperFL = AFMStop.getStepper(200, 1); //Front Left
Adafruit_StepperMotor *myStepperFR = AFMStop.getStepper(200, 2); // Front Right
Adafruit_StepperMotor *myStepperBL = AFMSbot.getStepper(200, 2); //Bottom Left
Adafruit_StepperMotor *myStepperBR = AFMSbot.getStepper(200, 1); //Bottom Right


void setup() {
  while (!Serial);
  Serial.begin(9600);           // set up Serial library at 9600 bps
  AFMSbot.begin(); // Start the bottom shield
  AFMStop.begin(); // Start the top shield
  TWBR = ((F_CPU /200000l) - 16) / 2; // Change the i2c clock to 200KHz so motor can run faster (400KHz is fastest it can go)
}

int i;
void loop() {
    
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
    delay(100);
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

void asyncGoBack(int steps){
    i = 0;
    while(i < steps){
    myStepperFL->onestep(FORWARD, DOUBLE);
    myStepperFR->onestep(BACKWARD, DOUBLE);
    myStepperBL->onestep(FORWARD, DOUBLE);
    myStepperBR->onestep(BACKWARD, DOUBLE);
    delay(delayMs);
  }
}

