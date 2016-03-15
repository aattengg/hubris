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

#include <Adafruit_MotorShield.h>
#include <NewPing.h>
#include <Wire.h>

/****************************************
 * Ultrasonic Sensors Pin Configuration *
 ****************************************/
#define US_MAX_DIST 200

/* Distances in cm */
#define US_FRONT_SEP
#define US_FRONT_DIST_TO_CENTER

/* Front Left Sensor */
#define US_ECHO_FL      38
#define US_TRIGGER_FL   39

/* Front Right Sensor */
#define US_ECHO_FR      40
#define US_TRIGGER_FR   41

/* Left Front Sensor */
#define US_ECHO_LF      24
#define US_TRIGGER_LF   25

/* Left Back Sensor */
#define US_ECHO_LB      22
#define US_TRIGGER_LB   23

/* Right Front Sensor */
#define US_ECHO_RF      50
#define US_TRIGGER_RF   51

/* Right Back Sensor */
#define US_ECHO_RB      52
#define US_TRIGGER_RB   53


// Define Address for the Two Motor Sheilds
Adafruit_MotorShield AFMSbot(0x61); // Rightmost jumper closed
Adafruit_MotorShield AFMStop(0x60); // Default address, no jumpers

// Define Motors
Adafruit_StepperMotor *myStepperFR = AFMStop.getStepper(200, 1);  //Front Right
Adafruit_StepperMotor *myStepperBR= AFMStop.getStepper(200, 2);   // Bottom Right
Adafruit_StepperMotor *myStepperFL = AFMSbot.getStepper(200, 1);  //Front Left
Adafruit_StepperMotor *myStepperBL = AFMSbot.getStepper(200, 2);  //Bottom Left

// Define sonars from the ultrasonic sensors
NewPing sonarFL(US_TRIGGER_FL, US_ECHO_FL, US_MAX_DIST);
NewPing sonarFR(US_TRIGGER_FR, US_ECHO_FR, US_MAX_DIST);
NewPing sonarLF(US_TRIGGER_LF, US_ECHO_LF, US_MAX_DIST);
NewPing sonarLB(US_TRIGGER_LB, US_ECHO_LB, US_MAX_DIST);
NewPing sonarRF(US_TRIGGER_RF, US_ECHO_RF, US_MAX_DIST);
NewPing sonarRB(US_TRIGGER_RB, US_ECHO_RB, US_MAX_DIST);

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
    int uS1 = sonar1.ping();
    int uS2 = sonar2.ping();

    float angle = asin((uS1 - uS2) / (US_ROUNDTRIP_CM * 15.0));
    int distance = (uS1 + uS2)/(US_ROUNDTRIP_CM * 2.0) + 5/2.0 * sin(angle) + 0.5;

    Serial.print("Angle: ");
    Serial.println(180 / 3.14 * angle);

    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println("cm");
*/
    incrementForward();
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

// Incremental movement functions.
void incrementForward() {
    myStepperFL->onestep(BACKWARD, DOUBLE);
    myStepperFR->onestep(FORWARD, DOUBLE);
    myStepperBL->onestep(BACKWARD, DOUBLE);
    myStepperBR->onestep(FORWARD, DOUBLE);
}

void incrementBackward() {
    myStepperFL->onestep(FORWARD, DOUBLE);
    myStepperFR->onestep(BACKWARD, DOUBLE);
    myStepperBL->onestep(FORWARD, DOUBLE);
    myStepperBR->onestep(BACKWARD, DOUBLE);
}

void incrementRotateLeft() {
    myStepperFL->onestep(FORWARD, DOUBLE);
    myStepperFR->onestep(FORWARD, DOUBLE);
    myStepperBL->onestep(FORWARD, DOUBLE);
    myStepperBR->onestep(FORWARD, DOUBLE);
}

void incrementRotateRight() {
    myStepperFL->onestep(BACKWARD, DOUBLE);
    myStepperFR->onestep(BACKWARD, DOUBLE);
    myStepperBL->onestep(BACKWARD, DOUBLE);
    myStepperBR->onestep(BACKWARD, DOUBLE);
}
