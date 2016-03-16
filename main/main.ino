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
#include <I2Cdev.h>
#include <MPU6050.h>

/********************
 * Robot Dimensions *
 ********************/
/* Heights in cm */
#define ROBOT_WIDTH                 17
#define ROBOT_HEIGHT                20

/****************************************
 * Ultrasonic Sensors Pin Configuration *
 ****************************************/
/* Distances in cm */
#define US_MAX_DIST                 200

#define US_FRONT_SEP                5.5
#define US_LEFT_SEP                 12
#define US_RIGHT_SEP                12

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
Adafruit_StepperMotor *myStepperBR = AFMStop.getStepper(200, 2);  // Bottom Right
Adafruit_StepperMotor *myStepperFL = AFMSbot.getStepper(200, 1);  //Front Left
Adafruit_StepperMotor *myStepperBL = AFMSbot.getStepper(200, 2);  //Bottom Left

// Define sonars from the ultrasonic sensors
NewPing sonarFL(US_TRIGGER_FL, US_ECHO_FL, US_MAX_DIST);
NewPing sonarFR(US_TRIGGER_FR, US_ECHO_FR, US_MAX_DIST);
NewPing sonarLF(US_TRIGGER_LF, US_ECHO_LF, US_MAX_DIST);
NewPing sonarLB(US_TRIGGER_LB, US_ECHO_LB, US_MAX_DIST);
NewPing sonarRF(US_TRIGGER_RF, US_ECHO_RF, US_MAX_DIST);
NewPing sonarRB(US_TRIGGER_RB, US_ECHO_RB, US_MAX_DIST);

// Ultrasonic variables.
struct uSPair_t {
  NewPing *sonar1;
  NewPing *sonar2;
  unsigned int uS1Filtered;
  unsigned int uS1Current;
  unsigned int uS1Buffer1 = 0;
  unsigned int uS1Buffer2 = 0;
  unsigned int uS2Filtered;
  unsigned int uS2Current;
  unsigned int uS2Buffer1 = 0;
  unsigned int uS2Buffer2 = 0;
  float uSSepDist;
  float distToCenter;
  int angleCurrent;
  int angleBuffer1 = 0;
  int angleBuffer2 = 0;
  int angleFiltered;
  unsigned int distance;
};

// Define Address for IMU
MPU6050 accelgyro; //0x68

int16_t ax, ay, az;
int16_t gx, gy, gz;
double fXg = 0;
double fYg = 0;
double fZg = 0;
const float alpha = 0.5;

#define LED_PIN 13 //IMU LED
bool blinkState = false;

uSPair_t uSFront;
uSPair_t uSLeft;
uSPair_t uSRight;

void setup() {
  while (!Serial);
  Serial.begin(115200);   // set up Serial library at 9600 bps
  AFMSbot.begin();      // Start the bottom shield
  AFMStop.begin();      // Start the top shield
  initIMU();
  TWBR = ((F_CPU /200000l) - 16) / 2; // Change the i2c clock to 200KHz so motor can run faster (400KHz is fastest it can go)
  initSonar();
}

int i;
void loop() {
//    updateSonar(&uSFront);
//    updateSonar(&uSLeft);
//    updateSonar(&uSRight);
//
//    //printSonarData(uSFront);
//    //printSonarData(uSLeft);
//    printSonarData(uSRight);

    getYPR();
    for(int i=0;i<100;i++)
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

// Incremental movement helper functions.
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

// Ultrasonic sensor helper functions.
void initSonar() {
  uSFront.sonar1 = &sonarFL;
  uSFront.sonar2 = &sonarFR;
  uSFront.uSSepDist = US_FRONT_SEP;
  uSFront.distToCenter = ROBOT_HEIGHT / 2.0;
  uSLeft.sonar1 = &sonarLB;
  uSLeft.sonar2 = &sonarLF;
  uSLeft.uSSepDist = US_LEFT_SEP;
  uSLeft.distToCenter = ROBOT_WIDTH / 2.0;
  uSRight.sonar1 = &sonarRF;
  uSRight.sonar2 = &sonarRB;
  uSRight.uSSepDist = US_RIGHT_SEP;
  uSRight.distToCenter = ROBOT_WIDTH / 2.0;
}

void updateSonar(uSPair_t* uSPair) {
    uSPair->uS1Current = uSPair->sonar1->ping();
    uSPair->uS2Current = uSPair->sonar2->ping();

    uSPair->uS1Filtered = median3Filter(uSPair->uS1Current, uSPair->uS1Buffer1, uSPair->uS1Buffer2);
    uSPair->uS2Filtered = median3Filter(uSPair->uS2Current, uSPair->uS2Buffer1, uSPair->uS2Buffer2);

    uSPair->uS1Buffer2 = uSPair->uS1Buffer1;
    uSPair->uS1Buffer1 = uSPair->uS1Current;
    uSPair->uS2Buffer2 = uSPair->uS2Buffer1;
    uSPair->uS2Buffer1 = uSPair->uS2Current;

    uSPair->angleCurrent = (int)(180 / 3.14 * asin(int(uSPair->uS1Filtered - uSPair->uS2Filtered) / (US_ROUNDTRIP_CM * uSPair->uSSepDist)));
    uSPair->distance = (uSPair->uS1Filtered + uSPair->uS2Filtered)/(US_ROUNDTRIP_CM * 2.0) + uSPair->distToCenter * cos(uSPair->angleCurrent) + 0.5;

    uSPair->angleFiltered = median3Filter(uSPair->angleCurrent, uSPair->angleBuffer1, uSPair->angleBuffer2);

    uSPair->angleBuffer2 = uSPair->angleBuffer1;
    uSPair->angleBuffer1 = uSPair->angleCurrent;
}

void printSonarData(uSPair_t uSPair) {

    Serial.print("Angle: ");
    Serial.println(uSPair.angleFiltered);
    Serial.print("Distance: ");
    Serial.print(uSPair.distance);
    Serial.println("cm");
    /*
    Serial.print("US1: ");
    Serial.println(uSPair.uS1Filtered / US_ROUNDTRIP_CM);
    Serial.print("US2: ");
    Serial.println(uSPair.uS2Filtered / US_ROUNDTRIP_CM);
    */

}

// Data processing helper functions.
int median3Filter(int a, int b, int c) {
    if (a >= b)
    {
      if (a <= c)
      {
        return a; // b <= a <= c
      }
      else  // a >= b; a > c
      {
        if (b >= c)
        {
          return b; // c <= b <= a
        }
        else
        {
          return c; // b < c < a
        }
      }
    }
    else
    {
      if (a >= c)
      {
        return a; // c <= a < b
      }
      else  // a < b; a < c
      {
        if (b <= c)
        {
          return b; // a < b <= c
        }
        else
        {
          return c; // a < c < b
        }
      }
    }
}

void initIMU()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize device
    accelgyro.initialize();

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}

void getYPR()
{     
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
 
    //Low Pass Filter
    fXg = ax * alpha + (fXg * (1.0 - alpha));
    fYg = ay * alpha + (fYg * (1.0 - alpha));
    fZg = az * alpha + (fZg * (1.0 - alpha));
 
    //Roll & Pitch Equations
    double roll  = (atan2(-fYg, fZg)*180.0)/M_PI;
    double pitch = (atan2(fXg, sqrt(fYg*fYg + fZg*fZg))*180.0)/M_PI;
    
    // display tab-separated pitch/roll/yaw values
    Serial.print("a/g:\t");
    Serial.print(pitch); Serial.print("\t");
    Serial.println(roll);
    //Serial.println(yaw);
}

