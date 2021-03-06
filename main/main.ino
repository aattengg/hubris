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
#include <FiniteStateMachine.h>
#include <math.h>
#include "Ultrasonic.h"

/********************
 * Robot Dimensions *
 ********************/
/* Heights in cm */
#define ROBOT_WIDTH                 17
#define ROBOT_HEIGHT                20

/****************************************
 * Ultrasonic Sensors Pin Configuration *
 ****************************************/
#define US_PAIRS                    3
#define US_NUM                      6

// Milliseconds between pings
#define US_PING_INTERVAL            33

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

// Define ultrasonic sensors
Ultrasonic usFL(US_TRIGGER_FL, US_ECHO_FL, US_MAX_DIST);
Ultrasonic usFR(US_TRIGGER_FR, US_ECHO_FR, US_MAX_DIST);
Ultrasonic usLF(US_TRIGGER_LF, US_ECHO_LF, US_MAX_DIST);
Ultrasonic usLB(US_TRIGGER_LB, US_ECHO_LB, US_MAX_DIST);
Ultrasonic usRF(US_TRIGGER_RF, US_ECHO_RF, US_MAX_DIST);
Ultrasonic usRB(US_TRIGGER_RB, US_ECHO_RB, US_MAX_DIST);

// Struct for a pair of ultrasonic sensors.
struct USPair_t {
  Ultrasonic *us[2];
  float usSepDist;
  float distToCenter;
  float angleCurrent; // Angle in radians
  float angleBuffer1 = 0;
  float angleBuffer2 = 0;
  float angle;
  float distance;
};

enum USPairDir {
    USPairDirLeft = 0,
    USPairDirFront,
    USPairDirRight
};

USPair_t USPairs[US_NUM];

Ultrasonic *curUS = NULL;

// Define Address for IMU
MPU6050 accelgyro; //0x68

int16_t ax, ay, az;
int16_t gx, gy, gz;
double pitch, roll;
double fXg = 0;
double fYg = 0;
double fZg = 0;
const float alpha = 0.5;

#define LED_PIN 13 //IMU LED
bool blinkState = false;

volatile bool interrupt = false;

//State Machine Constants
const unsigned int DISTANCE_TOLERANCE = 0;
const unsigned int DRIVE_TO_WALL_DESIRED_DISTANCE = 36;
const unsigned int DRIVE_TO_WALL_REFERENCE_DISTANCE = 25;
const float PITCH_TOLERANCE = 15;
const float ROLL_TOLERANCE = 0.75;
const float ANGLE_TOLERANCE = 3*3.14159/180;
const float BIG_ANGLE_TOLERANCE = 7*3.14/180;
const unsigned int SEARCH_SPACING = 30;
const unsigned int SEARCH_EXPECTED_MAX_DISTANCE = 80;

// State Machine Variables
unsigned int stateInternalCounter = 0;
unsigned int stateInternalCounter2 = 0;
unsigned int stateInternalCounter3 = 0;
bool stateInternalFlag = false;
float rollReference;

/** this is the definitions of the states that our program uses */
void driveToWallUpdate();
void rotateLeft45FirstUpdate();
void rotateLeft45SecondUpdate();
void driveToRampUpdate();
void getOnRampUpdate();
void goUpRampUpdate();
void onFlatRampUpdate();
void goDownRampUpdate();
void findBaseUpdate();
void driveToBaseUpdate();
void idleUpdate();
State driveToWall = State(driveToWallUpdate);  //first state where we drive To the wall
State rotateLeft45First = State(rotateLeft45FirstUpdate);  //state where robot rotates left 90 degrees
State rotateLeft45Second = State(rotateLeft45SecondUpdate);
State driveToRamp = State(driveToRampUpdate);  //keeps itself aligned to the ramp as it drives closer
State getOnRamp = State(getOnRampUpdate); //get the wheels onto the ramp itself via alignment corrections
State goUpRamp = State(goUpRampUpdate); //pretty much just driving straight lol
State onFlatRamp = State(onFlatRampUpdate); //still just driving straight (why is this even a state)
State goDownRamp = State(goDownRampUpdate); //see above -_-
State findBase = State(findBaseUpdate); //drive around and look for the base
State driveToBase = State(driveToBaseUpdate); //probably not needed since findBase should totally cover it
State idle = State(idleUpdate);

/** the state machine controls which of the states get attention and execution time */
FSM stateMachine = FSM(driveToWall); //initialize state machine

//counter variable
static byte currentState = 0; //increment the state in each state function? (we could try something else too)
static byte prevState;

void setup() {
  while (!Serial);
  Serial.begin(115200);   // set up Serial library at 9600 bps
  AFMSbot.begin();      // Start the bottom shield
  AFMStop.begin();      // Start the top shield
  initIMU();
  TWBR = ((F_CPU /400000l) - 16) / 2; // Change the i2c clock to 200KHz so motor can run faster (400KHz is fastest it can go)
  initSonar();
}

void loop() {
    for (int i = 0; i < US_PAIRS; i++) {
        for (int j = 0; j < 2; j++) {
            if (millis() >= USPairs[i].us[j]->pingTimer) {
                USPairs[i].us[j]->pingTimer += US_PING_INTERVAL * US_NUM;
                curUS->us.timer_stop();
                curUS = USPairs[i].us[j];
                curUS->us.ping_timer(echoCheck);
            }
        }
    }
    runFSM();
    /*updateSonar(&USPairs[USPairDirRight]);
    float rightAngle = USPairs[USPairDirRight].angle;
    Serial.print("State: ");
    Serial.println(currentState);
    Serial.print("Angle: ");
    Serial.println(rightAngle * 3.14159/180);
    
    updateSonar(&USPairs[USPairDirRight]);
    updateSonar(&USPairs[USPairDirFront]);
    updateSonar(&USPairs[USPairDirLeft]);
    
    float rightAngle = USPairs[USPairDirRight].angle;
    Serial.print("Right Angle: ");
    Serial.println(rightAngle * 180/3.14159);
    
    Serial.print("Front Face [0]:");
    Serial.println(USPairs[USPairDirFront].us[0]->filteredDist);
    Serial.print("Front Face [1]:");
    Serial.println(USPairs[USPairDirFront].us[1]->filteredDist);
    Serial.print("Right Face [0]:");
    Serial.println(USPairs[USPairDirRight].us[0]->filteredDist);
    Serial.print("Right Face [1]:");
    Serial.println(USPairs[USPairDirRight].us[1]->filteredDist);
    */
}

// Incremental movement helper functions.
void incrementBackward() {
    myStepperFL->onestep(BACKWARD, DOUBLE);
    myStepperFR->onestep(FORWARD, DOUBLE);
    myStepperBL->onestep(BACKWARD, DOUBLE);
    myStepperBR->onestep(FORWARD, DOUBLE);
}

void incrementForward() {
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

void releaseSteppers() {
    myStepperFL->release();
    myStepperFR->release();
    myStepperBL->release();
    myStepperBR->release();
}


// Ultrasonic sensor helper functions.
void initSonar() {
    USPairs[USPairDirLeft].us[0] = &usLB;
    USPairs[USPairDirLeft].us[1] = &usLF;
    USPairs[USPairDirLeft].usSepDist = US_LEFT_SEP;
    USPairs[USPairDirLeft].distToCenter = ROBOT_WIDTH / 2.0;

    USPairs[USPairDirFront].us[0] = &usFL;
    USPairs[USPairDirFront].us[1] = &usFR;
    USPairs[USPairDirFront].usSepDist = US_FRONT_SEP;
    USPairs[USPairDirFront].distToCenter = ROBOT_HEIGHT / 2.0;

    USPairs[USPairDirRight].us[0] = &usRF;
    USPairs[USPairDirRight].us[1] = &usRB;
    USPairs[USPairDirRight].usSepDist = US_RIGHT_SEP;
    USPairs[USPairDirRight].distToCenter = ROBOT_WIDTH / 2.0;

    USPairs[0].us[0]->pingTimer = millis() + 75;
    USPairs[0].us[1]->pingTimer = USPairs[0].us[0]->pingTimer + US_PING_INTERVAL;
    for (int i = 1; i < US_NUM; i++)
    {
        USPairs[i].us[0]->pingTimer = USPairs[i-1].us[1]->pingTimer + US_PING_INTERVAL;
        USPairs[i].us[1]->pingTimer = USPairs[i].us[0]->pingTimer + US_PING_INTERVAL;
    }
}

void echoCheck(){
    if (curUS->us.check_timer())
        curUS->curDist = curUS->us.ping_result / float(US_ROUNDTRIP_CM);
}

void updateSonar(USPair_t *USPair) {
    Ultrasonic *us0 = USPair->us[0];
    Ultrasonic *us1 = USPair->us[1];

    us0->filteredDist = median3Filter(us0->curDist, us0->distBuffer1, us0->distBuffer2);
    us1->filteredDist = median3Filter(us1->curDist, us1->distBuffer1, us1->distBuffer2);

    us0->distBuffer2 = us0->distBuffer1;
    us0->distBuffer1 = us0->curDist;
    us1->distBuffer2 = us1->distBuffer1;
    us1->distBuffer1 = us1->curDist;

    USPair->angleCurrent = asin(float(us0->filteredDist - us1->filteredDist) / (USPair->usSepDist));

    USPair->angle = median3Filter(USPair->angleCurrent, USPair->angleBuffer1, USPair->angleBuffer2);
    USPair->angleBuffer2 = USPair->angleBuffer1;
    USPair->angleBuffer1 = USPair->angleCurrent;

    USPair->distance = (us0->filteredDist + us1->filteredDist)/(2.0) + USPair->distToCenter * cos(USPair->angle);
}

void printSonarData(USPair_t *USPair) {

    Ultrasonic *us0 = USPair->us[0];
    Ultrasonic *us1 = USPair->us[1];

    Serial.print("Angle: ");
    Serial.println(USPair->angle * 180 / 3.1415, 4);
    Serial.print("Distance: ");
    Serial.print(USPair->distance, 4);
    Serial.println("cm");
    /*
    Serial.println("Anthony Debugs:");
    Serial.println(us0->filteredDist - us1->filteredDist);
    Serial.println((us0->curDist + us1->curDist)/(2.0), 4);
    Serial.println(USPair->distToCenter, 4);
    Serial.println(USPair->usSepDist, 4);
    Serial.println(USPair->angleCurrent, 4);
    Serial.println(cos(USPair->angleCurrent), 4);
    Serial.print("US1: ");
    Serial.println(USPair->us[0]->curDist, 4);
    Serial.print("US2: ");
    Serial.println(USPair->us[1]->curDist, 4);
    */
}

// Data processing helper functions.
float median3Filter(float a, float b, float c) {
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
    accelgyro.setDLPFMode(MPU6050_DLPF_BW_5);

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    delay(1000);
    getPR();
    rollReference = roll;
    Serial.println(rollReference);
}

void getPR()
{
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    //Low Pass Filter
    fXg = ax * alpha + (fXg * (1.0 - alpha));
    fYg = ay * alpha + (fYg * (1.0 - alpha));
    fZg = az * alpha + (fZg * (1.0 - alpha));

    //Roll & Pitch Equations
    pitch  = (atan2(-fYg, fZg)*180.0)/M_PI;
    roll = (atan2(fXg, sqrt(fYg*fYg + fZg*fZg))*180.0)/M_PI;

    // display tab-separated pitch/roll values

/*
    Serial.print("pitch/roll:\t");
    Serial.print(pitch); Serial.print("\t");
    Serial.println(roll);
    */
}

void runFSM()
{
    /*
    manipulate the state machine
  */
  //CONTROL THE STATE
    switch (currentState){
      case 0: stateMachine.transitionTo(driveToWall); break;
      case 1: stateMachine.transitionTo(rotateLeft45First); break;
      case 2: stateMachine.transitionTo(rotateLeft45Second); break;
      case 3: stateMachine.transitionTo(driveToRamp); break;
      case 4: stateMachine.transitionTo(getOnRamp); break;
      case 5: stateMachine.transitionTo(goUpRamp); break;
      case 6: stateMachine.transitionTo(onFlatRamp); break;
      case 7: stateMachine.transitionTo(goDownRamp); break;
      case 8: stateMachine.transitionTo(findBase); break;
      case 9: stateMachine.transitionTo(driveToBase); break;
      case 10: stateMachine.transitionTo(idle); break;
    }

  //THIS LINE IS CRITICAL
  //do not remove the stateMachine.update() call, it is what makes this program 'tick'
  stateMachine.update();
}

//State 0
void driveToWallUpdate() {
    updateSonar(&USPairs[USPairDirFront]);
    //updateSonar(&USPairs[USPairDirRight]);

    float frontDistance = USPairs[USPairDirFront].distance;
    //float rightDistance = USPairs[USPairDirRight].distance;
    //float rightAngle = USPairs[USPairDirRight].angle;

    if (frontDistance < DRIVE_TO_WALL_DESIRED_DISTANCE) {
        // The ultrasonic appears to read constant distance of 10 cm for some time after initialization.
        // Wait until the front sees a value that indicates it should not yet transition before beginning proper operation.
        if (stateInternalFlag) {
            stateInternalFlag = false;
            // Trigger state transition. Use a proper enum for this when time permits.
            prevState = currentState;
            currentState = 1;
        }
    }
    else {
        if (!stateInternalFlag) {
            stateInternalFlag = true;
        }/*
        if ((rightDistance < (DRIVE_TO_WALL_REFERENCE_DISTANCE - DISTANCE_TOLERANCE)) && (rightAngle < -ANGLE_TOLERANCE)) {
            incrementRotateLeft();
        }
        else if ((rightDistance > (DRIVE_TO_WALL_REFERENCE_DISTANCE + DISTANCE_TOLERANCE)) && (rightAngle > ANGLE_TOLERANCE)) {
            incrementRotateRight();
        }*/
        else {
            //for (int incrCount = 0; incrCount < 5; incrCount++) {
                incrementForward();
            //}
        }
    }
}

//change name?
void rotateLeft45FirstUpdate() {
    /*updateSonar(&USPairs[USPairDirRight]);
=======
//State 1
void rotateLeft90Update() {
    updateSonar(&USPairs[USPairDirRight]);
>>>>>>> 0526576b0d4fcb186ad8b9f8400bf7c7d62e5f80
    float rightAngle = USPairs[USPairDirRight].angle;
    //will require low angle tolerance and must avoid initial right wall (utilize front sensors as well?)
    //if (!(abs(rightAngle) < ANGLE_TOLERANCE)) {
    if (!isnan(rightAngle)) {
        if (stateInternalCounter2 < 20) {
            stateInternalCounter2++;
        }
        else {
            if (!stateInternalFlag) {
                stateInternalFlag = true;
            }
            incrementRotateLeft();
            stateInternalCounter = 0;
        }
    }
    else if (rightAngle < -ANGLE_TOLERANCE) {
        incrementRotateRight();
        stateInternalCounter = 0;
    }
    else {
        stateInternalCounter2 = 0;
        if (stateInternalFlag) {
            if (stateInternalCounter < 20) {
                stateInternalCounter++;
            }
                else {
                stateInternalFlag = false;
                stateInternalCounter = 0;
                // Trigger state transition. Use a proper enum for this when time permits.
                currentState = 2;
                if (prevState == 0) {
                    currentState = 2;
                }
                else {  // prevState == 7 || prevState == 8;
                    currentState = 8;
                }
            }
        }
        else {
            incrementRotateLeft();
        }
    }*/
    if (stateInternalCounter < 180) { // 150 is fairly reliable
        incrementRotateLeft();
        stateInternalCounter++;
    }
    else {
        stateInternalCounter = 0;
            // Trigger state transition. Use a proper enum for this when time permits.
            if (prevState == 0) {
                currentState = 3;
            }
            else {  // prevState == 7 || prevState == 8;
                currentState = 8;
            }
    }
}

void rotateLeft45SecondUpdate() {
    /*updateSonar(&USPairs[USPairDirRight]);
    float rightAngle = USPairs[USPairDirRight].angle;
    //will require low angle tolerance and must avoid initial right wall (utilize front sensors as well?)
    //if (!(abs(rightAngle) < ANGLE_TOLERANCE)) {
    //GOOD START HERE
    
    if (!(abs(rightAngle) < BIG_ANGLE_TOLERANCE + 2*3.14159/180)) {
      if (stateInternalCounter2 < 10) {
            stateInternalCounter2++;
        }
        else {
            incrementRotateLeft();
            stateInternalCounter = 0;
        // Trigger state transition. Use a proper enum for this when time permits.
        prevState = currentState;
        if (prevState == 0) {
            currentState = 2;
        }
        else if(prevState == 7){  
            currentState = 7;
        }
        else{// prevState == 8;
            currentState = 8;

        }
    }
    */
    //GOOD END HERE
    /*else if (rightAngle < -ANGLE_TOLERANCE) {
        incrementRotateRight();
        stateInternalCounter = 0;
    }*/
    //GOOD START HERE
    /*
    else {
        stateInternalCounter2 = 0;
            if (stateInternalCounter < 10) {
                stateInternalCounter++;
            }
                else {
                stateInternalCounter = 0;
                // Trigger state transition. Use a proper enum for this when time permits.
                if (prevState == 0) {
                    currentState = 3;
                }
                else {  // prevState == 8 || prevState == 9;
                    currentState = 10;
                }
            }
    }*/
    //GOOD END HERE
    /*
    if (stateInternalCounter < 160) { // 150 is fairly reliable
        incrementRotateLeft();
        stateInternalCounter++;
    }*/
    updateSonar(&USPairs[USPairDirRight]);
    float rightAngle = USPairs[USPairDirRight].angle;
    if (isnan(rightAngle)) {
        if (stateInternalCounter3 < 30) {
            stateInternalCounter3++;
        }
        else {
            stateInternalCounter = 0;
            stateInternalCounter2 = 0;
            stateInternalCounter3 = 0;
            // Trigger state transition. Use a proper enum for this when time permits.
            if (prevState == 0) {
                currentState = 3;
            }
            else {  // prevState == 7 || prevState == 8;
                currentState = 8;
            }
        }
    }
    else if (rightAngle < -ANGLE_TOLERANCE) {
        if (stateInternalCounter2 > 0) {
            stateInternalCounter2 = 0;
        }
        if (stateInternalCounter3 > 0) {
            stateInternalCounter3 = 0;
        }
        if (stateInternalCounter < 30) {
            stateInternalCounter++;
        }
        else {
            incrementRotateLeft();
        }
    }
    else if (rightAngle > ANGLE_TOLERANCE) {
        if (stateInternalCounter > 0) {
            stateInternalCounter = 0;
        }
        if (stateInternalCounter3 > 0) {
            stateInternalCounter = 0;
        }
        if (stateInternalCounter2 < 30) {
            stateInternalCounter2++;
        }
        else {
            incrementRotateRight();
        }
    }
    else {
        stateInternalCounter = 0;
        stateInternalCounter2 = 0;
        stateInternalCounter3 = 0;
            // Trigger state transition. Use a proper enum for this when time permits.
            if (prevState == 0) {
                currentState = 3;
            }
            else {  // prevState == 7 || prevState == 8;
                currentState = 8;
            }
    }
}

//State 2
void driveToRampUpdate() {
    /*updateSonar(&USPairs[USPairDirRight]);
    float rightAngle = USPairs[USPairDirRight].angle;
    //double check logic
    if (isnan(rightAngle) || rightAngle > BIG_ANGLE_TOLERANCE) {
        if (stateInternalCounter2 > 0) {
            stateInternalCounter2 = 0;
        }
        if (stateInternalCounter3 > 0) {
            stateInternalCounter3 = 0;
        }
        getPR();
      if (pitch > (99 - PITCH_TOLERANCE)) {
          if (stateInternalCounter > 0) {
              stateInternalCounter = 0;
          }
          incrementForward();
      }
      else {
          if (stateInternalCounter < 100) {
              stateInternalCounter++;
          }
          else {
              stateInternalCounter = 0;
              // Trigger state transition. Use a proper enum for this when time permits.
              currentState = 4;
          }
      }
    }
    else if (rightAngle > ANGLE_TOLERANCE) {
        if (stateInternalCounter3 > 0) {
            stateInternalCounter3 = 0;
        }
        if (stateInternalCounter2 < 100) {
            stateInternalCounter2++;
        }
        else {
            for (int incrCount = 0; incrCount < 5; incrCount++) {
                incrementRotateRight();
                //incrementForward();
            }
        }
    }
    else if(rightAngle < -ANGLE_TOLERANCE)  {
        if (stateInternalCounter2 > 0) {
            stateInternalCounter2 = 0;
        }
        if (stateInternalCounter3 < 100) {
            stateInternalCounter3++;
        }
        else {
            for (int incrCount = 0; incrCount < 5; incrCount++) {
                incrementRotateLeft();
                //incrementForward();
            }
        }
    }
    else  {
      if (stateInternalCounter2 > 0) {
          stateInternalCounter2 = 0;
      }*/
      getPR();
      if (pitch > (99 - PITCH_TOLERANCE)) {
          if (stateInternalCounter > 0) {
              stateInternalCounter = 0;
          }
          for (int incrCount = 0; incrCount < 20; incrCount++) {
              incrementForward();
          }
      }
      else {
          if (stateInternalCounter < 100) {
              stateInternalCounter++;
          }
          else {
              stateInternalCounter = 0;
              // Trigger state transition. Use a proper enum for this when time permits.
              currentState = 4;
          }
      }
    //}
}

//State 3
void getOnRampUpdate() {
    getPR();
    Serial.print("Roll: ");
    Serial.println(roll);
    if (roll < -ROLL_TOLERANCE + rollReference) {
        if (stateInternalCounter2 > 0) {
            stateInternalCounter2 = 0;
        }
        if (stateInternalCounter < 100) {
            stateInternalCounter++;
        }
        else {
        for (int i = 0; i < 150; i++) {
            incrementBackward();
        }
        for (int i = 0; i < 20; i++) {
            incrementRotateLeft();
        }
        delay(500);
        getPR();
        rollReference = roll;
        for (int i = 0; i < 100; i++) {
            incrementForward();
        }
        for (int i = 0; i < 40; i++) {
            incrementRotateRight();
        }
        for (int i = 0; i < 100; i++) {
            incrementForward();
        }
            stateInternalCounter = 0;
        }
    }
    else if (roll > ROLL_TOLERANCE + rollReference) {
        if (stateInternalCounter2 > 0) {
            stateInternalCounter2 = 0;
        }
        if (stateInternalCounter < 100) {
            stateInternalCounter++;
        }
        else {
        releaseSteppers();   
        for (int i = 0; i < 200; i++) {
            incrementBackward();
        }
        for (int i = 0; i < 20; i++) {
            incrementRotateRight();
        }
        delay(500);
        getPR();
        rollReference = roll;
        for (int i = 0; i < 100; i++) {
            incrementForward();
        }
        for (int i = 0; i < 40; i++) {
            incrementRotateLeft();
        }
        for (int i = 0; i < 150; i++) {
            incrementForward();
        }
            stateInternalCounter = 0;
        }
    }
    else if (pitch > (99 - PITCH_TOLERANCE)) {
        if (stateInternalCounter > 0) {
            stateInternalCounter = 0;
        }
        if (stateInternalCounter2 > 0) {
            stateInternalCounter2 = 0;
        }
        for (int incrCount = 0; incrCount < 100; incrCount++) {
            incrementForward();
        }
    }
    else {
        if (stateInternalCounter2 < 100) {
            stateInternalCounter2++;
        }
        else {
            releaseSteppers();
            stateInternalCounter = 0;
            stateInternalCounter2 = 0;
            // Trigger state transition. Use a proper enum for this when time permits.
            currentState = 5;
        }
    }
}

//State 4
void goUpRampUpdate() {
    getPR();
    if (pitch < (99 - PITCH_TOLERANCE)) {
        if (stateInternalCounter > 0) {
            stateInternalCounter = 0;
        }
        for (int incrCount = 0; incrCount < 2000; incrCount++) {
            incrementForward();
        }
    }
    else {
        if (stateInternalCounter < 100) {
            stateInternalCounter++;
        }
        else {
            stateInternalCounter = 0;
            // Trigger state transition. Use a proper enum for this when time permits.
            currentState = 6;
        }
    }
}

//State 5
void onFlatRampUpdate() {
    getPR();
    if (pitch < (99 + PITCH_TOLERANCE)) {
        if (stateInternalCounter > 0) {
            stateInternalCounter = 0;
        }
        for (int incrCount = 0; incrCount < 50; incrCount++) {
            incrementForward();
        }
    }
    else {
        if (stateInternalCounter < 100) {
            stateInternalCounter++;
        }
        else {
            stateInternalCounter = 0;
            // Trigger state transition. Use a proper enum for this when time permits.
            currentState = 7;
        }
    }
}

//State 6
void goDownRampUpdate() {
    getPR();
    if (pitch > (99 + PITCH_TOLERANCE)) {
        if (stateInternalCounter > 0) {
            stateInternalCounter = 0;
        }
        for (int incrCount = 0; incrCount < 2000; incrCount++) {
            incrementForward();
        }
    }
    else {
        if (stateInternalCounter < 10) {
            stateInternalCounter++;
        }
        else {
            stateInternalCounter = 0;
            // Trigger state transition. Use a proper enum for this when time permits.
            currentState = 8;
        }
    }
}

//State 7
void findBaseUpdate() {
    updateSonar(&USPairs[USPairDirLeft]);
    updateSonar(&USPairs[USPairDirFront]);
    updateSonar(&USPairs[USPairDirRight]);

    float leftDistance = USPairs[USPairDirLeft].distance;
    float frontDistance = USPairs[USPairDirFront].distance;
    float rightDistance = USPairs[USPairDirRight].distance;
    float rightAngle = USPairs[USPairDirRight].angle;

    //found the base
    if (leftDistance < SEARCH_EXPECTED_MAX_DISTANCE) {
        // Trigger state transition. Use a proper enum for this when time permits.
        currentState = 10;
    }
    else if (frontDistance < SEARCH_SPACING) {
        // Trigger state transition. Use a proper enum for this when time permits.
        prevState = currentState;
        currentState = 1;
    }
    else {
        if ((rightDistance < (DRIVE_TO_WALL_REFERENCE_DISTANCE - DISTANCE_TOLERANCE)) && (rightAngle < ANGLE_TOLERANCE)) {
            incrementRotateLeft();
        }
        else if ((rightDistance > (DRIVE_TO_WALL_REFERENCE_DISTANCE + DISTANCE_TOLERANCE)) && (rightAngle > ANGLE_TOLERANCE)) {
            incrementRotateRight();
        }
        else {
            incrementForward();
        }
    }
}

//State 8
void driveToBaseUpdate() {
    updateSonar(&USPairs[USPairDirFront]);
    
    float frontDistanceLeft = USPairs[USPairDirFront].us[0]->filteredDist;
    float frontDistanceRight = USPairs[USPairDirFront].us[1]->filteredDist;
    
//    if(prevState == 7)
//    {
//        prevState == currentState;
//        currentState = 1;
//    }
//    else
//    {
//      if (frontAngle > ANGLE_TOLERANCE)
//      {
//          for(int i = 0; i < 10; i++)
//            incrementRotateRight();
//      }
//      else if(frontAngle < ANGLE_TOLERANCE)
//      {
//          for(int i = 0; i < 10; i++)
//            incrementRotateLeft();
//      }
//      incrementForward();
//      if(frontDistance == DISTANCE_TOLERANCE)
//            currentState = 9;
//    }
}

//State 9
void idleUpdate () {
    delay(1000);
}
