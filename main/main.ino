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
  int angleCurrent;
  int angleBuffer1 = 0;
  int angleBuffer2 = 0;
  int angleFiltered;
  unsigned int distance;
};

typedef enum USPairDir {
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
                if (i == 0 && curUS == USPairs[US_PAIRS-1].us[1]) {

                    //printSonarData(&USPairs[USPairDirLeft]);
                    //printSonarData(&USPairs[USPairDirFront]);
                    printSonarData(&USPairs[USPairDirRight]);
                }
                curUS->us.timer_stop();
                curUS = USPairs[i].us[j];
                curUS->us.ping_timer(echoCheck);
            }
        }
    }
    getPR();
    incrementForward();
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
        curUS->curDist = curUS->us.ping_result / US_ROUNDTRIP_CM;
}

/*
void updateSonar(USPair_t* USPair) {
    USPair->US1Current = USPair->us1->ping();
    USPair->US2Current = USPair->us2->ping();

    USPair->US1Filtered = median3Filter(USPair->US1Current, USPair->US1Buffer1, USPair->US1Buffer2);
    USPair->US2Filtered = median3Filter(USPair->US2Current, USPair->US2Buffer1, USPair->US2Buffer2);

    USPair->US1Buffer2 = USPair->US1Buffer1;
    USPair->US1Buffer1 = USPair->US1Current;
    USPair->US2Buffer2 = USPair->US2Buffer1;
    USPair->US2Buffer1 = USPair->US2Current;

    USPair->angleCurrent = (int)(180 / 3.14 * asin(int(USPair->US1Filtered - USPair->US2Filtered) / (US_ROUNDTRIP_CM * USPair->USSepDist)));
    USPair->distance = (USPair->US1Filtered + USPair->US2Filtered)/(US_ROUNDTRIP_CM * 2.0) + USPair->distToCenter * cos(USPair->angleCurrent) + 0.5;

    USPair->angleFiltered = median3Filter(USPair->angleCurrent, USPair->angleBuffer1, USPair->angleBuffer2);

    USPair->angleBuffer2 = USPair->angleBuffer1;
    USPair->angleBuffer1 = USPair->angleCurrent;
}
*/

void printSonarData(USPair_t *USPair) {

    /*
    Serial.print("Angle: ");
    Serial.println(USPair->angleFiltered);
    Serial.print("Distance: ");
    Serial.print(USPair->distance);
    Serial.println("cm");
    */

    Serial.print("US1: ");
    Serial.println(USPair->us[0]->curDist);
    Serial.print("US2: ");
    Serial.println(USPair->us[1]->curDist);

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

void getPR()
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

/*
    Serial.print("a/g:\t");
    Serial.print(pitch); Serial.print("\t");
    Serial.println(roll);
    */
    //Serial.println(yaw);
}
