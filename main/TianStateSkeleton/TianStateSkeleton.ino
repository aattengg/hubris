#include <FiniteStateMachine.h>

//how many states are we cycling through?
const byte NUMBER_OF_SELECTABLE_STATES = 10;
const unsigned int DISTANCE_TOLERANCE = 2;
const unsigned int DRIVE_TO_WALL_DESIRED_DISTANCE = 15;
const unsigned int DRIVE_TO_WALL_REFERENCE_DISTANCE = 15;
const float PITCH_TOLERANCE = 10;
const float ROLL_TOLERANCE = 5;
const float ANGLE_TOLERANCE = 10;
const unsigned int SEARCH_SPACING = 30;
const unsigned int SEARCH_EXPECTED_MAX_DISTANCE = 80;

/** this is the definitions of the states that our program uses */
State driveToWall = State(driveToWallUpdate);  //first state where we drive To the wall
State rotateLeft90 = State(rotateLeft90Update);  //state where robot rotates left 90 degrees
State driveToRamp = State(driveToRampUpdate);  //keeps itself aligned to the ramp as it drives closer
State getOnRamp = State(getOnRampUpdate); //get the wheels onto the ramp itself via alignment corrections
State goUpRamp = State(goUpRampUpdate); //pretty much just driving straight lol
State onFlatRamp = State(onFlatRampUpdate); //still just driving straight (why is this even a state)
State goDownRamp = State(goDownRampUpdate); //see above -_-
State findBaseLeg1 = State(findBaseLeg1Update); //drive around and look for the base
State findBaseLeg2 = State(findBaseLeg2Update);
State driveToBase = State(driveToBaseUpdate); //probably not needed since findBase should totally cover it
State idle = State(idleUpdate);

/** the state machine controls which of the states get attention and execution time */
FSM stateMachine = FSM(driveToWall); //initialize state machine, start in state: noop

//counter variable
static byte currentState = 0; //increment the state in each state function? (we could try something else too)
static byte prevState;
  
void setup(){
  //all the init shit from our sensors and motors
}

//get dat state machine started
void loop(){
  /*
    manipulate the state machine
  */
  //CONTROL THE STATE
    switch (currentState){
      case 0: stateMachine.transitionTo(driveToWall); break;
      case 1: stateMachine.transitionTo(rotateLeft90); break;
      case 2: stateMachine.transitionTo(driveToRamp); break;
      case 3: stateMachine.transitionTo(getOnRamp); break;
      case 4: stateMachine.transitionTo(goUpRamp); break;
      case 5: stateMachine.transitionTo(onFlatRamp); break;
      case 6: stateMachine.transitionTo(goDownRamp); break;
      case 7: stateMachine.transitionTo(findBaseLeg1); break;
      case 8: stateMachine.transitionTo(findBaseLeg2); break;
      case 9: stateMachine.transitionTo(driveToBase); break;
      case 10: stateMachine.transitionTo(idle); break;
    }
  }
  //THIS LINE IS CRITICAL
  //do not remove the stateMachine.update() call, it is what makes this program 'tick'
  stateMachine.update();
}

/*
  ALL the functions below are helper functions for the states of the program
*/
///[example state:update] this state does whateva
void example() {
  //just chillinDude
}

void driveToWallUpdate() {
    if (frontSensorsNewData) {
        updateSonar(&uSFront);
    }
    if (rightSensorsNewData) {
        updateSonar(&uSRight);
    }
    if (uSFront.distance < DRIVE_TO_WALL_DESIRED_DISTANCE) {
        // Trigger state transition. Use a proper enum for this when time permits.
        prevState = currentState;
        currentState = 1;
    }
    else {
        if ((uSRight.distance < (DRIVE_TO_WALL_REFERENCE_DISTANCE - DISTANCE_TOLERANCE)) && (uSRight.angleFiltered < ANGLE_TOLERANCE)) {
            incrementRotateLeft();
        }
        else if ((uSRight.distance > (DRIVE_TO_WALL_REFERENCE_DISTANCE + DISTANCE_TOLERANCE)) && (uSRight.angleFiltered > ANGLE_TOLERANCE)) {
            incrementRotateRight();
        }
        else {
            incrementForward();
        }
    }
}

void rotateLeft90Update() {
    if (rightSensorNewData) {
        updateSonar(&uSRight);
    }
    if (abs(uSRight.angleFiltered - 90) > ANGLE_TOLERANCE) {
        incrementRotateLeft();
    }
    else {
        // Trigger state transition. Use a proper enum for this when time permits.
        if (prevState == 0) {
            currentState = 2;
        }
        else  // prevState == 7 || prevState == 8;
            currentState = 8;
        }
    }
}

void driveToRampUpdate() {
    if (rightSensorsNewData) {
        updateSonar(&uSRight);
    }
    getYPR();
    if (pitch > (90 - PITCH_TOLERANCE)) {
        incrementForward();
    }
    else {
        // Trigger state transition. Use a proper enum for this when time permits.
        currentState = 3;
    }
}

void getOnRampUpdate() {
    getYPR();
    if (roll < -ROLL_TOLERANCE) {
        for (i = 0; i < 100; i++) {
            incrementBackward();
        }
        for (i = 0; i < 50; i++) {
            incrementRotateRight();
        }
        for (i = 0; i < 100; i++) {
            incrementForward();
        }
        for (i = 0; i < 50; i++) {
            incrementRotateLeft();
        }
    }
    else if (roll > ROLL_TOLERANCE) {
        for (i = 0; i < 100; i++) {
            incrementBackward();
        }
        for (i = 0; i < 50; i++) {
            incrementRotateLeft();
        }
        for (i = 0; i < 100; i++) {
            incrementForward();
        }
        for (i = 0; i < 50; i++) {
            incrementRotateRight();
        }
    }
    else if (pitch > (90 - PITCH_TOLERANCE)) {
        incrementForward();
    }
    else {
        // Trigger state transition. Use a proper enum for this when time permits.
        currentState = 4;
    }
}

void goUpRampUpdate() {
    getYPR();
    if (pitch < (90 - PITCH_TOLERANCE)) {
        incrementForward();
    }
    else {
        // Trigger state transition. Use a proper enum for this when time permits.
        currentState = 5;
    }
}

void onFlatRampUpdate() {
    getYPR();
    if (pitch < (90 + PITCH_TOLERANCE)) {
        incrementForward();
    }
    else {
        // Trigger state transition. Use a proper enum for this when time permits.
        currentState = 6;
    }
}

void goDownRampUpdate() {
    getYPR();
    if (pitch > (90 + PITCH_TOLERANCE)) {
        incrementForward();
    }
    else {
        // Trigger state transition. Use a proper enum for this when time permits.
        currentState = 7;
    }
}

void findBaseLeg1Update() {
    if (frontSensorsNewData) {
        updateSonar(&uSFront);
    }
    if (rightSensorsNewData) {
        updateSonar(&uSRight);
    }
    if (leftSensorsNewData) {
        updateSonar(&uSLeft);
    }
    if (uSLeft.distance < SEARCH_EXPECTED_MAX_DISTANCE) {
        // Trigger state transition. Use a proper enum for this when time permits.
        currentState = 9;
    }
    else if (uSFront.distance < SEARCH_SPACING) {
        // Trigger state transition. Use a proper enum for this when time permits.
        prevState = currentState;
        currentState = 1;
    }
    else {
        if ((uSRight.distance < (DRIVE_TO_WALL_REFERENCE_DISTANCE - DISTANCE_TOLERANCE)) && (uSRight.angleFiltered < ANGLE_TOLERANCE)) {
            incrementRotateLeft();
        }
        else if ((uSRight.distance > (DRIVE_TO_WALL_REFERENCE_DISTANCE + DISTANCE_TOLERANCE)) && (uSRight.angleFiltered > ANGLE_TOLERANCE)) {
            incrementRotateRight();
        }
        else {
            incrementForward();
        }
    }
}

void findBaseLeg2Update() {
    if (frontSensorsNewData) {
        updateSonar(&uSFront);
    }
    if (rightSensorsNewData) {
        updateSonar(&uSRight);
    }
    if (leftSensorsNewData) {
        updateSonar(&uSLeft);
    }
    if (uSLeft.distance < SEARCH_EXPECTED_MAX_DISTANCE) {
        // Trigger state transition. Use a proper enum for this when time permits.
        currentState = 9;
    }
    else if (uSFront.distance < SEARCH_SPACING) {
        // Trigger state transition. Use a proper enum for this when time permits.
        currentState = 1;
    }
    else {
        if ((uSRight.distance < (SEARCH_SPACING - DISTANCE_TOLERANCE)) && (uSRight.angleFiltered < ANGLE_TOLERANCE)) {
            incrementRotateLeft();
        }
        else if ((uSRight.distance > (SEARCH_SPACING + DISTANCE_TOLERANCE)) && (uSRight.angleFiltered > ANGLE_TOLERANCE)) {
            incrementRotateRight();
        }
        else {
            incrementForward();
        }
    }
}

void driveToBaseUpdate() {
    
}

void idleUpdate {
    delay(1000);
}
