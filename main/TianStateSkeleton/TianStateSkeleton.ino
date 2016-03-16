#include <FiniteStateMachine.h>

//how many states are we cycling through?
const byte NUMBER_OF_SELECTABLE_STATES = 10;
const unsigned int DRIVE_TO_WALL_DESIRED_DISTANCE = 15;
const float PITCH_TOLERANCE = 80;

/** this is the definitions of the states that our program uses */
State driveToWall = State(driveToWallUpdate);  //first state where we drive To the wall
State adjustToRamp = State(adjustToRamp);  //state where robot turns to face the ramp
State driveToRamp = State(driveToRampUpdate);  //keeps itself aligned to the ramp as it drives closer
State getOnRamp = State(getOnRamp); //get the wheels onto the ramp itself via alignment corrections
State goUpRamp = State(goUpRamp); //pretty much just driving straight lol
State readjustOnRamp = State(readjustOnRamp); //comes here when we slip off ramp
State onFlatRamp = State(onFlatRamp); //still just driving straight (why is this even a state)
State goDownRamp = State(goDownRamp); //see above -_-
State findBase = State(findBase); //drive around and look for the base
State driveToBase = State(driveToBase); //probably not needed since findBase should totally cover it

/** the state machine controls which of the states get attention and execution time */
FSM stateMachine = FSM(driveToWall); //initialize state machine, start in state: noop

//counter variable
static byte currentState = 0; //increment the state in each state function? (we could try something else too)
  
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
      case 1: stateMachine.transitionTo(adjustToRamp); break;
      case 2: stateMachine.transitionTo(driveToRamp); break;
      case 3: stateMachine.transitionTo(getOnRamp); break;
      case 4: stateMachine.transitionTo(goUpRamp); break;
      case 5: stateMachine.transitionTo(readjustOnRamp); break;
      case 6: stateMachine.transitionTo(onFlatRamp); break;
      case 7: stateMachine.transitionTo(goDownRamp); break;
      case 8: stateMachine.transitionTo(findBase); break;
      case 9: stateMachine.transitionTo(driveToBase); break;
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
    if (frontDistance > DRIVE_TO_WALL_DESIRED_DISTANCE) {
        incrementForward();
    }
    else {
        // Trigger state transition. Use a proper enum for this when time permits.
        currentState = 1;
    }
}

void adjustToRampUpdate() {
    if (rightSensorNewData) {
        updateSonar(&uSRight);
    }
    if (uSRight.angleFiltered < 80 || uSRight.angleFiltered > 100) {
        incrementRotateLeft();
    }
    else {
        // Trigger state transition. Use a proper enum for this when time permits.
        currentState = 2;
    }
}

void driveToRampUpdate() {
    if (rightSensorsNewData) {
        updateSonar(&uSRight);
    }
    getYPR();
    if (pitch > PITCH_TOLERANCE) {
        incrementForward();
    }
    else {
        // Trigger state transition. Use a proper enum for this when time permits.
        currentState = 3;
    }
}
