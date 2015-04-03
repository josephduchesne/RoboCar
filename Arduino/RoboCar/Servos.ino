/**
 * Servo handling functions for control and position estimation
 *
 * @author Joseph Duchesne
 */
 
// Servo lib for camera servos
#include <Servo.h>

//a few useful constants
#define SERVO_PITCH 0
#define SERVO_YAW 1

//fraction multiplier consts
#define NUMERATOR 0
#define DENOMINATOR 1

//sweep constants
#define SWEEP_DEGREES_PER_MILLISECOND 0.285
#define SWEEP_DEGREE_OFFSET 78.326
#define SWEEP_PRE_BLANK 50000
#define SWEEP_POST_BLANK 460000
#define SWEEP_END 800000
int sweep_stage = 0;
short int sweep_direction = 1;  //invert angle sign for backwards sweep
long int sweep_time = 0;

//camera servos: pitch, yaw
const int servoPins[2] = {11,10};
//for pitch, yaw what are the min/max valid values
const int servoRanges[2][2] = {{-10,100}, {-65,65}}; 
//for pitch, yaw what are the microsecond timings for those degrees

const int servoTimings[2][2] = {{710,1920}, {858 ,2242}}; 

//servo position estimation vars/consts
int servoPositionsOld[2] = {0}; 
int servoPositions[2] = {0}; 
int timeOfLastServoChange[2] = {0};
//0.23 sec/60deg @4.8v as per sheed, estimated 0.224 sec/60deg @5v via linear approx.
//for fast and accurate int math, I'm storing a fraction: 0: numerator, 1: denominator
const long int servoDegreesPerMillisecond[2] = {60, 224};
const int maxServoMoveTime = 700; //ms. At 0.224/60deg it would take 672ms to move 180deg.

//servo objects for writing
Servo servos[2];

/**
 * Init servo objects
 * 
 * @return void
 */
void servo_setup() {
  int i;
  
  //attach the servo objects and
  for (i=0;i<2;i++) {
    servos[i].attach(servoPins[i]);
  }
  //move the servos to their home positions (0 degrees horizontal and vertical0
  setServo('P', 0);
  setServo('Y', 0);
}

/**
 * Using the servo's estimated speed, old position, and time since last move command
 * estimate the servo's current position in degrees
 * 
 * @param int offset The servo offset (0 for pitch, 1 for yaw)
 *
 * @return int The current estimated servo position in degrees
 */
int estimateServoPosition(int offset){
  long int delta = millis()-timeOfLastServoChange[offset];
  
  //estimation cutoff to avoid wraparound errors
  if (!timeOfLastServoChange[offset] || delta>maxServoMoveTime) {
    timeOfLastServoChange[offset]=0; //skip estimation in the future
    return servoPositions[offset]; //it's clearly arrived
  }
  
  //multiply the time delta by the degrees/milli fraction to get degree delta
  long int degreeDelta = delta * servoDegreesPerMillisecond[NUMERATOR] / servoDegreesPerMillisecond[DENOMINATOR];
  
  //add or subtract the degree delta and constrain between the old and the new position values
  if (servoPositions[offset]>servoPositionsOld[offset]) { //+ direction
    return constrain(servoPositionsOld[offset]+degreeDelta, servoPositionsOld[offset], servoPositions[offset]);
  } else {  //- direction
    return constrain(servoPositionsOld[offset]-degreeDelta, servoPositions[offset], servoPositionsOld[offset]);
  }
}

/**
 * Set the servos ('P' or 'Y') to an angle
 *
 * @param char dir   The side 'P' for pitch up/down, 'Y' for yaw side to side
 * @param int  angle The angle in degrees
 *
 * @return void
 */
void setServo(char servo, int angle) {
  int microseconds = 0;
  
  int offset = SERVO_PITCH; //pitch defaults to offset 0
  if (servo=='Y') {  //yaw is offset 1
    offset = SERVO_YAW; 
  }
  
  //clamp servo angle between the configured min/max
  angle = constrain(angle, servoRanges[offset][0], servoRanges[offset][1]);
  
  //now calculate the timing 
  microseconds = map(angle,  servoRanges[offset][0], servoRanges[offset][1], servoTimings[offset][0],  servoTimings[offset][1]);

  servos[offset].writeMicroseconds(microseconds);
  
  //update position estimation data
  servoPositionsOld[offset] = estimateServoPosition(offset); 
  servoPositions[offset] = angle; 
  timeOfLastServoChange[offset] = millis();
}

/**
 * Record a sensor sweep reading to the circular buffer
 *
 * @return void
 */
void recordSensorSweepReading(){
  int blocking_range = getRangeNow();
  float milliseconds = (float)(micros() - sweep_time)/1000.0f;
  float angle = milliseconds * SWEEP_DEGREES_PER_MILLISECOND + SWEEP_DEGREE_OFFSET; //This is why we learn y = mx + b
   
  //if we're traveling backwards, the degrees are backwards
  angle = angle * sweep_direction; //this fixes that
  
  setSensorData(angle, blocking_range);  //record the sensor data in the circular output buffer
}

/**
 * A state machine that:
 * - 0: Start the servos moving into forward facing position
 * - 1: First moves the servos into a forward facing position
 * - 2: Then sweep left pre-blanking, after SWEEP_PRE_BLANK
 * - 3: Then sweep left recording data, after SWEEP_POST_BLANK goto 2
 * - 4: Then sweep left post-blanking, then flip direction, reset clock, and goto 1
 *
 * @return void
 */
void processSensorSweep(){
  switch(sweep_stage){
    case 0: //reset to start position from wherever, then get into case 1,2,3 loop
      setServo('P', 0);  //pitch level with horizon
      setServo('Y', servoRanges[SERVO_YAW][0]);  //move the low side
      sweep_direction = -1;  //initially moving towards negative
      sweep_time = micros();
      sweep_stage = 1;
      break;
      
    case 1: //wait for move to be done
      if (micros() - sweep_time >= SWEEP_END) {  //give it the max time to arrive
        sweep_time = micros();
        sweep_direction *= -1; //reverse sweep direction
        sweep_stage = 2;
        setServo('Y', servoRanges[SERVO_YAW][(1+sweep_direction)/2]); //move to the other side
      }
      break;
      
    case 2: //skip the spin up
      if (micros() - sweep_time >= SWEEP_PRE_BLANK) {
          sweep_stage = 3;
      }
      break;
      
    case 3: //RECORD while moving at a constant angular velocity
      recordSensorSweepReading();
      if (micros() - sweep_time >= SWEEP_POST_BLANK) {
          sweep_stage = 1;
      }
      break;
  }
}

void endSensorSweep(){
  sweep_stage = 0; 
  setServo('P', 0);  //pitch level with horizon
  setServo('Y', 0);  //pitch level with horizon
}
