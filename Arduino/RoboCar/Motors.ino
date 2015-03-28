/**
 * Servo handling functions for control and position estimation
 *
 * @author Joseph Duchesne
 */
 

// We need the soft pwm lib since we're not using hardware PWM pins for the motors
#include <SoftPWM.h>
#include <SoftPWM_timer.h>

//pinout:
static int motorEnable = 13; // LED connected to digital pin 13

//if this goes low, one of the motor drivers is having a bad day
static int motorError = 2;

//used by the rotary encoders to know if we should increment/decrement
static int motorDirections[2] = {0, 0}; //stopped, +1 for forward, -1 for back

//wheelEncoders: back left, back right
static int wheelEncoders[2] = {A2,A3};
boolean wheelEncoderValues[2] = {false, false}; //logic levels for wheel encoders
long int wheelEncoderCounters[2] = {0,0};  //number of counts for wheel encoders

unsigned long lastEncoderRead = 0;  //miliseconds since arduino start
//the motors can spin at about 1.3rev/sec. With 20 state changes/rev 
//lets poll this at ~5x that which should incidentally debounce things
static int timeBetweenEncoderReads = 8; //ms, 0.008s


//Front left, back left, back right, front right {forward, reverse}
int motors[4][2] = {{3,4},{5,6},{7,8},{9,12}};

/**
 * Init motor encoders and output
 * 
 * @return void
 */
void motor_setup() {
  //disable motor drivers
  pinMode(motorEnable, OUTPUT);      // sets the digital pin as output
  SoftPWMBegin();
  
  //set motors
  int i,j;
  for (i=0;i<4;i++) {
    for (j=0;j<2;j++) {
      SoftPWMSet(motors[i][j], 0);
    }
  }
  
  //configure wheel encoders for reading
  for(i=0;i<2;i++){
    pinMode(wheelEncoders[i], INPUT);
  } 
}


/**
 * Set the motors ('L' or 'R') to a speed (-100 through 100)
 *
 * @param char dir   The side 'L' for left, 'R' for right
 * @param int  speed The speed as a percentage from -100 to 100
 *
 * @return void
 */
void setMotors(char side, int speed) {
  int offset = 0; //left motors are 0, 1
  if (side=='R') {  //right motors are 2,3
    offset = 2; 
  }
  
  Serial.println("#M");
  /*
  Serial.print("#Motors driving: ");
  Serial.print(side);
  Serial.print(" side: ");
  Serial.print(speed);
  Serial.println("speed");*/
  
  if (speed>0) { //forward
    SoftPWMSetPercent(motors[offset+0][0], speed);
    SoftPWMSetPercent(motors[offset+0][1], 0);
    SoftPWMSetPercent(motors[offset+1][0], speed);
    SoftPWMSetPercent(motors[offset+1][1], 0);
    motorDirections[offset/2] = 1;  
  } else if (speed <0) {  //reverse
    SoftPWMSetPercent(motors[offset+0][0], 0);
    SoftPWMSetPercent(motors[offset+0][1], -1*speed);
    SoftPWMSetPercent(motors[offset+1][0], 0);
    SoftPWMSetPercent(motors[offset+1][1], -1*speed);
    motorDirections[offset/2] = -1;
  } else { //stop
     //direction left alone
     SoftPWMSetPercent(motors[offset+0][0], 0);
     SoftPWMSetPercent(motors[offset+0][1], 0);
     SoftPWMSetPercent(motors[offset+1][0], 0);
     SoftPWMSetPercent(motors[offset+1][1], 0);
  }
}

/**
 * Enable/disable motor drivers
 * 
 * @param boolean value HIGH or LOW to enable/disable motors
 *
 * @return void
 */
void setMotorEnable(boolean value) {
  wheelEncoderCounters[1] = wheelEncoderCounters[0] = 0;  //clear the encoder counters, we've restarted
   
  digitalWrite(motorEnable, value); 
}

/**
 * Read rotary encoder values to try to estimate distance traveled
 *
 * @return void
 */
void readRotaryEncoders() {
  //output sensor value every timeBetweenOutputs
  if (millis()-lastEncoderRead < timeBetweenEncoderReads) {
    return; 
  } 
  lastEncoderRead = millis();
  
  //iterate left and right encoders
  for (int i=0; i<2; i++) {
    //if the value has changed
    if (digitalRead(wheelEncoders[i])!=wheelEncoderValues[i]) {
      wheelEncoderValues[i] = digitalRead(wheelEncoders[i]);
      wheelEncoderCounters[i] += motorDirections[i]; //increment/decrement as needed
    }
  }
}

/**
 * Return the number of wheel encoder ticks for a wheel
 * 
 * @param int wheel left (0) or right(1) wheels
 * 
 * @return long int Ticks each wheel has undergone
 */
long int getWheelCounter(int wheel) {
  return wheelEncoderCounters[wheel];
}

