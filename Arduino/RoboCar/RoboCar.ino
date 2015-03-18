// Servo lib for camera servos
#include <Servo.h>

// We need the soft pwm lib since we're not using hardware PWM pins for the motors
#include <SoftPWM.h>
#include <SoftPWM_timer.h>

//fastest serial available since
//we're polling the wheel encoders
//and can't wait around reading serial
#define SERIAL_BAUD 115200
//ms to wait for more serial data
#define SERIAL_WAIT 2

/* --- incoming serial:  --- */
//then an any (but normally this) order:
//'L-100\n' to 'L100\n' - left speed
//'R-100\n' to 'R100\n' - right speed
//'M0\n' or 'M1\n' - enable/disable motors (default 0)
//'P0\n' to 'P180\n'  Pitch servo degree byte (0-180 typically)
//'Y0\n' to 'Y180\n' Yaw servo degree (0-180 typically)

/* --- outgoing serial: --- */
//'L' $lowByte $highByte (little endian int16 of the left wheel encoder ticks)
//'R' $lowByte $highByte (little endian int16 of the right wheel encoder ticks)
//'D' $lowByte $highByte (little endian int16 of the range finder)
//'S' $byte $byte (unused sensor bits) 

//pinout:
int motorEnable = 13; // LED connected to digital pin 13

//if this goes low, one of the motor drivers is having a bad day
int motorError = 2;

//used by the rotary encoders to know if we should increment/decrement
int motorDirections[2] = {0, 0}; //stopped, +1 for forward, -1 for back

//camera servos: yaw, pitch
int cameraServos[2] = {10,11};

//wheelEncoders: back left, back right
int wheelEncoders[2] = {A4,A5};
boolean wheelEncoderValues[2] = {false, false}; //logic levels for wheel encoders
int wheelEncoderCounters[2] = {0,0};  //number of counts for wheel encoders

unsigned long lastEncoderRead = 0;  //miliseconds since arduino start
//the motors can spin at about 1.3rev/sec. With 20 state changes/rev 
//lets poll this at ~5x that which should incidentally debounce things
static int timeBetweenEncoderReads = 8; //ms, 0.008s

unsigned long lastOutput = 0;  //miliseconds since arduino start
static int timeBetweenOutputs = 2000; //ms, 2 seconds

//Front left, back left, back right, front right {forward, reverse}
int motors[4][2] = {{3,4},{5,6},{7,8},{9,12}};

void setup()
{
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
  
  
  
  //Enable serial communication
  Serial.begin(SERIAL_BAUD); 
  //serial timeout on readBytesUntil()
  Serial.setTimeout(SERIAL_WAIT); //ms
}

void loop()
{




  readRotaryEncoders();
  
  outputSensorData();
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
  
  Serial.print("#Motors driving: ");
  Serial.print(side);
  Serial.print(" side: ");
  Serial.print(speed);
  Serial.println("speed");
  
  
  if (speed>=0) { //forward
    SoftPWMSetPercent(motors[offset+0][0], speed);
    SoftPWMSetPercent(motors[offset+0][1], 0);
    SoftPWMSetPercent(motors[offset+1][0], speed);
    SoftPWMSetPercent(motors[offset+1][1], 0);
    motorDirections[offset/2] = 1;  
  } else {  //reverse
    SoftPWMSetPercent(motors[offset+0][0], 0);
    SoftPWMSetPercent(motors[offset+0][1], -1*speed);
    SoftPWMSetPercent(motors[offset+1][0], 0);
    SoftPWMSetPercent(motors[offset+1][1], -1*speed);
    motorDirections[offset/2] = -1;
  }
}


void serialEvent(){
  char buff[16] = {0};
  char *data = buff+1;  //pointer offset by 1 to the buffer for the data segment of the command
  int intData;
  
  //read a command line inputs
  Serial.readBytesUntil('\n', buff, 16);
  switch (buff[0]) {
    case 'M':
      if (data[0]=='1') {
        Serial.println("#Motor command run: High");
        digitalWrite(motorEnable, HIGH);
      } else {
        Serial.println("#Motor command run: Low");
        digitalWrite(motorEnable, LOW);
      }
      break;
    case 'L':
      //intentional fallthrough to 'R'
    case 'R':
      intData = String(data).toInt();
      setMotors(buff[0], intData);
      break;
    default:
      if (buff[0]!='\0') {
        Serial.print("#Unknown command: ");
        Serial.println(buff);
      }
      break;
  }
}

void readRotaryEncoders() {
  //output sensor value every timeBetweenOutputs
  if (millis()-lastEncoderRead < timeBetweenEncoderReads) {
    return; 
  } 
  lastEncoderRead = millis();
  
  //Serial.println("Reading");
  
  //iterate left and right encoders
  for (int i=0; i<2; i++) {
    //if the value has changed
    if (digitalRead(wheelEncoders[i])!=wheelEncoderValues[i]) {
      wheelEncoderValues[i] = digitalRead(wheelEncoders[i]);
      wheelEncoderCounters[i] += motorDirections[i]; //increment/decrement as needed
    }
  }
}


void outputSensorData(){
  //output sensor value every timeBetweenOutputs
  if (millis()-lastOutput < timeBetweenOutputs) {
    return;
  }
  lastOutput = millis();
  
  Serial.print('L');
  Serial.println(wheelEncoderCounters[0]);
  Serial.print('R');
  Serial.println(wheelEncoderCounters[1]);
  
}
  
