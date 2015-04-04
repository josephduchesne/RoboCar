/**
 * Main program
 * Handles serial I/O using a simple g-code inspired ASCII interface
 * Passes off specific motor, servo, and other sensor operations to other files.
 *
 * @author Joseph Duchesne
 */

/* --- incoming serial:  --- */
/*
 in an any (but normally this) order:
 'L-100\n' to 'L100\n' - left speed
 'R-100\n' to 'R100\n' - right speed
 'M0\n' or 'M1\n' - mode: enable/disable things. LSB: motors, LSB+1: sensor sweep
 'P-10\n' to 'P100\n'  Pitch servo degree
 'Y-45\n' to '45\n' Yaw servo degree
 '#some text' - debugging comments
*/

/* --- outgoing serial: --- */
/*
 in an any (but normally this) order:
 'L-32000\n' to 'L32000\n' the left wheel encoder ticks
 'R-32000\n' to 'R32000\n' the right wheel encoder ticks
 'P-10\n' to 'P100\n'  Pitch servo degree estimate @now
 'Y-45\n' to '45\n' Yaw servo degree estimate @now
 'D0\n' to 'D32000\n' - Rangefinder distance in millimeters
 '#some text' - debugging comments
*/

//decently fast serial
#define SERIAL_BAUD 115200
//ms to wait for more serial data
#define SERIAL_WAIT 2


//variables for controlling sensor data frequency
unsigned long lastOutput = 0;  //miliseconds since arduino start
const int timeBetweenOutputs = 100; //ms between outputting data

int range = 0;
int sensorOffset = 0;
boolean sensorSweep = false;
boolean flushData = false;
#define SENSOR_READINGS 15

// output mode
//#define OUTPUT_ASCII
#define OUTPUT_STRUCT

#ifdef OUTPUT_STRUCT
//note: these are 16 bit ints and should be int16 on PC
struct sensor_packet {
  int servo_pitch;
  int servo_yaw;
  int left_wheel_encoder;
  int right_wheel_encoder;
  int rangefinder_distance;
  int sweep[SENSOR_READINGS*2];
  char end1;
  char end2;
} __attribute__((__packed__));

struct sensor_packet sensorPacket;
#endif

/**
 * Setup function configures all pins, inits libs, resets values
 *
 * @return void
 */
void setup()
{
  motor_setup();
  
  servo_setup();
  
  rangefinder_setup();
  
  //Enable serial communication
  Serial.begin(SERIAL_BAUD); 
  //serial timeout on readBytesUntil()
  Serial.setTimeout(SERIAL_WAIT); //ms
  
  //Serial.println("#Started up");
}

/**
 * Main loop. Checks sensors and outputs readings.
 * 
 * @return void
 */
 
void loop()
{
  readRotaryEncoders(); //for motor data
  
  if (sensorSweep) {
    processSensorSweep();
  } else {
    range = getRange();
  }

  outputSensorData();
}


/**
 * Pseudo-interrupt for new serial data availability
 * Read user input, parse it, and pass that data to the appropriate functions.
 *
 * @return void
 */
void serialEvent(){
  char buff[16] = {0};
  char *data = buff+1;  //pointer offset by 1 to the buffer for the data segment of the command
  int intData;
  
  //read a command line inputs
  Serial.readBytesUntil('\n', buff, 32);
  switch (buff[0]) {
    case 'M': //mode
      intData = String(data).toInt();
      if (intData & 0b1) {  //LSB: 
        //Serial.println("#Motor command run: High");
        setMotorEnable(HIGH);
      } else {
        //Serial.println("#Motor command run: Low");
        setMotorEnable(LOW);
      }
      
      if (intData & 0b10 ) { //byte LSB+1: sensor sweep mode
        sensorSweep = true;
        
      } else {  //not in sweep mode
        sensorSweep = false;
        endSensorSweep();
      }
      break;
    case 'L': //left motor
      //intentional fallthrough to 'R'
    case 'R': //right motor
      intData = String(data).toInt();
      setMotors(buff[0], intData);
      break;
    case 'P': //pitch
      //intentional fallthrough to Yaw
    case 'Y': //yaw
      if ( sensorSweep ) break; //no manual servo movement during sensor sweep
      intData = String(data).toInt();
      setServo(buff[0], intData);
      break;
    case '#': //debug comments
      //we can ignore them
      break;
    default:
      if (buff[0]!='\0') {
        //Serial.print("#Unknown command: ");
        //Serial.println(buff);
      }
      break;
  }
}

/**
 * Tell the communication process to flush a reading
 *
 * @return  void
 */
void setFlushData(){
  flushData = true; 
}

/**
 * Output sensor data periodically via serial port
 *
 * @return void
 */
void outputSensorData(){
  //output sensor value every timeBetweenOutputs
  //or if we're full on sensor readings
  if (millis()-lastOutput < timeBetweenOutputs && sensorOffset<SENSOR_READINGS && !flushData) {
    return;
  }
  lastOutput = millis();
  flushData = false;
  
  #ifdef OUTPUT_ASCII
    //output estimated servo positions
    Serial.print('P');
    Serial.println(estimateServoPosition(0));
    Serial.print('Y');
    Serial.println(estimateServoPosition(1));
    
    //output wheel encoder data
    Serial.print('L');
    Serial.println(getWheelCounter(0));
    Serial.print('R');
    Serial.println(getWheelCounter(1));
    
    //output rangefinder distance data
    
    Serial.print('D');
    Serial.println(range);
  #endif
  
  #ifdef OUTPUT_STRUCT
    sensorPacket.servo_pitch = estimateServoPosition(0);
    sensorPacket.servo_yaw = estimateServoPosition(1);
    sensorPacket.left_wheel_encoder = getWheelCounter(0);
    sensorPacket.right_wheel_encoder = getWheelCounter(1);
    if (sensorSweep) {
      sensorPacket.rangefinder_distance = -sensorOffset;
    } else {
      sensorPacket.rangefinder_distance = range;
    }
    sensorPacket.end1 = 'E';
    sensorPacket.end2 = 'D';
    Serial.write((char *)&sensorPacket, sizeof(sensorPacket));
    
    //clear the sensor readings packet
    for (int i=0; i<SENSOR_READINGS*2;i++) {
      sensorPacket.sweep[i]=0;
    }
    sensorOffset = 0;
  #endif
  

}


/**
 * Write a sensor sweep reading to the sensor packet's data buffer
 *
 * @return void
 */
void setSensorData(float angle, int value){
  if( sensorSweep && sensorOffset<SENSOR_READINGS) {
    int angle_int = (int)(angle*100.0f);
    sensorPacket.sweep[sensorOffset*2] = angle_int;
    sensorPacket.sweep[sensorOffset*2+1] = value;
    sensorOffset++;
  }
}

