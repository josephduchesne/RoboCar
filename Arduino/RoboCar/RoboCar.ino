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
 'M0\n' or 'M1\n' - enable/disable motors (default 0)
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
const int timeBetweenOutputs = 20; //ms between outputting data

static int range = 0;

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
  
  Serial.println("#Started up");
}

/**
 * Main loop. Checks sensors and outputs readings.
 * 
 * @return void
 */
 
void loop()
{
  readRotaryEncoders(); //for motor data
  
  range = getRange();
  
  outputSensorData();
}


/**
 * Pseudo-interrupt for new serial data availability
 * Read user input, parse it, and pass that data to the appropriate functions.
 *
 * @return void
 */
void serialEvent(){
  char buff[32] = {0};
  char *data = buff+1;  //pointer offset by 1 to the buffer for the data segment of the command
  int intData;
  
  //read a command line inputs
  Serial.readBytesUntil('\n', buff, 32);
  switch (buff[0]) {
    case 'M': //motor enable
      if (data[0]=='1') {
        Serial.println("#Motor command run: High");
        setMotorEnable(HIGH);
      } else {
        Serial.println("#Motor command run: Low");
        setMotorEnable(LOW);
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
      intData = String(data).toInt();
      setServo(buff[0], intData);
      break;
    case '#': //debug comments
      //we can ignore them
      break;
    default:
      if (buff[0]!='\0') {
        Serial.print("#Unknown command: ");
        Serial.println(buff);
      }
      break;
  }
}


/**
 * Output sensor data periodically via serial port
 *
 * @return void
 */
void outputSensorData(){
  //output sensor value every timeBetweenOutputs
  if (millis()-lastOutput < timeBetweenOutputs) {
    return;
  }
  lastOutput = millis();
  
  
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

}

