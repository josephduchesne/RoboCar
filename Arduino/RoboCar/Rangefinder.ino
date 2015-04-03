#include <I2C.h>
#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.


/**
 * Init the LIDAR-Lite rangefinder via I2C
 *
 * @return void
 */
void rangefinder_setup() {
  I2c.begin(); // Opens & joins the irc bus as master
  delay(100); // Waits to make sure everything is powered up before sending or receiving data  
  I2c.timeOut(5); // Sets a timeout to ensure no locking up of sketch if I2C communication fails
}

/**
 * Get and return the rangefinder reading. "nice" relatively non-blocking version.
 * 
 * @return int Rangefinder reading in CM
 */
int getRange() {
  static int range = 0;
  static boolean requestMode = true;
  
  // Write 0x04 to register 0x00
  uint8_t nackack = 100; // Setup variable to hold ACK/NACK resopnses     
  
  if ( requestMode ) {  //request a reading mode
    nackack = I2c.write(LIDARLite_ADDRESS,RegisterMeasure, MeasureValue); // Write 0x04 to 0x00
    
    if (!nackack) { //we recieved an ack
       requestMode = false;
    }
  } else { //read value mode
  
    byte distanceArray[2]; // array to store distance bytes from read function
    nackack = I2c.read(LIDARLite_ADDRESS,RegisterHighLowB, 2, distanceArray); // Read 2 Bytes from LIDAR-Lite Address and store in array
    
    if (!nackack) { //we recieved an ack
      range = (distanceArray[0] << 8) + distanceArray[1];  // Shift high byte [0] 8 to the left and add low byte [1] to create 16-bit int
      requestMode = true;
    }
  }
  
  // return the last distance reading we got
  return range;
}

/**
 * Get the range right now, don't be nice about it. Blocks until reading.
 *
 * @return int cm to target
 */
int getRangeNow(){
  while (I2c.write(LIDARLite_ADDRESS,RegisterMeasure, MeasureValue) ) delay(1); // Write 0x04 to 0x00
  delay(1); //slight breathing room
  
  byte distanceArray[2]; // array to store distance bytes from read function
  while(I2c.read(LIDARLite_ADDRESS,RegisterHighLowB, 2, distanceArray)) delay(1); // Wait 1 ms to prevent overpolling; // Read 2 Bytes from LIDAR-Lite Address and store in array
  
  return (int)((distanceArray[0] << 8) + distanceArray[1]);  // Shift high byte [0] 8 to the left and add low byte [1] to create 16-bit int
}
