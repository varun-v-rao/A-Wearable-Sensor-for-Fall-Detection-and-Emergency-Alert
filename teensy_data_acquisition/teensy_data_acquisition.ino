//This code must be uploaded to the Teensy device powering the IMU and HC-05 Bluetooth module


/*  ********************************************* 
 *  SparkFun_ADXL345_Example
 *  Triple Axis Accelerometer Breakout - ADXL345 
 *  Hook Up Guide Example 
 *  
 *  Utilizing Sparkfun's ADXL345 Library
 *  Bildr ADXL345 source file modified to support 
 *  both I2C and SPI Communication
 *  
 *  E.Robert @ SparkFun Electronics
 *  Created: Jul 13, 2016
 *  Updated: Sep 06, 2016
 *  
 *  Development Environment Specifics:
 *  Arduino 1.6.11
 *  
 *  Hardware Specifications:
 *  SparkFun ADXL345
 *  Arduino Uno
 *  *********************************************/
#define USE_ARDUINO_INTERRUPTS true    // Set-up low-level interrupts for most acurate BPM math.

#include <SparkFun_ADXL345.h>         // SparkFun ADXL345 Library
#include <PulseSensorPlayground.h>     // Includes the PulseSensorPlayground Library.   
#include <SoftwareSerial.h>
#include <SPI.h> // SPI library



//IMU

#define POWER_CTL_Register    0x2D // B101101
#define DATA_FORMAT_Register  0x31 // B110001
#define DATAX0_Register       0x32 // B110010

#define SIX_BYTES 6

int16_t accelX;
int16_t accelY;
int16_t accelZ;
byte buffer[6];
////////////////


//  Variables
const int PulseWire = 14;       // PulseSensor PURPLE WIRE connected to ANALOG PIN 0
//const int LED = LED_BULITIN;          // The on-board Arduino LED, close to PIN 13.
int Threshold = 550;           // Determine which Signal to "count as a beat" and which to ignore.
                               // Use the "Gettting Started Project" to fine-tune Threshold Value beyond default setting.
                               // Otherwise leave the default "550" value. 
unsigned long timer = 0;
String message;
int myBPM = 0;
//end

SoftwareSerial btSerial(0, 1); // RX, TX
PulseSensorPlayground pulseSensor;  // Creates an instance of the PulseSensorPlayground object called "pulseSensor"





/*********** COMMUNICATION SELECTION ***********/
/*    Comment Out The One You Are Not Using    */
ADXL345 adxl = ADXL345(10);           // USE FOR SPI COMMUNICATION, ADXL345(CS_PIN);
//ADXL345 adxl = ADXL345();             // USE FOR I2C COMMUNICATION

/****************** INTERRUPT ******************/
/*      Uncomment If Attaching Interrupt       */
//int interruptPin = 2;                 // Setup pin 2 to be the interrupt pin (for most Arduino Boards)


/******************** SETUP ********************/
/*          Configure ADXL345 Settings         */
void setup(){
  btSerial.begin(38400);
  Serial.begin(9600);                 // Start the serial terminal
  //Serial.println("SparkFun ADXL345 Accelerometer Hook Up Guide Example");
  Serial.println();

  SPI.begin();
  TurnOnADXL345(10);   
  
  
  adxl.powerOn();                     // Power on the ADXL345

  adxl.setRangeSetting(16);           // Give the range settings
                                      // Accepted values are 2g, 4g, 8g or 16g
                                      // Higher Values = Wider Measurement Range
                                      // Lower Values = Greater Sensitivity

  adxl.setSpiBit(0);                  // Configure the device to be in 4 wire SPI mode when set to '0' or 3 wire SPI mode when set to 1
                                      // Default: Set to 1
                                      // SPI pins on the ATMega328: 11, 12 and 13 as reference in SPI Library 
   
  adxl.setActivityXYZ(1, 0, 0);       // Set to activate movement detection in the axes "adxl.setActivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setActivityThreshold(75);      // 62.5mg per increment   // Set activity   // Inactivity thresholds (0-255)
 
  adxl.setInactivityXYZ(1, 0, 0);     // Set to detect inactivity in all the axes "adxl.setInactivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setInactivityThreshold(75);    // 62.5mg per increment   // Set inactivity // Inactivity thresholds (0-255)
  adxl.setTimeInactivity(10);         // How many seconds of no activity is inactive?

  adxl.setTapDetectionOnXYZ(0, 0, 1); // Detect taps in the directions turned ON "adxl.setTapDetectionOnX(X, Y, Z);" (1 == ON, 0 == OFF)
 
  // Set values for what is considered a TAP and what is a DOUBLE TAP (0-255)
  adxl.setTapThreshold(50);           // 62.5 mg per increment
  adxl.setTapDuration(15);            // 625 μs per increment
  adxl.setDoubleTapLatency(80);       // 1.25 ms per increment
  adxl.setDoubleTapWindow(200);       // 1.25 ms per increment
 
  // Set values for what is considered FREE FALL (0-255)
  adxl.setFreeFallThreshold(7);       // (5 - 9) recommended - 62.5mg per increment
  adxl.setFreeFallDuration(30);       // (20 - 70) recommended - 5ms per increment
 
  // Setting all interupts to take place on INT1 pin
  //adxl.setImportantInterruptMapping(1, 1, 1, 1, 1);     // Sets "adxl.setEveryInterruptMapping(single tap, double tap, free fall, activity, inactivity);" 
                                                        // Accepts only 1 or 2 values for pins INT1 and INT2. This chooses the pin on the ADXL345 to use for Interrupts.
                                                        // This library may have a problem using INT2 pin. Default to INT1 pin.
  
  // Turn on Interrupts for each mode (1 == ON, 0 == OFF)
  adxl.InactivityINT(1);
  adxl.ActivityINT(1);
  adxl.FreeFallINT(1);
  adxl.doubleTapINT(1);
  adxl.singleTapINT(1);
  
//attachInterrupt(digitalPinToInterrupt(interruptPin), ADXL_ISR, RISING);   // Attach Interrupt


//PPG
  //Configure the PulseSensor object, by assigning our variables to it. 
  pulseSensor.analogInput(PulseWire);   
  //pulseSensor.blinkOnPulse(LED);       //auto-magically blink Arduino's LED with heartbeat.
  pulseSensor.setThreshold(Threshold);   

  //Double-check the "pulseSensor" object was created and "began" seeing a signal. 
   if (pulseSensor.begin()) {
    Serial.println("We created a pulseSensor Object!");  //This prints one time at Arduino power-up,  or on Arduino reset.  
  }


}





/****************** MAIN CODE ******************/
/*     Accelerometer Readings and Interrupt    */
void loop(){
  
    // Accelerometer Readings
    int x,y,z;   
    adxl.readAccel(&x, &y, &z);   // Read the accelerometer values and store them in variables declared above x,y,z
  
    // Output Results to Serial
    /* UNCOMMENT TO VIEW X Y Z ACCELEROMETER VALUES */  
  
  if (pulseSensor.sawStartOfBeat()) {            // Constantly test to see if "a beat happened".
    myBPM = pulseSensor.getBeatsPerMinute();  // Calls function on our pulseSensor object that returns BPM as an "int".
                                                 // "myBPM" hold this BPM value now. 
    //Serial.println("HeartBeat Detected!"); // If test is "true", print a message "a heartbeat happened".
    //Serial.print("BPM: ");                        // Print phrase "BPM: " 
    //Serial.println(myBPM);

  }
// Accelerometer Readings
  // Please note these are uncalibrated numbers
  // Read SPI device on CS pin 10
  readAccel(10, &accelX, &accelY, &accelZ);
  
  float accelMag = sqrt(accelX*accelX + accelY*accelY + accelZ*accelZ); 
  
  // Print out the accelerator readings
  // The resolution of the values is 2^10 = 1024
  // Now we have positive and negative numbers, so resolution is +/- 512
  // We have set 16g, so 16g = 512
  // So we need to multiple by 16 and divide by 512 to get units of gravity.
  Serial.print(accelX*16.0f/512,3);
  Serial.print(",");
  Serial.print(accelY*16.0f/512,3);
  Serial.print(",");
  Serial.print(accelZ*16.0f/512,3);
//  Serial.print(",");
//  Serial.print(accelMag*16.0f/512,3); 
  Serial.println(" ");
//  Serial.print("BPM: ");
//  Serial.println(myBPM);

  btSerial.print(accelX*16.0f/512,3);
  btSerial.print(",");
  btSerial.print(accelY*16.0f/512,3);
  btSerial.print(",");
  btSerial.print(accelZ*16.0f/512,3);
//  btSerial.print(",");
//  btSerial.print(accelMag*16.0f/512,3); 
  btSerial.println(" ");
//  btSerial.print("BPM: ");
//  btSerial.println(myBPM);// Print the value inside of myBPM. 
//
//  //delay(10);


//
//    
   // btSerial.print(x);
   // btSerial.print(", ");
  //  btSerial.print(y);
  //  btSerial.print(", ");
  //  btSerial.println(z); 


//    Serial.print(x);
//    Serial.print(", ");
//    Serial.print(y);
//    Serial.print(", ");
//    Serial.println(z); 


 delay(300);

  




  
  ADXL_ISR();
  // You may also choose to avoid using interrupts and simply run the functions within ADXL_ISR(); 
  //  and place it within the loop instead.  
  // This may come in handy when it doesn't matter when the action occurs. 

}

/********************* ISR *********************/
/* Look for Interrupts and Triggered Action    */
void ADXL_ISR() {
  
  // getInterruptSource clears all triggered actions after returning value
  // Do not call again until you need to recheck for triggered actions
  byte interrupts = adxl.getInterruptSource();
  
//  // Free Fall Detection
//  if(adxl.triggered(interrupts, ADXL345_FREE_FALL)){
//    Serial.println("*** FREE FALL ***");
//    //add code here to do when free fall is sensed
//  } 
  
//  // Inactivity
//  if(adxl.triggered(interrupts, ADXL345_INACTIVITY)){
//    Serial.println("*** INACTIVITY ***");
//     //add code here to do when inactivity is sensed
//  }
  
//  // Activity
//  if(adxl.triggered(interrupts, ADXL345_ACTIVITY)){
//    Serial.println("*** ACTIVITY ***"); 
//     //add code here to do when activity is sensed
//  }
//  
//  // Double Tap Detection
  if(adxl.triggered(interrupts, ADXL345_DOUBLE_TAP)){
    Serial.println("*** DOUBLE TAP ***");
    btSerial.println("*** DOUBLE TAP ***");
     //add code here to do when a 2X tap is sensed
  }
//  
//  // Tap Detection
//  if(adxl.triggered(interrupts, ADXL345_SINGLE_TAP)){
//    Serial.println("*** TAP ***");
//     //add code here to do when a tap is sensed
//  } 
}


void TurnOnADXL345(int16_t pinCS) 
{
  // We control this pin
  pinMode(pinCS, OUTPUT);

  // Don't let the pin float - so set it to a value.
  // HIGH means we are not talking to it.
  digitalWrite(pinCS, HIGH);

  // POWER_CTL_Register
  // D7
  // D6
  // D5 Link       = 0
  // D4 AUTO_SLEEP = 0
  // D3 Measure    = 1
  // D2 Sleep      = 0
  // D1 Wakeup     = 0
  // D0 Wakeup     = 0
  
  // Wakeup device
  // D1 = 0 & D0 = 0, reading frequency 8Hz
  // Puts the ADXL345 into measurement mode
  writeSPIregister(pinCS, POWER_CTL_Register, 8);

  // Set accelerator range to 2g
  setRangeSetting(pinCS);
}

void setRangeSetting(int16_t pinCS) 
{
  // Set accelerator range e.g. min range = max sensitivity
  
  // Register 0x31 - DATA FORMAT (Read/Write) page 17 of 24
  // D7 - SELF_TEST  = 0 Disables the self-test force
  // D6 - SPI        = 0 Sets the device to 4-wire SPI mode
  // D5 - INT_INVERT = 0 Sets interrupts to active HIGH
  // D4 - 0
  // D3 - FULL_RES   = 0 Sets device ro 10-bit mode, and range set to max g range
  // D2 - Justify    = 0 Set right justified mode with sign extension
  // D1 - Range
  // D0 - Range
  
  // _s = B00000000; for 2g
  // _s = B00000001; for 4g
  // _s = B00000010; for 8g
  // _s = B00000011; for 16g

  // Want 2g range, so maximum sensitivity
  byte cmd = B00000011;

  // Update the SPI register
  writeSPIregister(pinCS, DATA_FORMAT_Register, cmd);
}

void readAccel(int16_t pinCS, int16_t *x, int16_t *y, int16_t *z) 
{
  // Read six bytes from the SPI
  readSPIregister(pinCS, DATAX0_Register, SIX_BYTES, buffer);
  
  // 10 Bit resolution accuracy
  *x = (((int16_t)buffer[1]) << 8) | buffer[0];   
  *y = (((int16_t)buffer[3]) << 8) | buffer[2];
  *z = (((int16_t)buffer[5]) << 8) | buffer[4];
}

// ======== SPI commands ===========

void writeSPIregister(int16_t pinCS, byte RegisterAddress, byte cmd) 
{
  // Define SPI bus parameters
  // Max SPI clock frequency 5 MHz - page 8  of 24
  // Mode      Clock Polarity  Clock Phase   Output_Edge Data_Capture
  // SPI_MODE3   (CPOL)1        (CPHA) 1       Falling     Rising
  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE3));

  // We want to communicate with this SPI slave
  digitalWrite(pinCS, LOW);

  // This is the register we want
  SPI.transfer(RegisterAddress); 

  // This is the command/value we set
  SPI.transfer(cmd);

  // Ok all done. Stop communication with this SPI slave
  digitalWrite(pinCS, HIGH);

  // Allow others to use SPI bus
  SPI.endTransaction();
}

void readSPIregister(int16_t pinCS, byte RegisterAddress, int16_t num, byte buffer[]) 
{  
  // Need to set left bit in register address to enable Read Data
  byte readRegisterAddress = 0x80 | RegisterAddress;

  // Read Data + Address Increment
  // To read or write multiple bytes in a single transmission, the multiple-byte bit, located after the R/W bit
  // in the first byte transfer, must be set. After the register addressing and the first byte of data, each 
  // subsequent set of clock pulses (eight clock pulses) causes the ADXL345 to point to the next register for 
  // a read or write.
  if(num > 1) 
  {
    // Set bit to enable address increment 
    // 0x40 = B01000000
    readRegisterAddress = readRegisterAddress | 0x40;
  }

  // Define SPI bus parameters
  // Max SPI clock frequency 5 MHz - page 8  of 24
  // Mode      Clock Polarity  Clock Phase   Output_Edge Data_Capture
  // SPI_MODE3   (CPOL)1        (CPHA) 1       Falling     Rising
  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE3));
  
  // We want to communicate with this SPI slave
  digitalWrite(pinCS, LOW);
  
  // This is the register we want
  SPI.transfer(readRegisterAddress); 

  // Read the consecutive bytes
  for (int16_t i=0; i<num; i++)
  {
    // Receiving buffer[i] whilst transmitting 0x00
    buffer[i] = SPI.transfer(0x00);
  }

  // Ok all done. Stop communication with this SPI slave
  digitalWrite(pinCS, HIGH);

  // Allow others to use SPI bus
  SPI.endTransaction();
}
