#ifdef __AVR__
  #include <avr/power.h>
#endif
#include <Thread.h>
#include <ThreadController.h>
#include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <Adafruit_VS1053.h>
#include <SD.h>
#include <Adafruit_GPS.h>
//#include <SerialHandler.h>
#include <Wire.h>
#include <PID_v1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <MyoBridge.h>










//SoftwareSerial connection to MyoBridge
SoftwareSerial bridgeSerial(50,51);
//initialize MyoBridge object with software serial connection
MyoBridge bridge(bridgeSerial);












//#define BNO055_SAMPLERATE_DELAY_MS (50)
Adafruit_BNO055 bno = Adafruit_BNO055(55);









#define SHIELD_RESET  -1      // VS1053 reset pin (unused!)
#define SHIELD_CS     7     // VS1053 chip select pin (output)
#define SHIELD_DCS    6      // VS1053 Data/command select pin (output)
// These are common pins between breakout and shield
#define CARDCS 4     // Card chip select pin
// DREQ should be an Int pin, see http://arduino.cc/en/Reference/attachInterrupt
#define DREQ 3       // VS1053 Data request, ideally an Interrupt pin

Adafruit_VS1053_FilePlayer musicPlayer = 
// create breakout-example object!
//Adafruit_VS1053_FilePlayer(BREAKOUT_RESET, BREAKOUT_CS, BREAKOUT_DCS, DREQ, CARDCS);
// create shield-example object!
Adafruit_VS1053_FilePlayer(SHIELD_RESET, SHIELD_CS, SHIELD_DCS, DREQ, CARDCS);
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy






//#define FIST_PIN 49
//#define WAVEIN_PIN 50
//#define WAVEOUT_PIN 51
//#define FINGERSSPREAD_PIN 52
//#define DOUBLETAP_PIN 53
boolean isMyo = false;






/*
//Adafruit GPS
Adafruit_GPS GPS(&Serial2);
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  true
// this keeps track of whether we're using the interrupt
// off by default!
*/






#define MD03_1_ADDRESS 0x58 // The address of the MD03 motor controller
#define MD03_2_ADDRESS 0x59 // The address of the MD03 motor controller
#define MD03_COMMAND_REG 0x00
#define MD03_STATUS_REG 0x01
#define MD03_SPEED_REG 0x02
#define MD03_ACCELERATION_REG 0x03
#define MD03_TEMPERATURE_REG 0x04
#define MD03_MOTOR_CURRENT_REG 0x05
#define MD03_SOFTWARE_REV_REG 0x07
#define MD03_MOVE_FORWARD 0x01
#define MD03_STOP 0x00
#define MD03_MOVE_BACKWARDS 0x02








// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1
#define neoPIN            40

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      1

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, neoPIN, NEO_GRB + NEO_KHZ800);














/*************************************************************************/
const int redPin = 24;  // R petal on RGB LED module connected to digital pin 11
const int greenPin = 23;  // G petal on RGB LED module connected to digital pin 9
const int bluePin = 22;  // B petal on RGB LED module connected to digital pin 10
/**************************************************************************/   











String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
String key = "";
int delim = 4;
int nextdelim = 4;





int x1 = 0;
int x2 = 0;
int y1 = 0;
int y2 = 0;
int z = 0;
int inpLength = 0;






//Define Variables we'll be connecting to
double xSetpoint, xInput, xOutput, ySetpoint, yInput, yOutput;
//Specify the links and initial tuning parameters
double xKp=4, xKi=0, xKd=1, yKp=4, yKi=0, yKd=1;
PID xPID(&xInput, &xOutput, &xSetpoint, xKp, xKi, xKd, REVERSE);
PID yPID(&yInput, &yOutput, &ySetpoint, yKp, yKi, yKd, REVERSE);
double bnoY = 0;
double bnoZ = 0;
int Xdirection = 0;
int Ydirection = 0;
int lastout = 0;
double xlow = 0;
double ylow = 0;
double xhigh = 255;
double yhigh = 255;












void serialEvent() {
  while (Serial1.available()) {
    // get the new byte:
    char inChar = (char)Serial1.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
      break;
    }
  }
}











void driveAny(int Motor, int aSpeed, int aDirection) {
  Wire.beginTransmission(Motor); // Start communicating with the MD03
  Wire.write(MD03_SPEED_REG);            // Send the address of speed register
  Wire.write(aSpeed);                       // Set the speed
  Wire.endTransmission();
 
  Wire.beginTransmission(Motor); // Start communicating with the MD03
  Wire.write(MD03_ACCELERATION_REG);     // Send the address of the acceleration register
  Wire.write(0);                        // Set the acceleration (27 = reach new speed in 1s)
  Wire.endTransmission();
 
  Wire.beginTransmission(Motor); // Start communicating with the MD03
  Wire.write(MD03_COMMAND_REG);          // Send the address of the command register
  Wire.write(aDirection);         // Send the forward command
  Wire.endTransmission();
}












void stopAny(int Motor) {
    Wire.beginTransmission(Motor);   // Start communicating with the MD03
    Wire.write(MD03_SPEED_REG);              // Send the address of speed register
    Wire.write(MD03_STOP);                   // Set the speed to be zero
    Wire.endTransmission();
    Wire.beginTransmission(Motor); // Start communicating with the MD03
    Wire.write(MD03_COMMAND_REG);          // Send the address of the command register
    Wire.write(MD03_MOVE_FORWARD);         // Send the forward command
    Wire.endTransmission();
}













//BNO055
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/











//BNO055
void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/











//BNO055
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}











//rgb led
void color (unsigned char red, unsigned char green, unsigned char blue)     // the color generating function 
{   
          analogWrite(redPin, red);  
          analogWrite(bluePin, blue);
          analogWrite(greenPin, green);
          /*
          // Basic colors: 
          color(255, 0, 0); // turn the RGB LED red
          delay(1000); // delay for 1 second 
          color(0,255, 0); // turn the RGB LED green 
          delay(1000); // delay for 1 second 
          color(0, 0, 255); // turn the RGB LED blue 
          delay(1000); // delay for 1 second
          // Example blended colors: 
          color(255,0,0); // turn the RGB LED red 
          delay(1000); // delay for 1 second 
          color(237,109,0); // turn the RGB LED orange 
          delay(1000); // delay for 1 second 
          color(255,215,0); // turn the RGB LED yellow 
          delay(1000); // delay for 1 second 
          color(34,139,34); // turn the RGB LED green 
          delay(1000); // delay for 1 second
          color(0,0,255); // turn the RGB LED blue 
          delay(1000); // delay for 1 second
          color(0,46,90); // turn the RGB LED  indigo
          delay(1000); // delay for 1 second
          color(128,0,128); // turn the RGB LED purple 
          delay(1000); // delay for 1 second
          */
}











//declare a function to handle MYO pose data
void handlePoseData(MyoPoseData& data) {
  
  //convert pose data to MyoPose
  MyoPose pose;
  pose = (MyoPose)data.pose;

  //print the pose
  Serial.println(bridge.poseToString(pose));
  //return pose;
}









//declare a function to handle MYOIMU data
void handleIMUData(MyoIMUData& data) {
  
  //Accelerometer data is scaled from 0 to 2048. But just print it out for the Moment:
  Serial.print(data.accelerometer[0]);
  Serial.print(" ");
  Serial.print(data.accelerometer[1]);
  Serial.print(" ");
  Serial.println(data.accelerometer[2]);
}











//timers
long sonarinterval = 199; 
long pirinterval = 499; 
long remoteinterval = 59;
long imuinterval = 47;
long myointerval = 1009;
long neointerval = 3319;
long mp3interval = 7919;

unsigned long previousMillis = 0;       
unsigned long previousMillis1 = 0;        
unsigned long previousMillis2 = 0;   
unsigned long previousMillis3 = 0; 
unsigned long previousMillis4 = 0; 
unsigned long previousMillis5 = 0; 
unsigned long previousMillis6 = 0; 








//pir motion senspr
int pirledPin = 39;                // choose the pin for the LED
int pirinputPin = 38;               // choose the input pin (for PIR sensor)
int pirState = LOW;             // we start, assuming no motion detected
int pirval = 0;                    // variable for reading the pin status







/**************************************************************************
***************************SETUP*****************************************
***************************************************************************
***************************SETUP*****************************************
***************************************************************************
***************************SETUP*****************************************
***************************************************************************
***************************SETUP*****************************************
***************************************************************************
***************************SETUP*****************************************
***************************************************************************
***************************SETUP*****************************************
***************************************************************************
***************************SETUP*****************************************
***************************************************************************
***************************SETUP*****************************************
***************************************************************************
*/










 

void setup() {
   randomSeed(analogRead(0));
   Serial.begin(38400);     
   Serial1.begin(38400); 
   bridgeSerial.begin(38400);
   Wire.begin(); // Initialize the I2C bus
   pixels.begin(); // This initializes the NeoPixel library.
   
   pinMode(redPin, OUTPUT); // sets the redPin to be an output
   pinMode(greenPin, OUTPUT); // sets the greenPin to be an output
   pinMode(bluePin, OUTPUT); // sets the bluePin to be an output
   pinMode(pirledPin, OUTPUT);      // declare LED as output
   pinMode(pirinputPin, INPUT);     // declare sensor as input
  /* 
  pinMode(FIST_PIN, OUTPUT);
  pinMode(WAVEIN_PIN, OUTPUT);
  pinMode(WAVEOUT_PIN, OUTPUT);
  pinMode(FINGERSSPREAD_PIN, OUTPUT);
  pinMode(DOUBLETAP_PIN, OUTPUT);
  */
  
  
  
  
  
  
  
  
  
  
   Serial.println("Searching for Myo...");
  bridge.begin();
  Serial.println("connected!");
    //set the function that handles the IMU data
  //bridge.setIMUDataCallBack(handleIMUData);
  //tell the Myo we just want the IMU data
  //bridge.setIMUMode(IMU_MODE_SEND_DATA);
  //disable sleep mode, so we get continous data even when not synced 
  bridge.disableSleep();
  //set the function that handles pose events
  //bridge.setPoseEventCallBack(handlePoseData);
  //tell the Myo we want Pose data
 // bridge.enablePoseData();
  //make sure Myo is unlocked
  bridge.unlockMyo(); 
  
  
  
  
  
  
  
  
  
  
  
   /* GPS
   
   // 9600 NMEA is the default baud rate for MTK - some use 4800
   GPS.begin(9600);
   // You can adjust which sentences to have the module emit, below
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data for high update rates!
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // uncomment this line to turn on all the available data - for 9600 baud you'll want 1 Hz rate
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
  
  // Set the update rate
  // Note you must send both commands below to change both the output rate (how often the position
  // is written to the serial line), and the position fix rate.
  // 1 Hz update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);
  // 5 Hz update rate- for 9600 baud you'll have to set the output to RMC or RMCGGA only (see above)
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  //GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
  // 10 Hz update rate - for 9600 baud you'll have to set the output to RMC only (see above)
  // Note the position can only be updated at most 5 times a second so it will lag behind serial output.
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  //GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  //useInterrupt(true);
  
  END GPS
  */
  
  inputString.reserve(200);
  delay(500);  
   
   Serial.println("Orientation Sensor Test"); Serial.println("");
     /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  Serial.print("Sesnor OK!");
  delay(1000);

  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Optional: Display current status */
  displaySensorStatus();

  bno.setExtCrystalUse(true);
  //windowStartTime = millis();

  //initialize the variables we're linked to
  xSetpoint = 0;
  ySetpoint = 0;

  //tell the PID to range between 0 and the full window size
  xPID.SetOutputLimits(xlow, xhigh);
  xPID.SetSampleTime(50);
  //turn the PID on
  xPID.SetMode(AUTOMATIC);
  yPID.SetOutputLimits(ylow, yhigh);
  yPID.SetSampleTime(50);
  //turn the PID on
  yPID.SetMode(AUTOMATIC);
  
  Serial.println("Adafruit VS1053 Simple Test");

  if (! musicPlayer.begin()) { // initialise the music player
     Serial.println(F("Couldn't find VS1053, do you have the right pins defined?"));
     while (1);
  }
  Serial.println(F("VS1053 found"));
  
  SD.begin(CARDCS);    // initialise the SD card
  
  // Set volume for left, right channels. lower numbers == louder volume!
  musicPlayer.setVolume(30,30);

  // Timer interrupts are not suggested, better to use DREQ interrupt!
  //musicPlayer.useInterrupt(VS1053_FILEPLAYER_TIMER0_INT); // timer int

  // If DREQ is on an interrupt pin (on uno, #2 or #3) we can do background
  // audio playing
  musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT);  // DREQ int
  /*
  // Play one file, don't return until complete
  Serial.println(F("Playing track 001"));
  musicPlayer.playFullFile("track001.mp3");
  // Play another file in the background, REQUIRES interrupts!
  Serial.println(F("Playing track 002"));
  musicPlayer.startPlayingFile("track002.mp2");
  */
}

uint8_t distanceDelta = 0;
/* GPS

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

END GPS 
*/










/**************************************************************************
***************************LOOP********************************************
***************************************************************************
***************************LOOP********************************************
***************************************************************************
***************************LOOP********************************************
***************************************************************************
***************************LOOP********************************************
***************************************************************************
***************************LOOP********************************************
***************************************************************************
***************************LOOP********************************************
***************************************************************************
***************************LOOP********************************************
***************************************************************************
***************************LOOP********************************************
***************************************************************************
***************************LOOP********************************************
***************************************************************************
*/










int distsensor, lastdistsensor,track;
void loop()
{
  unsigned long sonarMillis = millis();
  unsigned long remoteMillis = millis();
  unsigned long imuMillis = millis();
  unsigned long pirMillis = millis();
  unsigned long neoMillis = millis();
  unsigned long mp3Millis = millis();
  unsigned long myoMillis = millis();
  int i;
  
  if(sonarMillis - previousMillis > sonarinterval) {
    // save the last time you took a distance reading
    previousMillis = sonarMillis;   
    distsensor = 0;
    for (i=0; i<8; i++) {
      distsensor += analogRead(0);
    }
    distsensor /= 8;
    //Serial.println(distsensor);
    /*
    //if(lastdistsensor > distsensor) Serial.println("Getting warmer"); 
    //else if(lastdistsensor < distsensor) Serial.println("Getting colder");
    if(lastdistsensor != distsensor) { 
      Serial.print(F("Sensor = ")); 
      Serial.println(distsensor); 
    }
    */
    if (distsensor <= 50) {
       color(34,139,34); // turn the RGB LED green 
       //Serial.println("// turn the RGB LED mixed green ");
    } 
    
     if (distsensor > 50) {
       color(0,255,0); // turn the RGB LED green  
       //Serial.println("// turn the RGB LED basic green ");
       distanceDelta = 1;
     } // end if (distsensor > 500)
     else if ((distsensor > 30) && (distsensor < 40)) {
       if (distanceDelta <= 1) {    
            color(255,215,0); // turn the RGB LED yellow 
            //Serial.println("// turn the RGB LED yellow ");
       } else {
         color(0, 0, 255); // turn the RGB LED blue
         //Serial.println("// turn the RGB LED blue ");
       }
       distanceDelta = 2; 
   } 
   else if ((distsensor > 10) && (distsensor <= 30)) {
     if (distanceDelta <= 2) {    
       color(237,109,0); // turn the RGB LED orange 
       //Serial.println("// turn the RGB LED orange ");
     } else {
       color(0,46,90); // turn the RGB LED  indigo 
       //Serial.println("// turn the RGB LED indigo ");
     }
     distanceDelta = 3;
   } else if (distsensor <= 10) {
     if (distanceDelta <= 3) {
        color(255, 0, 0); // turn the RGB LED red
        //Serial.println("// turn the RGB LED red ");
     } else {
       color(128,0,128); // turn the RGB LED purple
       //Serial.println("// turn the RGB LED purple ");
   }
    distanceDelta = 4;
 }
 lastdistsensor = distsensor;
  } //end if(sonarMillis - previousMillis > sonarinterval) 
  
//*******************************************************
//PIR Handler
  if(pirMillis - previousMillis3 > pirinterval) {
    // save the last time you took a distance reading
    previousMillis3 = pirMillis;   
    pirval = digitalRead(pirinputPin);  // read input value
      if (pirval == HIGH) {            // check if the input is HIGH
        digitalWrite(pirledPin, HIGH);  // turn LED ON
        if (pirState == LOW) {
          // we have just turned on
          Serial.println("Motion detected!");
          // We only want to print on the output change, not state
          pirState = HIGH;
        }
      } else {
        digitalWrite(pirledPin, LOW); // turn LED OFF
        if (pirState == HIGH){
          // we have just turned of
          Serial.println("Motion ended!");
          // We only want to print on the output change, not state
          pirState = LOW;
        }
      }
  } //end if(pirMillis - previousMillis3 > pirinterval)

//END PIR Handler
//*******************************************************

//*******************************************************
//NEOpixel Handler
if(neoMillis - previousMillis4 > neointerval) {
    // save the last time you took a distance reading
    previousMillis4 = neoMillis;   
      for(int i=0;i<NUMPIXELS;i++){
        // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
        pixels.setPixelColor(i, pixels.Color(random(0,255),random(0,255),random(0,255))); // Moderately bright green color.
        pixels.show(); // This sends the updated pixel color to the hardware.
      }
}      
//End NEOpixel
//*******************************************************

//*******************************************************
//Myo Handler
if(myoMillis - previousMillis6 > myointerval) {
    /*
    #define FIST_PIN 4
    #define WAVEIN_PIN 5
    #define WAVEOUT_PIN 6
    #define FINGERSSPREAD_PIN 7
    #define DOUBLETAP_PIN 8
    */
    previousMillis6 = myoMillis; 
        //set the function that handles the IMU data
  bridge.setIMUDataCallBack(handleIMUData);
  //tell the Myo we just want the IMU data
  bridge.setIMUMode(IMU_MODE_SEND_DATA);
    bridge.update();
  //set the function that handles pose events
  bridge.setPoseEventCallBack(handlePoseData);
  //tell the Myo we want Pose data
  bridge.enablePoseData();
    //update the connection to MyoBridge
    bridge.update();
  /*  
      myo.updatePose();
   switch ( myo.getCurrentPose() ) {
    case rest:
      digitalWrite(FIST_PIN,LOW); 
      digitalWrite(WAVEIN_PIN,LOW);
      digitalWrite(WAVEOUT_PIN,LOW);
      digitalWrite(FINGERSSPREAD_PIN,LOW);
      digitalWrite(DOUBLETAP_PIN,LOW);
      //isMyo = false;
      break;
    case fist:
      digitalWrite(FIST_PIN,HIGH);
      stopAny(MD03_1_ADDRESS);
      stopAny(MD03_2_ADDRESS);
      isMyo = false;
      break;
    case waveIn:
      digitalWrite(WAVEIN_PIN,HIGH);
      //do {
        driveAny(MD03_1_ADDRESS, 255, MD03_MOVE_FORWARD);
      //} while (!FIST_PIN);
      isMyo = true;
      break;
    case waveOut:
      digitalWrite(WAVEOUT_PIN,HIGH);
      driveAny(MD03_1_ADDRESS, 255, MD03_MOVE_BACKWARDS);
      isMyo = true;
      break;
    case fingersSpread:
      digitalWrite(FINGERSSPREAD_PIN,HIGH);
      driveAny(MD03_2_ADDRESS, 255, MD03_MOVE_FORWARD);
      isMyo = true;
      break;
    case doubleTap:
      digitalWrite(DOUBLETAP_PIN,HIGH);
      driveAny(MD03_2_ADDRESS, 255, MD03_MOVE_BACKWARDS);
      isMyo = true;
      break;
   }
   */
}      
//End Myo
//*******************************************************

//*******************************************************
//Audio Handler
if(mp3Millis - previousMillis5 > mp3interval) {
    previousMillis5 = mp3Millis;   
    track = random(1,10);
    switch(track) {
      case 1:
        musicPlayer.startPlayingFile("track001.mp3");
        Serial.println("track001.mp3");
        break;
      case 2:
        musicPlayer.startPlayingFile("track002.mp3");
        Serial.println("track002.mp3");
        break;
      case 3:
        musicPlayer.startPlayingFile("track003.mp3");
        Serial.println("track003.mp3");
        break;
      case 4:
        musicPlayer.startPlayingFile("track004.mp3");
        Serial.println("track004.mp3");
        break;
      case 5:
        musicPlayer.startPlayingFile("track005.mp3");
        Serial.println("track005.mp3");
        break;
      case 6:
        musicPlayer.startPlayingFile("track006.mp3");
        Serial.println("track006.mp3");
        break;
      case 7:
        musicPlayer.startPlayingFile("track007.mp3");
        Serial.println("track007.mp3");
        break;
      case 8:
        musicPlayer.startPlayingFile("track008.mp3");
        Serial.println("track008.mp3");
        break;
      case 9:
        musicPlayer.startPlayingFile("track009.mp3");
        Serial.println("track009.mp3");
        break;
      case 10:
        musicPlayer.startPlayingFile("track010.mp3");
        Serial.println("track010.mp3");
        break;
    }
}      
//End Audio
//*******************************************************
  
  
  if(remoteMillis - previousMillis2 > remoteinterval) {
    // save the last time you blinked the LED 
    previousMillis2 = remoteMillis;
    serialEvent(); //call the function
    if (stringComplete) {
      inpLength = inputString.length();
      key = inputString.substring(0,4);
      if(key == "GPED") {
        nextdelim = inputString.indexOf(',',delim+1);
        x1 = inputString.substring(delim+1,nextdelim).toInt() - 128;
        delim=nextdelim;
        nextdelim = inputString.indexOf(',',delim+1);
        y1 = inputString.substring(delim+1,nextdelim).toInt() - 128;
        delim=nextdelim;
        nextdelim = inputString.indexOf(',',delim+1);
        x2 = inputString.substring(delim+1,nextdelim).toInt() -128;
        delim=nextdelim;
        nextdelim = inputString.indexOf(',',delim+1);
        y2 = inputString.substring(delim+1,nextdelim).toInt() - 128;
        delim = 4;
        nextdelim = 4;
        z = inputString.substring(inputString.lastIndexOf(',')+1,inpLength).toInt();
        Serial.print("X1: ");
        Serial.print(x1);
        Serial.print("\tY1: ");
        Serial.print(y1);
        Serial.print("\tX2: ");
        Serial.print(x2);
        Serial.print("\tY2: ");
        Serial.print(y2);
        Serial.print("\tRz: ");
        Serial.print(z);
        Serial.println();
      } // end if(key == "GPED")

      inputString = "";
      stringComplete = false;
    }  // end if (stringComplete)     
    if (x1 > 2) driveAny(MD03_1_ADDRESS, x1*2+1, MD03_MOVE_FORWARD);
    if (y1 > 2) driveAny(MD03_2_ADDRESS, y1*2+1, MD03_MOVE_FORWARD);
    if (x1 < -2) driveAny(MD03_1_ADDRESS, x1*-2-1, MD03_MOVE_BACKWARDS);
    if (y1 < -2) driveAny(MD03_2_ADDRESS, y1*-2-1, MD03_MOVE_BACKWARDS);
  } // end if(remoteMillis - previousMillis2 > remoteinterval)
  
  
  if(imuMillis - previousMillis1 > imuinterval) {
    previousMillis1 = imuMillis;
    sensors_event_t event;
    bno.getEvent(&event);
    /*
    Serial.println("");
    Serial.print("X: ");
    Serial.print(event.orientation.x, 4);
    Serial.print("\tY: ");
    Serial.print(event.orientation.y, 4);
    Serial.print("\tZ: ");
    Serial.print(event.orientation.z, 4);
    displayCalStatus();
    displaySensorStatus();
    */
    bnoZ = double(event.orientation.z);
    bnoY = double(event.orientation.y);
    if (bnoZ < 0) {
      Xdirection = MD03_MOVE_BACKWARDS;
    }
    else Xdirection = MD03_MOVE_FORWARD;
    if (bnoY < 0) {
      Ydirection = MD03_MOVE_BACKWARDS;
    }
    else Ydirection = MD03_MOVE_FORWARD;    
    xInput = abs(bnoZ);
    xInput = map(xInput, 0, 90, 0, 1023);
    /*
    Serial.print("\tX: ");
    Serial.print(event.orientation.x, 4);
    Serial.print("\tY: ");
    Serial.print(event.orientation.y, 4);
    Serial.print("\tZ: ");
    Serial.print(event.orientation.z, 4);
    Serial.print("\t xInput: ");
    Serial.print(xInput);
    */
    xPID.Compute();
    /*
    Serial.print("\t xOutput: ");
    Serial.print(xOutput);
    Serial.print("\t x1: ");
    Serial.print(abs(x1));
    */
    yInput = abs(bnoY);
    yInput = map(yInput, 0, 180, 0, 1023);
    /*
    Serial.print("\t yInput: ");
    Serial.print(yInput);
    */
    yPID.Compute();
    /*
    Serial.print("\t yOutput: ");
    Serial.print(yOutput);
    Serial.print("\t y1: ");
    Serial.print(abs(y1));
    Serial.println("");
    */
    
    if (xOutput > abs(x1)*2+1) driveAny(MD03_1_ADDRESS, xOutput, Xdirection);
    if (yOutput > abs(y1)*2+1) driveAny(MD03_2_ADDRESS, yOutput, Ydirection);
    if (xOutput == 0  && abs(x1) < 2 && isMyo == false) stopAny(MD03_1_ADDRESS);
    if (yOutput == 0  && abs(y1) < 2 && isMyo == false) stopAny(MD03_2_ADDRESS);
  } //end if(imuMillis - previousMillis1 > imuinterval)
} //end of main loop
