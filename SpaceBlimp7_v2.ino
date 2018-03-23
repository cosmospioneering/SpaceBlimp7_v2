/*
  Dual Control Moment Gyro - High Altitude Balloon Attitude Control via Arduino
  
  Read more:  
  GIT:       https://github.com/cosmospioneering/
  Web:       http://www.cosmospioneering.com
  Author:    Andrew Reed - Founder - Cosmos Pioneering
  License:   GNU General Public License, version 3 (GPL-3.0)

  Please Support Cosmos Pioneering!
  We are striving to simplify the complexities of space
  through easy to understand code and electronics.
*/

//How to program an Arduino Spacecraft
//Comments by Author



//Make sure we enable a WatchDog timer,
//so that when our microcontroller locks up for any reason
//the WatchDog will reset it and it will continue executing
//
#include <avr/wdt.h>


//Since our spacecraft will be functioning autonomously,
//we must write our program so that it reflects how
//our spacecraft's "mission" will execute, step by step.
//Not only will this simplify our code, but it will also
//aid us in knowing what our spacecraft will do and under
//what conditions it will do them in.
//
//One simple way of doing this is to implement a programming
//moethodology called "state machine".
//
//This is a cleaner way of dealing with what happens
//during different phases of our craft's mission
//
//
#include "Fsm.h"


//Create the states, including the enter, run & exit procedures for each state.
//Some states may not have some procedures (enter & run, but no exit)
State state_startup(&start_up_on_enter, NULL, &start_up_on_exit);
State state_idle(&idle_on_enter, &idle_on_state, &idle_on_exit);
State state_ascend(&ascend_on_enter, &ascend_on_state, &ascend_on_exit);
State state_stabilize(&stabilize_on_enter, &stabilize_on_state, NULL);
State state_slowpan(&slowpan_on_enter, &slowpan_on_state, &slowpan_on_exit);
State state_descend(&descend_on_enter, &descend_on_state, NULL);
State state_touchdown(&touchdown_on_enter, &touchdown_on_state, NULL);
State state_failsafe(&failsafe_on_enter, &failsafe_on_state, NULL);
//Create the state machine object (our spacecraft!)
Fsm spacecraft(&state_startup);


//Due to how the FSM library works, we need to assign unique ID numbers
//to each state machine transition.  We create these so we can trigger them
//when we programmatically detect the conditions under which we must trigger
//a state change.
//For example, in our mission, when our spacecraft is powered on and eventually
//enters the IDLE state, it reads the altimeter to determine if the spacecraft
//is ascending.  If it is ascending, then the spacecraft's state needs to change
//to ASCEND.  This is done by "triggering" the IDLE-->ASCEND transition.
//
//NOTE:  Transitions are created during the arduino setup procedure
//
//State Machine Transition Event IDs
#define STARTUP_EVENT  0
#define IDLE_EVENT  1
#define ASCEND_EVENT  2
#define STABILIZING_EVENT  3
#define STABLE_ASCEND_EVENT  4
#define SLOWPAN_EVENT  5
#define STABILIZED_SLOWPAN_EVENT  6
#define UNEXPECTED_DECEND_EVENT  8
#define STABILIZED_DECEND_EVENT  9
#define SLOWPAN_STABILIZE_EVENT 10
#define SLOWPAN_DECEND_EVENT  11
#define TOUCHDOWN_EVENT  12
#define FAILSAFE_EVENT  13
int currentState;
int lastStateChgMinute;




#include <string.h>



//In order to talk to many of the sensors and systems on board our spacecraft
//we chose to use a standard data bus called I2C to communicate with our systems.
//I2C uses a library called "Wire"
//
//
//----------------------------------------------
//    Wire Library   - I2C Bus Communications
//----------------------------------------------
#include <Wire.h>
//
//    This is required for...
//    Magnetometer (digital compass)
//    Accellerometer/Gyrometer
//    Altitude, Temperature & Air Pressure sensor
//    PWM Controller (CMG interface)
//    EEProm data storage module
//    Voltage/Current power sensors



//To keep track of when events occur, we need a clock and a place to store sensor data
//
//----------------------------------------------
//    Real Time Clock Library   -  Date/Time
//----------------------------------------------
#include <RTClib.h>
RTC_DS3231 rtc;
//char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
DateTime now;
int lastMinute;
int lastSenseSecond;
int lastListenSecond;
int lastHour;
//
//
//----------------------------------------------
// Arduino EEProm Library   - For Compass Cals
//----------------------------------------------
#include <EEPROM.h>
// EEPROM addresses for persisted data
const int CompassXOffsetAddress = 0;
const int CompassYOffsetAddress = 8;
const int CompassZOffsetAddress = 16;
const int EEPromPageAddress = 24; //goes to 28 (4 byte unsigned long)
//
//----------------------------------------------
//    EEProm Library   - For Logging
//----------------------------------------------
#include <PString.h>
//
uint32_t CurrentPageStartAddress = 0; //set to zero at the start of each cycle
char EEPROMBuffer[16]; //this buffer contains a string of ascii
#define EEPROM_ADDR  0x50 // I2C Buss address of FT24C128A 128K EEPROM
#define EEPromPageSize  64 //64 bytes for the FT24c128A I am using
#define EEPromBufferSize  16 //64 bytes for the FT24c128A I am using
PString str(EEPROMBuffer, sizeof(EEPROMBuffer));
//The FT24C128A series are 131,072 bits of serial Electrical Erasable & Programmable Read Only Memory
//64byte Page Size
//16,384 x8 (128K)
//-40C to 85C
//
//FT24C128A have 256 pages of 64bytes each.
//Random word addressing will require a 14 bit data word address
//128K bytes, used by 16 byte blocks
//  = 8,192 16 byte blocks!
//131,072 - 16
#define MaxStartAddress  16384



/*
const byte LogStateChange = B00000001;
const byte LogPower = B00000010;
const byte LogSensor = B00000100;
const byte LogPowerAndSensor = B00000110;
const byte LogEventAndPowerAndSensor = B00000111;
*/
#define LogStateChange  B00000001
#define LogPower  B00000010
#define LogSensor  B00000100
#define LogGuidance  B00001000
#define LogPowerAndSensor  B00000110
#define LogGuidanceAndPowerAndSensor  B00001110
#define LogGuidanceAndSensor  B00001010
#define LogEventAndPowerAndSensor  B00000111
#define LogCMGPosCal  B10000000
#define LogCMGNegCal  B01000000



//This is to read the voltage and current for the batteries
//
//------------------------------------------------
//    INA219 Power Sensor Library   -  Power Data
//------------------------------------------------
#include <Adafruit_INA219.h>
//Main Board
Adafruit_INA219 ina219 = Adafruit_INA219(0x45); //0x45??
//PWM Ch0 & Ch1
Adafruit_INA219 ina2191 = Adafruit_INA219(0x41);
//PWM Ch2 & Ch3
Adafruit_INA219 ina2192 = Adafruit_INA219(0x40);
float busvoltage = 0;
float current_mA = 0;
float busvoltage1 = 0;
float current_mA1 = 0;
float busvoltage2 = 0;
float current_mA2 = 0;




//This is to read the alt/temp/press data
//
//------------------------------------------------
//    BMP180 Sensor Library   -  Env. Data
//------------------------------------------------
//#include <Adafruit_BMP085_U.h>
#include <Adafruit_Sensor.h> 
#include <Adafruit_BMP280.h>
//
//Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
Adafruit_BMP280 bme; // I2C
float temperature;
float pressure;
float altitude;
uint8_t trend;
int ema_a = 0.06;
int ema_ema = 0;
int ema = 0;
int altDEMA;
int altLastDEMA;
int minDEMA;
int maxDEMA;
boolean ascendingFlag = false;
boolean decendingFlag = false;
boolean floatingFlag = false;
boolean launchedFlag = false;
boolean touchdownFlag = false;
#define BMP180_MAX_ALTITUDE  3000   //
#define PEAK_ESTIMATED_ALTITUDE  27500   //
#define SLOWPAN_ACTIVE_ALTITUDE  15000   //Jetstream stops around 11km
#define LAUNCH_DETECT_ALTITUDE  250   //Strasburg, VA is 578', or 176m
#define MIN_ASCENT_RATE  2  // 5m/sec = 300m/min
#define FLOAT_RATE  1  // 50m/min = ~1m/s
#define MIN_DECENT_RATE  5  //  7m/sec = 420m/min
int ascentRate; // meters/sec
/*
    Total payload size of 2.1 kg, Kaymont-1500 gm balloon, ascent rate of 5 m/s
    target burst of 33000 m/ 108k ft., target descecnt of 7m/s
*/
int altSimulator;
uint16_t launchDetectedAltitude;
uint16_t launchDetectedMinute;
uint16_t launchDetectedSeconds;
uint16_t launchTenMinutePoint;
uint16_t firstMinuteFinalAltitude = 0;
uint8_t firstMinutePerSecondAscentRate;
uint16_t lastAltitudeSampleSecond;
uint16_t lastAltitudeSample;
uint16_t firstTenMinutesFinalAltitude;
uint8_t firstTenMinutePerSecondAscentRate;
boolean pauseAltSensorReading = false;




//This is to read the magnetometer
//
//------------------------------------------------
//    HCM5883L Sensor Library   -  Guidance Data
//------------------------------------------------
//Compass interface & calibration code
#include "compass.h"

//To control our spacecraft's attitude, and to output visual user queues via on-board LEDs
//we'll need to activate the 16 channel PWM controller
//
//------------------------------------------------------
//    PWM Controller  - For CMG control & visual output
//------------------------------------------------------
#include <Adafruit_PWMServoDriver.h>
//Create the PWM Servo driver object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x78);
//
//
//
//CMG Motor variables
/*
const uint16_t MOTORZERO = 0; // this is the 'minimum' pulse length count (out of 4096)
const uint16_t MOTORREADY = 200; // this is the 'minimum' pulse length count (out of 4096)
const uint16_t MOTORSTART = 290; // was 275...  this is the 'minimum' pulse length count (out of 4096)
const uint16_t MOTORIDLE = 300; // this is the 'minimum' pulse length count (out of 4096)
const uint16_t MOTORMAX = 340; // this is the 'maximum' pulse length count (out of 4096)
const uint16_t TIMEDOMAIN = 150; // this is the
*/

#define MOTORZERO 0
#define MOTORREADY 200
#define MOTORSTART 290
#define MOTORIDLE 300
#define MOTORMAX 340
#define TIMEDOMAIN 150

uint16_t LastMOTOR;
boolean MotorRunning = false;
boolean cmgActive = false; 
uint16_t restartMotorCount = 0; 
uint16_t nextMotorKickSecond = 0;

//
//
//
//CMG Servo variables
// Depending on your servo make, the pulse width min and max may vary, you
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
/*
#define SERVOMIN  120 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  460 // this is the 'maximum' pulse length count (out of 4096)
uint8_t servochannelnum = 1;
double ServoOffset;  //Find the zero center of the servo offset.  We want the positional output to reflect turning...
int posCenter = 280;    // variable to store the servo position
//int posLeft = 460;    // variable to store the servo position
//int posRight = 120;    // variable to store the servo position
int posLeft = 380;    // variable to store the servo position
int posRight = 160;    // variable to store the servo position
*/
#define posLeft  445
#define posRight 85
#define posCenter 240
uint8_t posDelay = 15;    // variable to store the servo position
uint16_t posLast = 280;    // variable to store the servo position
uint16_t resetServoCount = 0; 
#define CMG1Offset  5
#define CMG3Offset  30



//
//Attitude Control Variables
double errTargetHeading;
double targHeading = 90;
int8_t valDirection = 0;
int8_t dwellCount = 0;
int8_t dwlTotalCount = 0;
int8_t dwellTest = 5; // this is minimum number of times the maneuver test must pass before a position is considered "hit"
float heading = 0;
float turnRate = 0;
boolean valDirection2 = false;
boolean dirPassN = false;
boolean dirPassS = false;
boolean dirPassE = false;
boolean dirPassW = false;
double baseFoundHeading = -1;   //This is the base station heading value...  -1 is the unassigned default
#define searchHeadingIncrement  30 // This is how much we will increment the search angle every XX seconds (listens)
#define maxListensBeforeHeadingChange  100 // 1/10th of a second for each maneuver... 1 second for each increment
int8_t listenCount = 0;
double findTargetHeading = 0;   //This is the target heading value...  -1 is the unassigned default
boolean findTarget = false;
//
//
//
//PID Loop settings
float intgError;
float lastError;
float dervError;
//Proportional gain (the bigger the number the harder the controller pushes)
#define propGAIN  .28
//
//Integral gain  (the SMALLER the number (except for 0, which turns it off,)  the more quickly the controller reacts to load changes, but the greater the risk of oscillations.)
#define intGAIN  0
//
//Derivative gain  (the bigger the number  the more the controller dampens oscillations (to the point where performance can be hindered))
#define dervGAIN  1














//#include "I2Cdev.h"
//#include <MPU6050.h>
//#include "MPU6050_6Axis_MotionApps20.h"
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
//MPU6050 mpu(0x69);

int16_t preTorqueDriveTurnRate = 0;  //The value we are interested in!
int16_t postTorqueDriveTurnRate = 0;  //The value we are interested in!
int16_t GyroTurnRate = 0;  //The value we are interested in!
boolean activeStabilize = false;  //When we detect excessive spin!
//the below setting is how much spin is required before we
//continuously apply CMG logic
#define MAX_ACCEPTIBLE_SPIN  24   //12 degrees per second, or 2 RPM
//the below setting is how much does spin have to be reduced to before we are done
#define TARGET_ACCEPTIBLE_SPIN  6  //6 degrees per second, or 1 RPM
#define MIN_STABILIZED_COUNT  0
#define MAX_GYRO_THRESHOLD  200
uint8_t stabilizedCount;
uint8_t lastStabilizerSecond;
int gyro_offset = -3;
int16_t GyroZ;
int accel_offset = 0;
float AccelZ;
float smoothedAccelZ;
float AccelAscentRate;
unsigned long lastAscentMillis = 0;





/*
//Load spin rates from smallest to largest
//then, load the associated Gimbal values
int16_t PosSpinRates[] = {0,1,2,3,4,5,6,7,8};
byte PosGimbalAdvDelay[] = {0,1,2,3,4,5,6,7,8};
byte PosGimbalRtrnDelay[] = {0,1,2,3,4,5,6,7,8};
int16_t NegSpinRates[] = {0,1,2,3,4,5,6,7,8};
byte NegGimbalAdvDelay[] = {0,1,2,3,4,5,6,7,8};
byte NegGimbalRtrnDelay[] = {0,1,2,3,4,5,6,7,8};
byte dlyGimbalAdv;
byte dlyGimbalRtn;
*/
byte idxGimbalAngle;
int16_t storedSpinRate = 0;
















//To communicate our spacecraft's status to anyone around it,
//we'll need to activate/assign the speaker hardware
//
//----------------------------------------------
//    Piezo Speaker  - For Audio Output
//----------------------------------------------
const int speakerPin = 4;
unsigned long lastSoundMillis = 0;
//
//
//To communicate our spacecraft's status to anyone around it,
//we'll need to activate/assign the speaker hardware
//
//----------------------------------------------
//    Simple LED  - For Simple Visual Feedback
//----------------------------------------------
#define simpleLEDPin  13
int ledState = LOW;
unsigned long lastBlinkMillis = 0;
//
//
//
//Other advanced LED display variables
int LEDindex = 0;
int LEDState = 0;
int patternCount = 0;   //sizeof(patterns) / 2;
int pingpoingPoint = 0;
int mirIndex = 0;
byte LEDOutput = 0;
byte LEDOutHold = 0;
byte LEDSetup = 0;
byte LEDOutTemp = 0;
int ShowLEDCount = 0;
byte StoredSetup = 0;






void setup() {
  Serial.begin(57600);

  //Initialize the software
  //setVariables();
  //Setup the hardware!
  setPins();

  //Initialize the hardware!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //Start the I2C interface
  Wire.begin();

  
  LoadParameters();  //Load parameters from EEProm
  dumpEEprom();
  //wipeEEprom();

  //bleep(1, 20, 3500, -150, 5, 0); //one long descending chirp
  //bleep(1, 2, 3500, -500, 5, 0);  //two quick descending chirp
  //bleep(1, 4, 3500, -500, 5, 0);  //four ultra-quick descending chirp
  //bleep(10, 20, 1000, 50, 1, 0);  //negative buzz
  //bleep(1, 10, 3500, -300, 3, 0);  //one medium descending chirp
  //bleep(1, 2, 3500, -2000, 10, 0);  //one medium-light descending chirp
  //bleep(1, 70, 800, 10, 5, 0);  //slow alert sound
  //bleep(5, 20, 1000, 50, 1, 0);  //negative buzz
  bleep(3, 100, 800, 10, 5, 0);  //alert sound
  //
  //bleep(3, 50, 1000, 30, 2, 0);  //slow alert sound
  delay(500);
  //
  //bleep(byte numBleeps, byte numTones, word bgnFreq, int stpFreq, byte milDelay, long repeatDelay)





  //Have a way to confirm I2C bus hardware
  // i2cScan();



  //
  //bleep(byte numBleeps, byte numTones, word bgnFreq, int stpFreq, byte milDelay, long repeatDelay)
  //
  bleep(1, 25, 500, 80, 3, 0);
  //Initialize the clock
  initRTC();
  //Initialize the Power sensors
  initMeter();
  bleep(1, 30, 2500, -66, 3, 0);  //slow alert sound
  delay(250);
  //
  bleep(2, 25, 500, 80, 3, 0);
  //Initialize the MPU6050
  intMPU();
  //Initialize the Magnetometer
  initCompass();
  //Initialize the Env Sensor
  initBMP();
  bleep(2, 30, 2500, -66, 3, 0);  //slow alert sound
  delay(250);

  bleep(3, 25, 500, 80, 3, 0);
  //Initialize the PWM module
  initPWM();
  delay(250);
  //calibrateServos(); 
  resetServos();  //center the servo pair
  //Initialize the CMG ESCs
  coldStartESCs();

  //calibrateESCs();

  //runCMGOutputTest();
  //runCMGOutputTest_2();
  //runCMGOutputTest_3();
  //runCMGOutputTest_3a();
  //runCMGOutputTest_3b();
  //runCMGOutputTest_4();


  //Setup the state transitions
  setup_state_transitions();
  bleep(3, 30, 2500, -66, 3, 0);  //slow alert sound



  //Enable the WatchDog Timer!
  //
  //We can't use it this trip because the bootloader doesn't disable
  //the watchdog timer on restart, and instead forces a shorter timeframe!
  //wdt_enable(WDTO_8S);  // options: WDTO_1S, WDTO_2S, WDTO_4S, WDTO_8S

}



void loop() {


  //Execute the State Machine
  spacecraft.run_machine();


  //Give the WatchDog a kick, before it bites us!
  //wdt_reset(); // confirm to watchdog timer that all is well
}












