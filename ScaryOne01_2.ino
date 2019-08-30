/*********************************************************************
Scary One point 2
8 DOF Quadruped Walker
includes 3 modes
1. Bluetooth remote control using Adafruit Bluetooth LE App
2. Walker Race using compass and distance sensors
3. Walker Race using Dead Reconing 

Design based on work by Javier Isabel Hernandes and his miniKame

Major Components:
610mah 7.4v Lipo battery  - HOBBYTOWN
3.7v Li-ion battery - ADAFRUIT
Feather M0 Bluefruit LE - ADAFRUIT
8-Channel Servo FeatherWing - ADAFRUIT
TGY-S306G-HV servo - HOBBYTOWN
HMC5883L Triple-axis Magnetometer Board - ADAFRUIT
VL53L0X Time-of-Flight Distance Sensor - POLOLU
5V, 5A Step-Down Voltage Regulator D24V50F5 - POLOLU

leg numbers looking down                 
                ^
           2 ___|__ 1
            |      |
            |      |  
            |______|
           4        3 
           

*********************************************************************/

#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
#include <SoftwareSerial.h>
#endif
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"

//compass and distance includes
#include <Wire.h>
#include <VL53L0X.h>
#include <HMC5883L.h>
VL53L0X sensor;
HMC5883L compass;



//servo includes
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);



uint8_t servonum = 0;
int routine = 0;
int prevRoutine = 0;
int currRoutine = 0;
int walkStart = 0;
int turnStart = 0;
int mode = 0;
int val = 0;
int sP = 2;   //speed lower the number the faster the speed default 5
int strR = 20; //stride Right for Dead Reconing
int strL = 20;  //stride Left for Dead Reconing
int str = 20;  //stride/2
int tH;  //target heading
int rT; //range to target
int sR = 40;   //stride Right
int sL = 40;   //stride Left
const int buttonPin = 9;     // the number of the pushbutton pin
const int ledPin =  6;      // the number of the LED pin


/*=========================================================================
    APPLICATION SETTINGS

FACTORYRESET_ENABLE       
Perform a factory reset when running this sketch Enabling this will put your 
Bluefruit LE module in a 'known good' state and clear any config data set in 
previous sketches or projects, so running this at least once is a good idea.
When deploying your project, however, you will want to disable factory 
reset by setting this value to 0. enable by setting value to 1. If you are 
making changes to your Bluefruit LE device via AT commands, and those changes
aren't persisting across resets, this is the reason why.  Factory reset will
erase the non-volatile memory where config data is stored, setting it back 
to factory default values.
Some sketches that require you to bond to a central device (HID mouse, 
keyboard, etc.) won't work at all with this feature enabled since the factory 
reset will clear all of the bonding data stored on the chip, meaning the
central device won't be able to reconnect.

    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "SPI"
/*=========================================================================*/


/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];


/*
**************************************************************************
SETUP
**************************************************************************
*/
void setup(void)
{
  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit App Controller Example"));
  Serial.println(F("-----------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));
  if ( !ble.begin(VERBOSE_MODE) ){
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );
  if ( FACTORYRESET_ENABLE ){
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
    delay(500);
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) ){
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));

  //Servo Setup
  Serial.println("16 channel Servo test!");
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
                       // using digital servo, look at using higher freq.

  yield();


  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);

//COMPASS AND DISTANCE SENSORS SETUPS
  Wire.begin();
  // Initialioze Compass
  sensor.init();
  sensor.setTimeout(500);
  sensor.startContinuous(); 

  // Initialize HMC5883L
  Serial.println("Initialize HMC5883L");
  while (!compass.begin())
  {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }
  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);
  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);
  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);
  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(-11, -262);
//COMPASS AND DISTANCE SENSOR SETUP END



  
  // Start position
  middle();
  selectMode();
}



/*
************************************************************************
FORWARD
BLUETOOTH
************************************************************************
*/
void forward(void)
{
  for (uint16_t i = 0; i < 25; i++) {
    pwm.setPWM(0, 0, 345+i);  //move 1 DWN
    pwm.setPWM(1, 0, 235+i);  //move 2 UP
    pwm.setPWM(2, 0, 230+i);  //move 3 UP
    pwm.setPWM(3, 0, 330+i);  //move 4 DWN
    delay(sP);
  } 
  for (uint16_t i = 0; i < 40; i++) {
    pwm.setPWM(4, 0, 300+i);  //move 1 REV
    pwm.setPWM(5, 0, 302+i);  //move 2 FWD
    pwm.setPWM(6, 0, 302-i);  //move 3 FWD
    pwm.setPWM(7, 0, 263-i);  //move 4 REV
    delay(sP);
  } 
  for (uint16_t i = 0; i < 25; i++) {
    pwm.setPWM(0, 0, 370-i);  //move 1 UP
    pwm.setPWM(1, 0, 260-i);  //move 2 DWN
    pwm.setPWM(2, 0, 255-i);  //move 3 DWN
    pwm.setPWM(3, 0, 355-i);  //move 4 UP   
    delay(sP);
  } 
  for (uint16_t i = 0; i< 40; i++) {
    pwm.setPWM(4, 0, 340-i);  //move 1 FWD
    pwm.setPWM(5, 0, 342-i);  //move 2 REV
    pwm.setPWM(6, 0, 262+i);  //move 3 REV
    pwm.setPWM(7, 0, 223+i);  //move 4 FWD
    delay(sP);
  } 
}
/*
************************************************************************
REVERSE
BLUETOOTH
************************************************************************
*/
void reverse(void)
{
for (uint16_t i = 0; i< 40; i++) {
    pwm.setPWM(4, 0, 300+i);  //move 1 REV
    pwm.setPWM(5, 0, 302+i);  //move 2 FWD
    pwm.setPWM(6, 0, 302-i);  //move 3 FWD
    pwm.setPWM(7, 0, 263-i);  //move 4 REV
    delay(sP);
  }   
  for (uint16_t i = 0; i < 25; i++) {
    pwm.setPWM(0, 0, 345+i);  //move 1 DWN
    pwm.setPWM(1, 0, 235+i);  //move 2 UP
    pwm.setPWM(2, 0, 230+i);  //move 3 UP
    pwm.setPWM(3, 0, 330+i);  //move 4 DWN
    delay(sP);
  } 
for (uint16_t i = 0; i< 40; i++) {
    pwm.setPWM(4, 0, 340-i);  //move 1 FWD
    pwm.setPWM(5, 0, 342-i);  //move 2 REV
    pwm.setPWM(6, 0, 262+i);  //move 3 REV
    pwm.setPWM(7, 0, 223+i);  //move 4 FWD
    delay(sP);
  }   
  for (uint16_t i = 0; i < 25; i++) {
    pwm.setPWM(0, 0, 370-i);  //move 1 UP
    pwm.setPWM(1, 0, 260-i);  //move 2 DWN
    pwm.setPWM(2, 0, 255-i);  //move 3 DWN
    pwm.setPWM(3, 0, 355-i);  //move 4 UP   
    delay(sP);
  }   
}
/*
************************************************************************
LEFT TURN
BLUETOOTH
************************************************************************
*/
void leftTurn(void)
{
for (uint16_t i = 0; i< 40; i++) {
    pwm.setPWM(4, 0, 300+i);  //move 1 REV
    pwm.setPWM(5, 0, 342-i);  //move 2 REV
    pwm.setPWM(6, 0, 302-i);  //move 3 FWD
    pwm.setPWM(7, 0, 223+i);  //move 4 FWD
    delay(sP);
  }  
  for (uint16_t i = 0; i < 25; i++) {
    pwm.setPWM(0, 0, 370-i);  //move 1 UP
    pwm.setPWM(1, 0, 265-i);  //move 2 DWN
    pwm.setPWM(2, 0, 260-i);  //move 3 DWN
    pwm.setPWM(3, 0, 355-i);  //move 4 UP   
    delay(sP);
  }
for (uint16_t i = 0; i< 40; i++) {
    pwm.setPWM(4, 0, 340-i);  //move 1 FWD
    pwm.setPWM(5, 0, 302+i);  //move 2 FWD
    pwm.setPWM(6, 0, 262+i);  //move 3 REV
    pwm.setPWM(7, 0, 263-i);  //move 4 REV
    delay(sP);
  } 
 for (uint16_t i = 0; i < 25; i++) {
    pwm.setPWM(0, 0, 345+i);  //move 1 DWN
    pwm.setPWM(1, 0, 240+i);  //move 2 UP
    pwm.setPWM(2, 0, 235+i);  //move 3 UP
    pwm.setPWM(3, 0, 330+i);  //move 4 DWN
    delay(sP);
  } 
}
/*
************************************************************************
RIGHT TURN
BLUETOOTH
************************************************************************
*/
void rightTurn(void){
  for (uint16_t i = 0; i < 25; i++) {
    pwm.setPWM(0, 0, 370-i);  //move 1 UP
    pwm.setPWM(1, 0, 265-i);  //move 2 DWN
    pwm.setPWM(2, 0, 260-i);  //move 3 DWN
    pwm.setPWM(3, 0, 355-i);  //move 4 UP   
    delay(sP);
  } 
  for (uint16_t i = 0; i <  40; i++) {
    pwm.setPWM(4, 0, 300+i);  //move 1 REV
    pwm.setPWM(5, 0, 342-i);  //move 2 REV
    pwm.setPWM(6, 0, 302-i);  //move 3 FWD
    pwm.setPWM(7, 0, 223+i);  //move 4 FWD
    delay(sP);
  } 
  for (uint16_t i = 0; i < 25; i++) {
    pwm.setPWM(0, 0, 345+i);  //move 1 DWN
    pwm.setPWM(1, 0, 240+i);  //move 2 UP
    pwm.setPWM(2, 0, 235+i);  //move 3 UP
    pwm.setPWM(3, 0, 330+i);  //move 4 DWN
    delay(sP);
  } 
  for (uint16_t i = 0; i < 40; i++) {
    pwm.setPWM(4, 0, 340-i);  //move 1 FWD
    pwm.setPWM(5, 0, 302+i);  //move 2 FWD
    pwm.setPWM(6, 0, 262+i);  //move 3 REV
    pwm.setPWM(7, 0, 263-i);  //move 4 REV
    delay(sP);
  }   
}



/*
************************************************************************
SHIFT TO TURN ROUTINE
BLUETOOTH
************************************************************************
*/
void shiftForTurn(void){
  for (uint16_t i = 0; i < 40; i++) {
    pwm.setPWM(7, 0, 263-i);  //move 4 REV
    delay(sP);
  }
  for (uint16_t i = 0; i < 25; i++) {
    pwm.setPWM(0, 0, 345+i);  //move 1 DWN
    pwm.setPWM(1, 0, 240+i);  //move 2 UP
    pwm.setPWM(2, 0, 235+i);  //move 3 UP
    pwm.setPWM(3, 0, 330+i);  //move 4 DWN
    delay(sP);
  }
  for (uint16_t i = 0; i < 40; i++) {
    pwm.setPWM(5, 0, 302+i);  //move 2 FWD
    delay(sP);
  }
}
/*
************************************************************************
SHIFT TO STRAIGHT ROUTINE
BlUETOOTH
************************************************************************
*/
void shiftForStraight(void){
  for (uint16_t i = 0; i < 40; i++) {
    pwm.setPWM(5, 0, 342-i);  //move 2 REV
    delay(sP);
  }
  for (uint16_t i = 0; i < 25; i++) {
    pwm.setPWM(0, 0, 370-i);  //move 1 UP
    pwm.setPWM(1, 0, 265-i);  //move 2 DWN
    pwm.setPWM(2, 0, 260-i);  //move 3 DWN
    pwm.setPWM(3, 0, 355-i);  //move 4 UP
    delay(sP);
  }
  for (uint16_t i = 0; i < 40; i++) {
    pwm.setPWM(7, 0, 223+i);  //move 4 FWD
    delay(sP);
  }
}

/*
************************************************************************
HALT
BLUETOOTH
************************************************************************
*/    
void halt(void){
  pwm.setPWM(0, 0, 328); //#1  UP
  pwm.setPWM(1, 0, 244); //#2  DWN
  pwm.setPWM(2, 0, 250); //#3  DWN
  pwm.setPWM(3, 0, 330); //#4  UP
  pwm.setPWM(4, 0, 287); //#1  FWD
  pwm.setPWM(5, 0, 288); //#2  REV
  pwm.setPWM(6, 0, 312); //#3  REV
  pwm.setPWM(7, 0, 278); //#4  FWD
} 

/*
************************************************************************
MIDDLE POSITION
************************************************************************
*/    
void middle(void){
  pwm.setPWM(0, 0, 370); //#1  DWN  -isUP
  pwm.setPWM(1, 0, 235); //#2  DWN  +isUP
  pwm.setPWM(2, 0, 230); //#3  DWN  +isUP
  pwm.setPWM(3, 0, 355); //#4  DWN  -isUP
  pwm.setPWM(4, 0, 320); //#1    MID  
  pwm.setPWM(5, 0, 322); //#2    MID
  pwm.setPWM(6, 0, 282); //#3    MID
  pwm.setPWM(7, 0, 243); //#4    MID
}

/*
************************************************************************
START TO WALK ROUTINE
WALKER
************************************************************************
*/
void startToWalk(void){

for (uint16_t i = 1; i < 26; i++) {
    pwm.setPWM(1, 0, 235+i);  //move 2 UP
    pwm.setPWM(2, 0, 230+i);  //move 3 UP 
  delay(sP);
  }
}

/*
************************************************************************
FORWARD TO TURN SHIFT
************************************************************************
*/
void forwardToTurn(void){

for (uint16_t i = 0; i< 40; i++) {
    pwm.setPWM(7, 0, 263-i);  //move 4 REV
        delay(sP);
  }  

for (uint16_t i = 0; i < 25; i++) {
  pwm.setPWM(0, 0, 345+i);  //move 1 DWN
  pwm.setPWM(3, 0, 330+i);  //move 4 DWN
  delay(sP);
  }


  for (uint16_t i = 0; i < 25; i++) {
    pwm.setPWM(1, 0, 235+i);  //move 2 UP
    delay(sP);
  }
for (uint16_t i = 0; i< 20; i++) {
    pwm.setPWM(5, 0, 302+i);  //move 2 FWD 
    delay(sP);
  }  
}

/*
************************************************************************
RIGHT TURN 2
************************************************************************
*/
void rightTurn2(void){
  for (uint16_t i = 0; i < 25; i++) {
    pwm.setPWM(0, 0, 370-i);  //move 1 UP
    pwm.setPWM(3, 0, 355-i);  //move 4 UP   
    delay(sP);
  } 
for (uint16_t i = 0; i< str*2; i++) {
    pwm.setPWM(4, 0, 320-str+i);  //move 1 REV
    pwm.setPWM(5, 0, 322+str-i);  //move 2 REV
    pwm.setPWM(6, 0, 285+str-i);  //move 3 FWD
    pwm.setPWM(7, 0, 243-str+i);  //move 4 FWD
    delay(sP);
  } 
  for (uint16_t i = 0; i < 25; i++) {
    pwm.setPWM(0, 0, 328+i);  //move 1 DWN
    pwm.setPWM(1, 0, 244+i);  //move 2 UP
    pwm.setPWM(2, 0, 250+i);  //move 3 UP
    pwm.setPWM(3, 0, 330+i);  //move 4 DWN
    delay(sP);
  } 
for (uint16_t i = 0; i< str*2; i++) {
    pwm.setPWM(4, 0, 320+str-i);  //move 1 FWD  340-320-300
    pwm.setPWM(5, 0, 322-str+i);  //move 2 FWD  302-322-342
    pwm.setPWM(6, 0, 285-str+i);  //move 3 REV  265-285-305
    pwm.setPWM(7, 0, 243+str-i);  //move 4 REV  263-243-223
    delay(sP);
  }  
for (uint16_t i = 0; i < 25; i++) {
    pwm.setPWM(1, 0, 269-i);  //move 2 DWN
    pwm.setPWM(2, 0, 275-i);  //move 3 DWN
    delay(sP);
  } 
   
}

/*
************************************************************************
TURN TO FORWARD SHIFT
************************************************************************

void turnToForwardWhatEver(void){


for (uint16_t i = 0; i < 25; i++) {
    pwm.setPWM(1, 0, 235+i);  //move 2 UP
    delay(sP);
  }  
for (uint16_t i = 0; i < 40; i++) {
    pwm.setPWM(5, 0, 342-i);  //move 2 REV
    delay(sP);
  }
for (uint16_t i = 0; i < 25; i++) {
    pwm.setPWM(1, 0, 269-i);  //move 2 DWN
    delay(sP);
  }
for (uint16_t i = 0; i < 25; i++) {
    pwm.setPWM(3, 0, 355-i);  //move 4 UP
    delay(sP);
  }
for (uint16_t i = 0; i< 40; i++) {
    pwm.setPWM(7, 0, 238+i);  //move 4 FWD 
        delay(sP);
  }
for (uint16_t i = 0; i< 25; i++) {
    pwm.setPWM(0, 0, 353-i);  //move 1 UP  
        delay(sP);
  }  
}

*/

/*
************************************************************************
FORWARD
DEAD RECONING
************************************************************************
*/
void forwardDR(void)
{
    
    int i;
    
   for (uint16_t i = 0; i < 20; i++) {     //2nd half of routine to move total 20 ticks
    if (i <= (sR/2)){                                 // rev   mid   fwd   
    pwm.setPWM(4, 0, 320+i);  //move 1 REV  right side  340 <- 320 -  300 
    pwm.setPWM(6, 0, 282-i);  //move 3 FWD  right side  302 -  282 -> 262 
    }
    if (i <= (sL/2)){ 
    pwm.setPWM(5, 0, 322+i);  //move 2 FWD  left  side  302 -  322 -> 342
    pwm.setPWM(7, 0, 243-i);  //move 4 REV  left  side  223 <- 243 -  263
    }
    delay(sP);
 }
 
  for (uint16_t i = 0; i < 25; i++) { //     dwn   up
    pwm.setPWM(1, 0, 260-i);  //move 2 DWN   240 - 265
    pwm.setPWM(2, 0, 255-i);  //move 3 DWN   235 - 260
    delay(sP);
  }
  for (uint16_t i = 0; i < 25; i++) {  
    pwm.setPWM(0, 0, 370-i);  //move 1 UP    370 - 345
    pwm.setPWM(3, 0, 355-i);  //move 4 UP    355 - 330
    delay(sP);
  } 

 for (uint16_t i = 0; i < 40; i++) {
    if (i <= sR){                                          // rev   mid   fwd   
    pwm.setPWM(4, 0, (320+(sR/2)) -i);  //move 1 FWD          340 - 320 - 300    
    pwm.setPWM(6, 0, (282-(sR/2)) +i);  //move 3 REV          302 - 282 - 262
   }
    if (i <= sL){                                      
    pwm.setPWM(5, 0, (322+(sL/2)) -i);  //move 2 REV          301 - 322 - 305
    pwm.setPWM(7, 0, (243-(sL/2)) +i);  //move 4 FWD          223 - 243 - 263
    }
    delay(sP);
 }

 for (uint16_t i = 0; i < 25; i++) {//     dwn   up
    pwm.setPWM(0, 0, 345+i);  //move 1 DWN
    pwm.setPWM(3, 0, 330+i);  //move 4 DWN 
    delay(sP);
    }
 for (uint16_t i = 0; i < 25; i++) { 
    pwm.setPWM(1, 0, 235+i);  //move 2 UP
    pwm.setPWM(2, 0, 230+i);  //move 3 UP
    delay(sP);
  } 

  for (uint16_t i = 0; i < 20; i++) {  //move 1 and 4 rev 1st half
     if (i <= (sR/2)){                                            // rev   mid   fwd  
       pwm.setPWM(4, 0, (320-(sR/2)) +i);  //move 1 REV              340 - 320 - 300
       pwm.setPWM(6, 0, (282+(sR/2)) -i);  //move 3 FWD              302 - 282 - 262
     }
     if (i <= (sL/2)){
       pwm.setPWM(5, 0, (322-(sL/2)) +i);  //move 2 FWD              302 - 322 - 342
       pwm.setPWM(7, 0, (243+(sL/2)) -i);  //move 4 REV              223 - 243 - 263
     }
    delay(sP);
 }
}


/*
************************************************************************
SHIFT TO TURN
DEAD RECKONING
************************************************************************
*/

void shiftToTurnDR(void){
  for (uint16_t i = 0; i < 20; i++) {
    pwm.setPWM(5, 0, 322+i);  //move 2 FWD
    pwm.setPWM(6, 0, 282+i);  //move 3 REV
    delay(sP);
  }
  for (uint16_t i = 0; i < 25; i++) {
    pwm.setPWM(1, 0, 260-i);  //move 2 DWN
    pwm.setPWM(2, 0, 255-i);  //move 3 DWN
    delay(sP);
  }
  for (uint16_t i = 0; i < 25; i++) {
    pwm.setPWM(0, 0, 370-i);  //move 1 UP
    pwm.setPWM(3, 0, 355-i);  //move 4 UP
    delay(sP);
  }
  for (uint16_t i = 0; i < 20; i++) {
    pwm.setPWM(4, 0, 320-i);  //move 1 FWD
    pwm.setPWM(7, 0, 243-i);  //move 4 REV
    delay(sP);
  }
  for (uint16_t i = 0; i < 25; i++) {
    pwm.setPWM(0, 0, 345+i);  //move 1 DWN
    pwm.setPWM(3, 0, 330+i);  //move 4 DWN
    delay(sP);
  }
}
/*
************************************************************************
RIGHT TURN
DEAD RECONING
************************************************************************
*/
void rightTurnDR(void){
  for (uint16_t i = 0; i < 25; i++) {
    pwm.setPWM(0, 0, 370-i);  //move 1 UP
    pwm.setPWM(3, 0, 355-i);  //move 4 UP   
    delay(sP);
  } 
for (uint16_t i = 0; i< str*2; i++) {
    pwm.setPWM(4, 0, 320-str+i);  //move 1 REV
    pwm.setPWM(5, 0, 322+str-i);  //move 2 REV
    pwm.setPWM(6, 0, 285+str-i);  //move 3 FWD
    pwm.setPWM(7, 0, 243-str+i);  //move 4 FWD
    delay(sP);
  } 
  for (uint16_t i = 0; i < 25; i++) {
    pwm.setPWM(0, 0, 345+i);  //move 1 DWN
    pwm.setPWM(1, 0, 235+i);  //move 2 UP
    pwm.setPWM(2, 0, 230+i);  //move 3 UP
    pwm.setPWM(3, 0, 330+i);  //move 4 DWN
    delay(sP);
  } 
for (uint16_t i = 0; i< str*2; i++) {
    pwm.setPWM(4, 0, 320+str-i);  //move 1 FWD  340-320-300
    pwm.setPWM(5, 0, 322-str+i);  //move 2 FWD  302-322-342
    pwm.setPWM(6, 0, 285-str+i);  //move 3 REV  265-285-305
    pwm.setPWM(7, 0, 243+str-i);  //move 4 REV  263-243-223
    delay(sP);
  }  
for (uint16_t i = 0; i < 25; i++) {
    pwm.setPWM(1, 0, 260-i);  //move 2 DWN
    pwm.setPWM(2, 0, 255-i);  //move 3 DWN
    delay(sP);
  } 
   
}
/*
************************************************************************
SHIFT TO FORWARD
DEAD RECKONING
************************************************************************
*/

void shiftToForwardDR(void){

    for (uint16_t i = 0; i < 25; i++) {
    pwm.setPWM(0, 0, 370-i);  //move 1 UP
    pwm.setPWM(3, 0, 355-i);  //move 4 UP
    delay(sP);
  }
  for (uint16_t i = 0; i < 20; i++) {
    pwm.setPWM(4, 0, 300+i);  //move 1 REV
    pwm.setPWM(7, 0, 223+i);  //move 4 FWD
    delay(sP);
  }
  for (uint16_t i = 0; i < 25; i++) {
    pwm.setPWM(0, 0, 345+i);  //move 1 DWN
    pwm.setPWM(3, 0, 330+i);  //move 4 DWN
    delay(sP);
  }
  for (uint16_t i = 0; i < 25; i++) {
    pwm.setPWM(1, 0, 235+i);  //move 2 UP
    pwm.setPWM(2, 0, 230+i);  //move 3 UP
    delay(sP);
  }
  for (uint16_t i = 0; i < 25; i++) {
    pwm.setPWM(5, 0, 342-i);  //move 2 REV
    pwm.setPWM(6, 0, 302-i);  //move 3 FWD
    delay(sP);
  }
  
  
}



/*
************************************************************************
DEAD RECKONING navigation
read compass and output number of degrees to Target Heading
************************************************************************
*/
void deadReckoning(void){

startToWalk();

for (uint16_t a = 0; a< 65; a++){
  forwardDR();
  }
  
shiftToTurnDR();
  
for (uint16_t a = 0; a< 9; a++){
  rightTurnDR();
  }
shiftToForwardDR();

  
for (uint16_t a = 0; a< 65; a++){
  forwardDR();
  }
}



/*
************************************************************************
SHIFT TO TURN ROUTINE
WALKER
************************************************************************
*/
void shiftToTurn(void){
  for (uint16_t i = 0; i < 20; i++) {
    pwm.setPWM(5, 0, 322+i);  //move 2 FWD
    pwm.setPWM(6, 0, 282+i);  //move 3 REV
    delay(sP);
  }
  for (uint16_t i = 0; i < 25; i++) {
    pwm.setPWM(1, 0, 260-i);  //move 2 DWN
    pwm.setPWM(2, 0, 255-i);  //move 3 DWN
    delay(sP);
  }
  for (uint16_t i = 0; i < 25; i++) {
    pwm.setPWM(0, 0, 370-i);  //move 1 UP
    pwm.setPWM(3, 0, 355-i);  //move 4 UP
    delay(sP);
  }
  for (uint16_t i = 0; i < 20; i++) {
    pwm.setPWM(4, 0, 320-i);  //move 1 FWD
    pwm.setPWM(7, 0, 243-i);  //move 4 REV
    delay(sP);
  }
  for (uint16_t i = 0; i < 25; i++) {
    pwm.setPWM(0, 0, 345+i);  //move 1 DWN
    pwm.setPWM(3, 0, 330+i);  //move 4 DWN
    delay(sP);
  }
}


/*
************************************************************************
TURN 180 DEGREE ROUTINE
WALKER
************************************************************************
*/
void turn180(void){
  int hE;
  int i;
  hE = headingError(i);

  while (abs(hE) > 10){

  for (uint16_t i = 0; i < 25; i++) {
    pwm.setPWM(0, 0, 370-i);  //move 1 UP
    pwm.setPWM(3, 0, 355-i);  //move 4 UP
    delay(sP);
  }
  for (uint16_t i = 0; i < 40; i++) {
    pwm.setPWM(4, 0, 300+i);  //move 1 REV
    pwm.setPWM(5, 0, 342-i);  //move 2 REV
    pwm.setPWM(6, 0, 302-i);  //move 3 FWD
    pwm.setPWM(7, 0, 223+i);  //move 4 FWD
    delay(sP);
  }
  for (uint16_t i = 0; i < 25; i++) {
    pwm.setPWM(0, 0, 345+i);  //move 1 DWN
    pwm.setPWM(3, 0, 330+i);  //move 4 DWN
    delay(sP);
  }
  for (uint16_t i = 0; i < 25; i++) {
    pwm.setPWM(1, 0, 235+i);  //move 2 UP
    pwm.setPWM(2, 0, 230+i);  //move 3 UP
    delay(sP);
  }

for (uint16_t i = 0; i < 40; i++) {
    pwm.setPWM(4, 0, 340-i);  //move 1 FWD
    pwm.setPWM(5, 0, 302+i);  //move 2 FWD
    pwm.setPWM(6, 0, 262+i);  //move 3 REV
    pwm.setPWM(7, 0, 263-i);  //move 4 REV
    delay(sP);

}
  for (uint16_t i = 0; i < 25; i++) {
    pwm.setPWM(1, 0, 260-i);  //move 2 DWN
    pwm.setPWM(2, 0, 255-i);  //move 3 DWN
    delay(sP);
  }
   hE = headingError(i);
  }
}



/*
************************************************************************
SHIFT TO STRAIGHT ROUTINE
WALKER
************************************************************************
*/
void shiftToStraight(void){
 for (uint16_t i = 0; i < 25; i++) {
    pwm.setPWM(0, 0, 370-i);  //move 1 UP
    pwm.setPWM(3, 0, 355-i);  //move 4 UP
    delay(sP);
  }
  for (uint16_t i = 0; i < 20; i++) {
    pwm.setPWM(4, 0, 300+i);  //move 1 REV
    pwm.setPWM(7, 0, 223+i);  //move 4 FWD
    delay(sP);
  }
   for (uint16_t i = 0; i < 25; i++) {
    pwm.setPWM(0, 0, 345+i);  //move 1 DWN
    pwm.setPWM(3, 0, 330+i);  //move 4 DWN
    delay(sP);
  }
  for (uint16_t i = 0; i < 25; i++) {
    pwm.setPWM(1, 0, 235+i);  //move 2 UP
    pwm.setPWM(2, 0, 230+i);  //move 3 UP
    delay(sP);
  }
  for (uint16_t i = 0; i < 20; i++) {
    pwm.setPWM(5, 0, 342-i);  //move 2 REV
    pwm.setPWM(6, 0, 302-i);  //move 3 FWD
    delay(sP);
  }
}




/*
************************************************************************
TARGET HEADING
read compass and output number of degrees to Target Heading
************************************************************************
*/
int targetHeading(int tH){
Vector norm = compass.readNormalize();

  // Calculate target heading
  float heading = atan2(norm.YAxis, norm.XAxis);
 // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (15.0 + (43.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;

// Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
   tH = heading * 180/M_PI;
  Serial.print("TargetHeading1 (degrees): "); Serial.println(tH);
  return tH;

}


/*
************************************************************************
ADJUST HEADING
read compass and output number of degrees to correct heading
negitive number to turn left and positvie number to turn right
************************************************************************
*/
int headingError(int hE){

  Vector norm = compass.readNormalize();
  // Calculate heading 

  float heading = atan2(norm.YAxis, norm.XAxis);

  float declinationAngle = (15.0 + (43.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;

// Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert to degrees

  int cH = heading * 180/M_PI; 
  
  Serial.print("CurrentHeading (degrees): "); Serial.println(cH);

  Serial.print("TargetHeading2 (degrees): "); Serial.println(tH);

  if(cH < tH) {
    cH +=360;
    }
   hE = cH - tH;
  if(hE < 180){
    hE *= -1 ;  //turn left hE degrees
 //   Serial.print("heading Error (degrees): "); Serial.println(hE);
  }
  else{
    hE = 360-hE;  //turn right hE degrees
 //   Serial.print("heading Error (degrees): "); Serial.println(hE);
  }
  
  hE = constrain(hE,-20,20);

  return hE;
}

/*
************************************************************************
WALKER RACE
************************************************************************
*/
void walkerRace(void){
  int hE,rT,sR,sL;  //heading Error , Range to Target, stride Right, stride Left, target Heading
  int i,j;
  
  hE = headingError(i);

 if (walkStart == 0){
    startToWalk();
    tH = targetHeading(i);
    walkStart = 1;
    Serial.print("target heading5: "); Serial.println(tH); 
  }

 
  
Serial.print("heading Error1 (degrees): "); Serial.println(hE);

//Serial.print(sensor.readRangeContinuousMillimeters());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  rT = sensor.readRangeContinuousMillimeters();  //Range to Target
  rT = rT/10;
Serial.print("range to target (cm): "); Serial.println(rT);

  if (hE < 0){
     (sR = 40) && (sL = 40 - (-2 * hE));  
     }
  if (hE > 0){
     (sL = 40) && (sR = 40 - (2 * hE));
     }
   if (hE == 0){
  (sL = 40) && (sR = 40 );
 }
  Serial.print("STride Left: "); Serial.println(sL); 
  Serial.print("STride Right: "); Serial.println(sR); 

if (turnStart == 0){
    if (rT < 20){
      tH = tH + 180;
      if (tH > 360){
        tH = tH - 360;
      }
      turnStart = 1;
    shiftToTurn();
    delay(250);
    turn180();
    delay(250);
    shiftToStraight();
    delay(250);
      
    }
    delay(10);
 }

 for (uint16_t i = 0; i < 20; i++) {     //2nd half of routine to move total 20 ticks
    if (i <= (sR/2)){                                // rev   mid   fwd   
    pwm.setPWM(4, 0, 320+i);  //move 1 REV  right side  340 - 320 - 300 
    pwm.setPWM(6, 0, 282-i);  //move 3 FWD  right side  302 - 282 - 262 
    }
    if (i <= (sL/2)){ 
    pwm.setPWM(5, 0, 322+i);  //move 2 FWD  left  side  302 - 322 - 342
    pwm.setPWM(7, 0, 243-i);  //move 4 REV  left  side  223 - 243 - 263
    }
    delay(sP);
 }
 
  for (uint16_t i = 0; i < 25; i++) { //     dwn   up
    pwm.setPWM(1, 0, 260-i);  //move 2 DWN   240 - 265
    pwm.setPWM(2, 0, 255-i);  //move 3 DWN   235 - 260
    delay(sP);
  }
  for (uint16_t i = 0; i < 25; i++) {  
    pwm.setPWM(0, 0, 370-i);  //move 1 UP    370 - 345
    pwm.setPWM(3, 0, 355-i);  //move 4 UP    355 - 330
    delay(sP);
  } 

 for (uint16_t i = 0; i < 40; i++) {
    if (i <= sR){                                          // rev   mid   fwd   
    pwm.setPWM(4, 0, (320+(sR/2)) -i);  //move 1 FWD          340 - 320 - 300    
    pwm.setPWM(6, 0, (282-(sR/2)) +i);  //move 3 REV          302 - 282 - 262
   }
    if (i <= sL){                                      
    pwm.setPWM(5, 0, (322+(sL/2)) -i);  //move 2 REV          301 - 322 - 305
    pwm.setPWM(7, 0, (243-(sL/2)) +i);  //move 4 FWD          223 - 243 - 263
    }
    delay(sP);
 }

 for (uint16_t i = 0; i < 25; i++) {//     dwn   up
    pwm.setPWM(0, 0, 345+i);  //move 1 DWN
    pwm.setPWM(3, 0, 330+i);  //move 4 DWN 
    delay(sP);
    }
 for (uint16_t i = 0; i < 25; i++) { 
    pwm.setPWM(1, 0, 235+i);  //move 2 UP
    pwm.setPWM(2, 0, 230+i);  //move 3 UP
    delay(sP);
  } 

  for (uint16_t i = 0; i < 20; i++) {  //move 1 and 4 rev 1st half
     if (i <= (sR/2)){                                            // rev   mid   fwd  
       pwm.setPWM(4, 0, (320-(sR/2)) +i);  //move 1 REV              340 - 320 - 300
       pwm.setPWM(6, 0, (282+(sR/2)) -i);  //move 3 FWD              302 - 282 - 262
     }
     if (i <= (sL/2)){
       pwm.setPWM(5, 0, (322-(sL/2)) +i);  //move 2 FWD              302 - 322 - 342
       pwm.setPWM(7, 0, (243+(sL/2)) -i);  //move 4 REV              223 - 243 - 263
     }
    delay(sP);
 }
} 

/*
************************************************************************
BLUETOOH REMOTE
************************************************************************
*/
void bluetoothRemote(void)
{
  /* Look for new data  */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  //if (len == 0)  return;

  /* Got a packet! */
   printHex(packetbuffer, len);

  // REMOTE Buttons
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    Serial.print ("Button "); Serial.print(buttnum);
    if (pressed) {
      Serial.println(" pressed");
      routine = buttnum;
    }
  }else{
    Serial.print(" released");
  }
  Serial.println(routine);
  switch(routine){
    case 1:
    halt();
    break;

    case 2:
    if (sP > 0){
    sP = sP - 1;
    digitalWrite(ledPin, HIGH);
    delay(500);
    digitalWrite(ledPin, LOW);
    routine = currRoutine;
    }
    break;

    case 4:
    if (sP < 15){
    sP = sP + 1;
    digitalWrite(ledPin, HIGH);
    delay(500);
    digitalWrite(ledPin, LOW);
    routine = currRoutine;
    }
    break;
    
    case 5:
      if ((prevRoutine == 5)||(prevRoutine == 6)||(prevRoutine == 0)){
        forward();
        currRoutine = 5;
      }else{
        shiftForStraight();  //from mmid possition to 
        prevRoutine = 5;
      }
      break;
    case 6:
      if ((prevRoutine == 5)||(prevRoutine == 6)||(prevRoutine == 0)){
        reverse();
       currRoutine = 6;
      }else{
        shiftForStraight();
        prevRoutine = 6;
      }
      break;
    case 7:
      if ((prevRoutine == 7)||(prevRoutine == 8)){
        leftTurn();
        currRoutine = 7;
      }else{
        shiftForTurn();
        prevRoutine = 7;
      }
    break;
    case 8:
      if ((prevRoutine == 7)||(prevRoutine == 8)){    
        rightTurn();
        currRoutine = 8;
      }else{
        shiftForTurn();
        prevRoutine = 8;
      }
    break;
  }
}
/*
************************************************************************
MODE SELECT ROUTINE
************************************************************************
*/
void selectMode(void){
  val = digitalRead(buttonPin);
  while (val == LOW){
    digitalWrite(ledPin, HIGH);
    delay(500);
    digitalWrite(ledPin, LOW);
    delay(500);
    val = digitalRead(buttonPin);
  }
  while (val == HIGH){
    mode++;
    Serial.println(mode);
    delay(1000);
    for (int i = 0; i < mode; i++){
      digitalWrite(ledPin, HIGH);
      delay(200);
      digitalWrite(ledPin, LOW);
      delay(200); 
    }
    delay(2000);
    val = digitalRead(buttonPin);
    }
}
/*
************************************************************************
MAIN 
************************************************************************
*/
void loop(void){
  switch(mode){
    case 1:
      bluetoothRemote();
      break;

    case 2:
      walkerRace();
      break;
      
    case 3:
      deadReckoning();
      break;  
  }
}
  

