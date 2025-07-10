/********************************************************************************************
*                                                                                           *
*                               Zebsboards cabBlaster R2.0                                  *
*                               March 2025 copyright Zebsboards.com                         *
*                                                                                           *
*     Identifier ids: Zebsboards,20A0,41C5                                                  *
*                                                                                           *
*     Board Revision: expressif ESP32 Rev 2.014    Lolin S2 mini controller                 *
*     Manufacturer, VID & PID set in Variants\lolinS2mini\pins_arduino.h.text file          *
*     Requires patched DOF directoutput.dll to be identified                                *     
*     Configure using Pinscape Device 1 controller entry in configtoolonline                *  
*                                                                                           *                                                                                    *
********************************************************************************************/
#include "at24c256.h"
#include "PCA9685.h"
#include <Wire.h>

//int addr = 0;
at24c256 eeprom(0x50);                       // initialize the eeprom chip and address at 0x50 (A0 GND, A1 GND)

#define Ledpin LED_BUILTIN  // ESP32 S2 module onboard LED
#define nightModePin 3
int nightModeIn;

/* Setting PWM Properties/Pins/Default Vals */
const int PWMFreq = 67000; /* 67KHz */
const int PWMResolution = 8;
const int FlipperLeftCh = 0;
const int FlipperRightCh = 1;
const int ShakerCh = 2;
int fLheld;
int fRheld;
int FlipperFireVal = 255;
int FlipperHoldVal = 33;
int FlipperOffVal = 0;
byte FlipperLeftPin = 1;     // placeholder for LEFT flipper output pin
byte FlipperRightPin = 5;    // placeholder for RIGHT flipper output pin
byte ShakerPin = 8;         
int fLpos = 0;         // position in array of Left Flipper (Output 1)
int fRpos = 1;         // position in array of Right Flipper (output 2) 
int ShkrPos = 2;       // position in array of Shaker Motor  (output 3)
byte UseSolenoidPWM = 1;
PCA9685 Chip0(0x40, Wire, 1000000);  // create the namespaces for the PWM output boards (Banks 3/4)
PCA9685 Chip1(0x41, Wire, 1000000);  // Banks 5/6
PCA9685 Chip2(0x42, Wire, 1000000);  // Banks 7/8
//available pwm pins (S2) 2,4,12,13,14,16,17,18,21,32
byte directOutputs[16] = { 1, 5, 8, 4, 39, 40, 37, 38, 21, 16, 6, 9, 10, 11, 12, 13};  // will be bank 1/2, strictly ON/OFF operation
byte previousOutputState[64];
byte nightModeAssigned[64] = { 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
byte outputTimerVal[64] = { 6, 6, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
byte ReceivedSettings[130];
long outputResetTimer[64];
byte numOutputBoards = 0;
int SetupScan;

void setup() 
{
  Serial.begin(2000000);
  Wire.begin();
  delay(100);
  //eeprom.init(); 

  pinMode(Ledpin, OUTPUT);
  pinMode(nightModePin, INPUT_PULLUP);
  for (int a = 0; a < 16; a++) 
  {
    pinMode(directOutputs[a], OUTPUT);
    digitalWrite(directOutputs[a], LOW);
  }
  SetupScan = 1;
  BusScan();
  SetupScan = 0;  
  switch (numOutputBoards) {
    case 1:
      Chip0.resetDevices();         // Resets all PCA9685 devices on i2c line
      Chip0.init();                 // Initializes module using default totem-pole driver mode, and default disabled phase balancer
      Chip0.setPWMFrequency(1000);  // Set PWM freq to 100Hz (default is 200Hz, supports 24Hz to 1526Hz)
      break;
    case 2:
      Chip0.resetDevices();
      Chip0.init();
      Chip0.setPWMFrequency(1000);
      Chip1.init();
      Chip1.setPWMFrequency(1000);
      break;
    case 3:
      Chip0.resetDevices();
      Chip0.init();
      Chip0.setPWMFrequency(1000);
      Chip1.init();
      Chip1.setPWMFrequency(1000);
      Chip2.init();
      Chip2.setPWMFrequency(1000);
      break;
  }
  ResetAllOutputs();  // start in OFF state

  // usage examples: ledcWrite(FlipperLeftCh, FlipperFireVal);  ledcWrite(FlipperLeftCh, FlipperHoldVal);
  ledcSetup(FlipperLeftCh, PWMFreq, PWMResolution);
  ledcAttachPin(FlipperLeftPin, FlipperLeftCh);
  ledcSetup(FlipperRightCh, PWMFreq, PWMResolution);
  ledcAttachPin(FlipperRightPin, FlipperRightCh);
  ledcSetup(ShakerCh, PWMFreq, PWMResolution);
  ledcAttachPin(ShakerPin, ShakerCh);

  ReadFromMemory();
  
  digitalWrite(Ledpin, HIGH);
}

void loop() {
  nightModeIn = digitalRead(nightModePin);
  CheckOutputTimes();  // check for expired outputs that are on and reset if necessary
  if (UseSolenoidPWM != 0)
  {
    if (previousOutputState[fLpos] != 0 && millis() - outputResetTimer[fLpos] > (outputTimerVal[fLpos] * 60) && fLheld != 1)
    {
      ledcWrite(FlipperLeftCh, FlipperHoldVal);
      fLheld = 1;
    }
    else if (previousOutputState[fRpos] != 0 && millis() - outputResetTimer[fRpos] > (outputTimerVal[fRpos] * 60) && fRheld != 1)
    {
      ledcWrite(FlipperRightCh, FlipperHoldVal);
      fRheld = 1;
    }
  }
  if (Serial.available()) 
  {
    ReceiveOutputData();
  }
}
