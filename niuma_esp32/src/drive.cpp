#include <Arduino.h>
#include "debug.h"

void setupDriveMotor(void);
void execDriveMotor(void);
void DriveMotorTask( void * parameter) {
  int core = xPortGetCoreID();
  DUMP_I(core);
  setupDriveMotor();
  for(;;) {//
    execDriveMotor();
    delay(100);
  }
}

static const uint8_t iConstPinSpeed = GPIO_NUM_25; 
static const uint8_t iConstPinDIR = GPIO_NUM_33;
static const uint8_t iConstPinBrake = GPIO_NUM_32;
static const uint8_t iConstPinLevelOE = GPIO_NUM_27;
static const uint8_t iConstPinSpeedPWMChannel = 1;

void setupDriveMotor(void) {
  pinMode(iConstPinSpeed, OUTPUT);
  ledcSetup(iConstPinSpeedPWMChannel,1000,8);
  ledcAttachPin(iConstPinSpeed, iConstPinSpeedPWMChannel);
  pinMode(iConstPinDIR, OUTPUT);
  pinMode(iConstPinBrake, OUTPUT);

  pinMode(iConstPinLevelOE, OUTPUT);
  digitalWrite(iConstPinLevelOE,1);
}

volatile uint8_t gDriveMotorSpeed = 16;
volatile uint8_t gDriveMotorDir = 1;
volatile uint8_t gDriveMotorBrake = 0;

void execDriveMotor(void) {
  DUMP_I(gDriveMotorSpeed);
  //dacWrite(iConstPinSpeed,gDriveMotorSpeed);
  ledcWrite(iConstPinSpeedPWMChannel,gDriveMotorSpeed);
  
  digitalWrite(iConstPinDIR,gDriveMotorDir);
  digitalWrite(iConstPinBrake,gDriveMotorBrake);
}
