#include <Arduino.h>
#include "debug.hpp"

void setupDriveMotor(void);
void execDriveMotor(void);
void DriveMotorTask( void * parameter) {
  int core = xPortGetCoreID();
  LOG_I(core);
  setupDriveMotor();
  for(;;) {//
    execDriveMotor();
    delay(1);
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


volatile uint8_t gDriveMotorSpeed = 0;
volatile uint8_t gDriveMotorDir = 1;
volatile uint8_t gDriveMotorBrake = 1;

static auto gMilliSecAtLastCommand = millis();
void refreshExternDrivecommand(float speed,bool dir,bool brake) {
  LOG_F(speed);
  LOG_I(dir);
  LOG_I(brake);
  gMilliSecAtLastCommand = millis();
  if(brake) {
    gDriveMotorSpeed = 0;
    gDriveMotorBrake = 1;
  } else {
    gDriveMotorSpeed = (uint8_t)speed;
    gDriveMotorBrake = 0;
  }
  if(dir) {
    gDriveMotorDir = 1;
  } else {
    gDriveMotorDir = 0;
  }
}

static const u_long iConstOneCommandInterval = 1000;
void execDriveMotor(void) {
  auto now = millis();
  if(now - gMilliSecAtLastCommand > iConstOneCommandInterval) {
    gDriveMotorSpeed = 0;
    gDriveMotorBrake = 1;
  }
  ledcWrite(iConstPinSpeedPWMChannel,gDriveMotorSpeed);
  digitalWrite(iConstPinDIR,gDriveMotorDir);
  digitalWrite(iConstPinBrake,gDriveMotorBrake);
}
