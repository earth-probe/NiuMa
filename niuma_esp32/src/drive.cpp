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

static const uint8_t iConstPinSpeed = GPIO_NUM_27;
static const uint8_t iConstPinBrake = GPIO_NUM_32;
static const uint8_t iConstPinDIR = GPIO_NUM_33;
static const uint8_t iConstPinSpeedPWMChannel = 1;




void setupDriveMotor(void) {
  pinMode(iConstPinSpeed, OUTPUT);
  ledcSetup(iConstPinSpeedPWMChannel,1000,8);
  ledcAttachPin(iConstPinSpeed, iConstPinSpeedPWMChannel);
  pinMode(iConstPinDIR, OUTPUT);
  pinMode(iConstPinBrake, OUTPUT);
}


volatile uint8_t gDriveMotorSpeed = 0;
volatile uint8_t gDriveMotorDir = 1;
volatile uint8_t gDriveMotorBrake = 1;

void resetSpeedCalc(void);
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
  resetSpeedCalc();
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



void setupHallSpeed(void);
void readHallSpeed(void);
void calcHallSpeed(void);

void HallSpeedTask( void * parameter) {
  int core = xPortGetCoreID();
  LOG_I(core);
  setupHallSpeed();
  for(;;) {//
    readHallSpeed();
    //calcHallSpeed();
    delayMicroseconds(100);
  }
}

static const uint8_t iConstPinLevelOE = GPIO_NUM_16;
static const uint8_t iConstPinHallA = GPIO_NUM_12;
static const uint8_t iConstPinHallC = GPIO_NUM_14;
static const uint8_t iConstPinHallB = GPIO_NUM_15;

void raiseUpHallA(void);
void raiseUpHallB(void);
void raiseUpHallC(void);

void setupHallSpeed(void) {
  pinMode(iConstPinLevelOE, OUTPUT);
  digitalWrite(iConstPinLevelOE,1);

  pinMode(iConstPinHallA, INPUT_PULLDOWN);
  pinMode(iConstPinHallB, INPUT_PULLDOWN);
  pinMode(iConstPinHallC, INPUT_PULLDOWN);

/*
  attachInterrupt(iConstPinHallA,raiseUpHallA,RISING);
  attachInterrupt(iConstPinHallB,raiseUpHallB,RISING);
  attachInterrupt(iConstPinHallC,raiseUpHallC,RISING);
*/
}

#include <atomic>
volatile static std::atomic_int gHallValueA(0);
volatile static std::atomic_int gHallValueB(0);
volatile static std::atomic_int gHallValueC(0);

volatile static std::atomic_int gHallValueCountA(0);
volatile static std::atomic_int gHallValueCountB(0);
volatile static std::atomic_int gHallValueCountC(0);


/*
volatile static int gHallValueA(0);
volatile static int gHallValueB(0);
volatile static int gHallValueC(0);
*/

void raiseUpHallA(void)
{
  gHallValueA++;
}
void raiseUpHallB(void)
{
  gHallValueB++;
}
void raiseUpHallC(void)
{
  gHallValueC++;
}

void resetSpeedCalc(void)
{
  //gHallValueA = 0;
  //gHallValueB = 0;
  //gHallValueC = 0;
  gHallValueCountA = 0;
  gHallValueCountB = 0;
  gHallValueCountC = 0;
}

void calcHallSpeed(void)
{
  LOG_I(gHallValueA);
  LOG_I(gHallValueB);
  LOG_I(gHallValueC);
}

static const unsigned long iConstHallReadInterval = 500;
void readHallSpeed(void) {
  auto hallA = digitalRead(iConstPinHallA);
  auto hallB = digitalRead(iConstPinHallB);
  auto hallC = digitalRead(iConstPinHallC);
  static auto prevHallUs = micros();
  auto now = millis();
  if(hallA) {
    if(hallA != gHallValueA) {
      //LOG_I(hallA);
      //LOG_I(hallB);
      //LOG_I(hallC);
      if(now - prevHallUs > iConstHallReadInterval) {
        gHallValueCountA++;
        LOG_I(gHallValueCountA);
        LOG_I(gHallValueCountB);
        LOG_I(gHallValueCountC);
      }
    }
  }
  gHallValueA = hallA;

  if(hallB) {
    if(hallB != gHallValueB) {
      //LOG_I(hallA);
      //LOG_I(hallB);
      //LOG_I(hallC);
      if(now - prevHallUs > iConstHallReadInterval) {
        gHallValueCountB++;
        LOG_I(gHallValueCountA);
        LOG_I(gHallValueCountB);
        LOG_I(gHallValueCountC);
      }
    }
  }
  gHallValueB = hallB;

  if(hallC) {
    if(hallC != gHallValueC) {
      //LOG_I(hallA);
      //LOG_I(hallB);
      //LOG_I(hallC);
      if(now - prevHallUs > iConstHallReadInterval) {
        gHallValueCountC++;
        LOG_I(gHallValueCountA);
        LOG_I(gHallValueCountB);
        LOG_I(gHallValueCountC);
      }
    }
  }
  gHallValueC = hallC;

}
