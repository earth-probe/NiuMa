#include <Arduino.h>
#include "debug.hpp"
void setupSteeringMotor(void);
void execSteeringMotor(void);
void execSteeringCalibration(void);

void read_angle_table(void);

void SteeringMotorTask( void * parameter) {
  int core = xPortGetCoreID();
  LOG_I(core);
  setupSteeringMotor();
  read_angle_table();
  for(;;) {//
    execSteeringCalibration();
    execSteeringMotor();
    delay(1);
  }
}

extern volatile float accX;
extern volatile float accY;
extern volatile float accZ;
extern volatile float gyroX;
extern volatile float gyroY;
extern volatile float gyroZ;
extern volatile float magnetX;
extern volatile float magnetY;
extern volatile float magnetZ;




static const uint8_t iConstPinExtend = GPIO_NUM_17; 
static const uint8_t iConstPinReduce = GPIO_NUM_18;
static const uint8_t iConstPinSteeringLevelOE = GPIO_NUM_19;

void setupSteeringMotor(void) {
  pinMode(iConstPinExtend, OUTPUT);
  pinMode(iConstPinReduce, OUTPUT);

  pinMode(iConstPinSteeringLevelOE, OUTPUT);
  digitalWrite(iConstPinSteeringLevelOE,1);
}

static auto gMilliSecAtLastCommand = millis();
volatile uint8_t gDriveMotorExtend = 1;
volatile uint8_t gDriveMotorReduce = 1;
void refreshExternSteeringCommand(float angle,bool brake) {
  if(brake) {
    gDriveMotorExtend = 1;
    gDriveMotorReduce = 1;
  } else {
    if(angle > 0) {
      gDriveMotorExtend = 0;
      gDriveMotorReduce = 1;
    } else {
      gDriveMotorExtend = 1;
      gDriveMotorReduce = 0;
    }
  }
  gMilliSecAtLastCommand = millis();
  LOG_I(gDriveMotorExtend);
  LOG_I(gDriveMotorReduce);
}

static const long constSteeringMotorIntervalMS = 200; 
static bool gIsRunCalibration = false;


uint8_t gDriveMotorExtend4Calibration = 1;
uint8_t gDriveMotorReduce4Calibration = 1;

void execSteeringMotor(void) {
  DUMP_I(gDriveMotorExtend);  
  DUMP_I(gDriveMotorReduce);
  if(gIsRunCalibration) {
    digitalWrite(iConstPinExtend,gDriveMotorExtend4Calibration);
    digitalWrite(iConstPinReduce,gDriveMotorReduce4Calibration);
  } else {
    digitalWrite(iConstPinExtend,gDriveMotorExtend);
    digitalWrite(iConstPinReduce,gDriveMotorReduce);
  }
}

#include <EEPROM.h>


void read_angle_table(void) {
}

#include <tuple>
#include <vector>
std::vector<std::tuple<float,float,float>> gStoreMagnet; 

static auto gMilliSecStartCalibration = millis();
void refreshExternSteeringCalibration(bool run) {
  gMilliSecStartCalibration = millis();
  gDriveMotorExtend4Calibration = 0;
  gDriveMotorReduce4Calibration = 1;
  gStoreMagnet.clear();
}

static const long constSteeringCalibrationStage1 = 500;
static const long constSteeringCalibrationStage2 = constSteeringCalibrationStage1 + 1000;
static const long constSteeringCalibrationFinnish = constSteeringCalibrationStage2 + 1000;


static const float fConstMagnetMin = 0.001;

void execSteeringCalibration(void) {
  auto const escaped_ms = millis() - gMilliSecStartCalibration;
  if(escaped_ms > constSteeringCalibrationFinnish) {
    gIsRunCalibration = false;
    return ;
  }
  gIsRunCalibration = true;
  bool storeManget = false;
  if( escaped_ms < constSteeringCalibrationStage1) {
    gDriveMotorExtend4Calibration = 0;
    gDriveMotorReduce4Calibration = 1;
  } else if(escaped_ms < constSteeringCalibrationStage2) {
    gDriveMotorExtend4Calibration = 1;
    gDriveMotorReduce4Calibration = 0;
    storeManget = true;
  } else if(escaped_ms < constSteeringCalibrationFinnish) {
    gDriveMotorExtend4Calibration = 0;
    gDriveMotorReduce4Calibration = 1;
    storeManget = true;
  } else {

  }
  if(storeManget) {
    if(gStoreMagnet.size() == 0) {
      auto magnet = std::make_tuple(magnetX,magnetY,magnetZ);
      gStoreMagnet.push_back(magnet);
    } else {
      static auto prevmagnetX = magnetX;
      static auto prevmagnetY = magnetY;
      static auto prevmagnetZ = magnetZ;
      if( std::abs(prevmagnetX - magnetX) > fConstMagnetMin ||
        std::abs(prevmagnetY - magnetY) > fConstMagnetMin ||
        std::abs(prevmagnetZ - magnetZ) > fConstMagnetMin 
      ) {
        prevmagnetX = magnetX;
        prevmagnetY = magnetY;
        prevmagnetZ = magnetZ;
        auto magnet = std::make_tuple(magnetX,magnetY,magnetZ);
        gStoreMagnet.push_back(magnet);
      }
    }
  }
}
