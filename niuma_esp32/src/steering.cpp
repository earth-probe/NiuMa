#include <Arduino.h>
#include "debug.hpp"
void setupSteeringMotor(void);
void execSteeringMotor(void);
void execSteeringCalibration(void);
void calcCalibration(void);
void calcSteeringTarget(void);


void read_angle_table(void);

void SteeringMotorTask( void * parameter) {
  int core = xPortGetCoreID();
  LOG_I(core);
  setupSteeringMotor();
  read_angle_table();
  for(;;) {//
    execSteeringCalibration();
    calcCalibration();
    calcSteeringTarget();
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
volatile static float fTargetTurnAngleLeft = -1.0;
volatile static float fTargetTurnAngleRight = -1.0;
volatile static float fTargetTurnAngle = -1.0;

volatile static float gLeftMaxTurn = 0.0;
volatile static float gCenterTurn = 0.0;
volatile static float gRightMaxTurn = 0.0;

volatile static float gLeftMaxTurnX = 0.0;
volatile static float gRightMaxTurnX = 0.0;
volatile static float gWdithTurnX = std::abs(gRightMaxTurnX - gLeftMaxTurnX);

void refreshExternSteeringCommand(float angle,bool brake) {
  if(brake) {
    gDriveMotorExtend = 1;
    gDriveMotorReduce = 1;
  } else {
    if(angle > 0) {
      //gDriveMotorExtend = 0;
      //gDriveMotorReduce = 1;
      fTargetTurnAngleLeft = -1.0;
      fTargetTurnAngleRight = angle;
    } else {
      //gDriveMotorExtend = 1;
      //gDriveMotorReduce = 0;
      fTargetTurnAngleLeft = angle;
      fTargetTurnAngleRight = -1.0;
    }
    fTargetTurnAngle = angle;
  }
  gMilliSecAtLastCommand = millis();
  LOG_I(gDriveMotorExtend);
  LOG_I(gDriveMotorReduce);
  LOG_F(fTargetTurnAngleLeft);
  LOG_F(fTargetTurnAngleRight);

  LOG_F(gLeftMaxTurn);
  LOG_F(gCenterTurn);
  LOG_F(gRightMaxTurn);

  LOG_F(gLeftMaxTurnX);
  LOG_F(gRightMaxTurnX);

}

static const long constSteeringMotorIntervalMS = 200; 

volatile bool gIsRunCalibration = false;


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


static const int iConstAddressOfLeftMaxTurn = 0;
static const int iConstAddressOfCenterTurn = iConstAddressOfLeftMaxTurn + sizeof(float);
static const int iConstAddressOfRightMaxTurn = iConstAddressOfCenterTurn + sizeof(float);


void read_angle_table(void) {
  float fLeftMaxTurn = 0.0;
  EEPROM.get( iConstAddressOfLeftMaxTurn, fLeftMaxTurn);
  gLeftMaxTurn = fLeftMaxTurn;
  float fCenterTurn = 0.0;
  EEPROM.get( iConstAddressOfCenterTurn, fCenterTurn);
  gCenterTurn = fCenterTurn;
  float fRightMaxTurn = 0.0;
  EEPROM.get( iConstAddressOfRightMaxTurn, fRightMaxTurn);
  gRightMaxTurn = fRightMaxTurn;
  LOG_F(fLeftMaxTurn);
  LOG_F(fCenterTurn);
  LOG_F(fLeftMaxTurn);
}

static const float iConstAngleLeftMax = 0.0 - 100.0;
static const float iConstAngleRightMax = 0.0 + 100.0;
static const float iConstAngleWidth = iConstAngleRightMax - iConstAngleLeftMax;

void calcSteeringTargetWithX(void);
void calcSteeringTarget(void) {
  DUMP_F(fTargetTurnAngleLeft);
  DUMP_F(fTargetTurnAngleRight);
  calcSteeringTargetWithX();
}


void calcSteeringTargetWithX(void) {
  DUMP_F(fTargetTurnAngle);
  const float targetRange = fTargetTurnAngle + iConstAngleWidth;
  const float targetMagnetX = gLeftMaxTurnX + (targetRange * gWdithTurnX)/iConstAngleWidth;
}



#include <tuple>
#include <vector>
std::vector<std::tuple<float,float,float>> gStoreMagnet; 

static volatile bool gIsCalcCalibration = false;
static const long constSteeringCalibrationStage1 = 2000;
static const long constSteeringCalibrationFinnish = constSteeringCalibrationStage1 + 3000;
static const long constSteeringCalibrationCalc = constSteeringCalibrationFinnish + 100;
static auto gMilliSecStartCalibration = millis()  - constSteeringCalibrationCalc;

void refreshExternSteeringCalibration(bool run) {
  gMilliSecStartCalibration = millis();
  gDriveMotorExtend4Calibration = 0;
  gDriveMotorReduce4Calibration = 1;
  gStoreMagnet.clear();
  gIsCalcCalibration = true;
}


static const float fConstMagnetMin = 0.01;

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
  } else if(escaped_ms < constSteeringCalibrationFinnish) {
    gDriveMotorExtend4Calibration = 1;
    gDriveMotorReduce4Calibration = 0;
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
        //LOG_F(magnetX);
        //LOG_F(magnetY);
        //LOG_F(magnetZ);
        auto magnet = std::make_tuple(magnetX,magnetY,magnetZ);
        gStoreMagnet.push_back(magnet);
      }
    }
  }
}

void calcCalibrationReal(void);
void calcCalibration(void) {
  if(gIsCalcCalibration == false) {
    return ;
  }
  auto const escaped_ms = millis() - gMilliSecStartCalibration;
  if(escaped_ms > constSteeringCalibrationFinnish && 
    escaped_ms < constSteeringCalibrationCalc
    )
  {
    calcCalibrationReal();
    gIsCalcCalibration = false;
  }
}

void calcCalibrationReal(void) {
  if(gStoreMagnet.size() == 0) {
    return;
  }
  auto first = gStoreMagnet.begin();
  float maxX = std::get<0>(*first);
  float maxY = std::get<1>(*first);
  float maxZ = std::get<2>(*first);
  int xIndexMax = 0;
  int yIndexMax = 0;
  int zIndexMax = 0;
  int indexOfAll = 0;
  for(auto magenetIt :gStoreMagnet) {
    auto x = std::get<0>(magenetIt);
    if(x > maxX) {
      maxX = x;
      xIndexMax = indexOfAll;
    }
    auto y = std::get<1>(magenetIt);
    if(y > maxY) {
      maxY = y;
      yIndexMax = indexOfAll;
    }
    auto z = std::get<2>(magenetIt);
    if(z > maxZ) {
      maxZ = z;
      zIndexMax = indexOfAll;
    }
    indexOfAll ++;
  }
  LOG_F(maxX);
  LOG_F(maxY);
  LOG_F(maxZ);
  LOG_I(xIndexMax);
  LOG_I(yIndexMax);
  LOG_I(zIndexMax);

  float minXLeft = std::get<0>(*first);
  float minYLeft = std::get<1>(*first);
  float minZLeft = std::get<2>(*first);
  auto last = gStoreMagnet.rbegin();
  float minXRight = std::get<0>(*last);
  float minYRight = std::get<1>(*last);
  float minZRight = std::get<2>(*last);

  int xIndexMinLeft = 0;
  int yIndexMinLeft = 0;
  int zIndexMinLeft = 0;

  int xIndexMinRight = gStoreMagnet.size()-1;
  int yIndexMinRight = gStoreMagnet.size()-1;
  int zIndexMinRight = gStoreMagnet.size()-1;

  int indexMin = 0;
  for(auto magenetIt :gStoreMagnet) {
    auto x = std::get<0>(magenetIt);
   if(indexMin <= xIndexMax) {
      if(x < minXLeft) {
        minXLeft = x;
        xIndexMinLeft = indexMin;
      }

    } else {
      if(x < minXRight) {
        minXRight = x;
        xIndexMinRight = indexMin;
      }
    }
    auto y = std::get<1>(magenetIt);
    if(indexMin <= yIndexMax) {
      if(y < minYLeft) {
        minYLeft = y;
        yIndexMinLeft = indexMin;
      }

    } else {
      if(y < minYRight) {
        minYRight = y;
        yIndexMinRight = indexMin;
      }
    }
    auto z = std::get<2>(magenetIt);
    if(indexMin <= zIndexMax) {
      if(z < minZLeft) {
        minZLeft = z;
        zIndexMinLeft = indexMin;
      }

    } else {
      if(z < minZRight) {
        minZRight = z;
        zIndexMinRight = indexMin;
      }
    }
    indexMin ++;
  }

  LOG_F(minXLeft);
  LOG_F(minYLeft);
  LOG_F(minZLeft);

  LOG_I(xIndexMinLeft);
  LOG_I(yIndexMinLeft);
  LOG_I(zIndexMinLeft);

  LOG_F(minXRight);
  LOG_F(minYRight);
  LOG_F(minZRight);

  LOG_I(xIndexMinRight);
  LOG_I(yIndexMinRight);
  LOG_I(zIndexMinRight);
  EEPROM.put(iConstAddressOfLeftMaxTurn,minYLeft);
  EEPROM.put(iConstAddressOfCenterTurn,maxY);
  EEPROM.put(iConstAddressOfRightMaxTurn,minYRight);
  gLeftMaxTurn = minYLeft;
  gCenterTurn = maxY;
  gRightMaxTurn = minYRight;


  gLeftMaxTurnX = minXLeft;
  gRightMaxTurnX = maxY;
  gWdithTurnX = std::abs(gRightMaxTurnX - gLeftMaxTurnX);
}
