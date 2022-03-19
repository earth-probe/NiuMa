#include <Arduino.h>
#include "debug.hpp"

#define X_TURN
//#define Y_TURN


void setupSteeringMotor(void);
void execSteeringMotor(void);
void execSteeringCalibration(void);
void calcCalibration(void);
void makeSteeringExec(void);


void read_angle_table(void);

void SteeringMotorTask_l298n( void * parameter) {
  int core = xPortGetCoreID();
  LOG_I(core);
  setupSteeringMotor();
  read_angle_table();
  for(;;) {//
    
    execSteeringCalibration();
    calcCalibration();

    makeSteeringExec();
    execSteeringMotor();
    
    delay(20);
  }
}

/*
extern volatile float accX;
extern volatile float accY;
extern volatile float accZ;
extern volatile float gyroX;
extern volatile float gyroY;
extern volatile float gyroZ;
extern volatile float magnetX;
extern volatile float magnetY;
extern volatile float magnetZ;
*/

extern volatile float magnet4SteeringX;
extern volatile float magnet4SteeringY;
extern volatile float magnet4SteeringZ;



static const uint8_t iConstPinPWNEna = GPIO_NUM_17; 
static const uint8_t iConstPinDirIN1 = GPIO_NUM_18;
static const uint8_t iConstPinDirIN2 = GPIO_NUM_19;


static const uint8_t iConstPinSteeringPWMChannel = 2;

void setupSteeringMotor(void) {
  pinMode(iConstPinPWNEna, OUTPUT);
  ledcSetup(iConstPinSteeringPWMChannel,1000,8);
  ledcAttachPin(iConstPinPWNEna, iConstPinSteeringPWMChannel);

  pinMode(iConstPinDirIN1, OUTPUT);
  pinMode(iConstPinDirIN2, OUTPUT);
  digitalWrite(iConstPinDirIN1,HIGH);
  digitalWrite(iConstPinDirIN2,HIGH);

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

#ifdef X_TURN
volatile static float gLeftMaxTurnX = -0.001775;
volatile static float gRightMaxTurnX = -0.411256;
volatile static float gCenterTurnX = (gLeftMaxTurnX + gRightMaxTurnX) /2;
volatile static float gWidthTurnX = std::abs(gRightMaxTurnX - gLeftMaxTurnX);
volatile static float fTargetMagnetX = (gLeftMaxTurnX+gRightMaxTurnX)/2;
#endif

#ifdef Y_TURN
volatile static float gLeftMaxTurnY = -1.744994;
volatile static float gRightMaxTurnY = 1.708018;
volatile static float gCenterTurnY = (gLeftMaxTurnY + gRightMaxTurnY) /2;
volatile static float gWidthTurnY = std::abs(gRightMaxTurnY - gLeftMaxTurnY);
volatile static float fTargetMagnetY = (gLeftMaxTurnY+gRightMaxTurnY)/2;
#endif


volatile static uint8_t gISpeedSteering = 0;


void calcSteeringTarget(void);

void refreshExternSteeringCommand(float angle,bool brake) {
  if(brake) {
    gDriveMotorExtend = HIGH;
    gDriveMotorReduce = HIGH;
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
  DUMP_I(gDriveMotorExtend);
  DUMP_I(gDriveMotorReduce);
  DUMP_F(fTargetTurnAngleLeft);
  DUMP_F(fTargetTurnAngleRight);

  DUMP_F(gLeftMaxTurn);
  DUMP_F(gCenterTurn);
  DUMP_F(gRightMaxTurn);

  LOG_F(fTargetTurnAngle);

#ifdef X_TURN
  LOG_F(gLeftMaxTurnX);
  LOG_F(gRightMaxTurnX);
#endif

#ifdef Y_TURN
  LOG_F(gLeftMaxTurnY);
  LOG_F(gRightMaxTurnY);
#endif

  calcSteeringTarget();
#ifdef X_TURN
  LOG_F(fTargetMagnetX);
#endif
#ifdef Y_TURN
  LOG_F(fTargetMagnetY);
#endif

}


volatile bool gIsRunCalibration = false;


uint8_t gDriveMotorExtend4Calibration = HIGH;
uint8_t gDriveMotorReduce4Calibration = HIGH;

static const float fConstSpeedIOVoltMin = 40.0;
static const uint8_t iConstSpeedIOVoltMin = static_cast<uint8_t>(fConstSpeedIOVoltMin);
static const float fConstSpeedIOVoltMax = 255.0;
static const float fConstSpeedIOVoltWidth = fConstSpeedIOVoltMax -fConstSpeedIOVoltMin;
static const uint8_t iConstSpeedIOVoltMax = static_cast<uint8_t>(fConstSpeedIOVoltMax);
static volatile bool gIsCalcCalibration = false;

void execSteeringMotor(void) {
  DUMP_I(gDriveMotorExtend);  
  DUMP_I(gDriveMotorReduce);
  if(gIsRunCalibration) {
    digitalWrite(iConstPinDirIN1,gDriveMotorExtend4Calibration);
    digitalWrite(iConstPinDirIN2,gDriveMotorReduce4Calibration);
    ledcWrite(iConstPinSteeringPWMChannel,iConstSpeedIOVoltMax);
  } else {
    digitalWrite(iConstPinDirIN1,gDriveMotorExtend);
    digitalWrite(iConstPinDirIN2,gDriveMotorReduce);
    ledcWrite(iConstPinSteeringPWMChannel,gISpeedSteering);
  }
}

#include <Preferences.h>
Preferences preferences;



void read_angle_table(void) {
  preferences.begin("steering", false);
#ifdef X_TURN
  gLeftMaxTurnX = preferences.getFloat("gLeftMaxTurnX",gLeftMaxTurnX);
  gRightMaxTurnX = preferences.getFloat("gRightMaxTurnX",gRightMaxTurnX);
  gCenterTurnX = preferences.getFloat("gCenterTurnX",gCenterTurnX);
  gWidthTurnX = preferences.getFloat("gWidthTurnX",gWidthTurnX);
#endif
#ifdef Y_TURN
  gLeftMaxTurnY = preferences.getFloat("gLeftMaxTurnY",gLeftMaxTurnY);
  gRightMaxTurnY = preferences.getFloat("gRightMaxTurnY",gRightMaxTurnY);
  gCenterTurnY = preferences.getFloat("gCenterTurnY",gCenterTurnY);
  gWidthTurnY = preferences.getFloat("gWidthTurnY",gWidthTurnY);
#endif
  
}

static const float iConstAngleLeftMax = 0.0 - 45.0;
static const float iConstAngleRightMax = 0.0 + 45.0;
static const float iConstAngleWidth = iConstAngleRightMax - iConstAngleLeftMax;
static float gDiffPIDOfSum = 0.0;

void calcSteeringTargetWithX(void);
void calcSteeringTarget(void) {
  DUMP_F(fTargetTurnAngleLeft);
  DUMP_F(fTargetTurnAngleRight);
  calcSteeringTargetWithX();
  gDiffPIDOfSum = 0;
}


void calcSteeringTargetWithX(void) {
  LOG_F(fTargetTurnAngle);
  const float targetRange = fTargetTurnAngle - iConstAngleLeftMax;
  LOG_F(targetRange);
#ifdef X_TURN
  fTargetMagnetX = gLeftMaxTurnX + (targetRange * gWidthTurnX)/iConstAngleWidth;
  LOG_F(fTargetMagnetX);
#endif
#ifdef Y_TURN
  fTargetMagnetY = gLeftMaxTurnY + (targetRange * gWidthTurnY)/iConstAngleWidth;
  LOG_F(fTargetMagnetY);
#endif

}

static const float fConstDiffOfMangetSteering = 0.15;

static const float fConstDiffGainOfKp = 1.0;
static const float fConstDiffGainOfKi = 0.05;
static const float fConstDiffGainOfKd = 8.0;
static float gDiffPIDOfDivPrev = 0.0;

void makeSteeringExec(void) {
  if(gIsCalcCalibration) {
    return;
  }
  #ifdef X_TURN
  const float diffMagnet =  fTargetMagnetX - magnet4SteeringX;
  #endif
  #ifdef Y_TURN
  const float diffMagnet =  fTargetMagnetY - magnet4SteeringY;
  #endif
  const float absDiffMagnet = std::abs(diffMagnet);
  DUMP_F(diffMagnet);
  DUMP_F(absDiffMagnet);
  if(absDiffMagnet > fConstDiffOfMangetSteering ) {
    DUMP_F(diffMagnet);
    if(diffMagnet < 0.0) {
      gDriveMotorExtend = 1;
      gDriveMotorReduce = 0;
    } else {
      gDriveMotorExtend = 0;
      gDriveMotorReduce = 1;
    }

    DUMP_F(absDiffMagnet);
    DUMP_F(fConstSpeedIOVoltMax);
    #ifdef X_TURN
    DUMP_F(gWidthTurnX);
    #endif
    #ifdef Y_TURN
    DUMP_F(gWidthTurnY);
    #endif


    auto pidAdjustValue = absDiffMagnet * fConstDiffGainOfKp;
    gDiffPIDOfSum += absDiffMagnet;
    DUMP_F(gDiffPIDOfSum);
    pidAdjustValue += gDiffPIDOfSum * fConstDiffGainOfKi;


    pidAdjustValue += std::abs(absDiffMagnet-gDiffPIDOfDivPrev) * fConstDiffGainOfKd;
    gDiffPIDOfDivPrev = absDiffMagnet;
  
    DUMP_F(pidAdjustValue);
    #ifdef X_TURN
    float diff2Speed = fConstSpeedIOVoltMin + (pidAdjustValue *fConstSpeedIOVoltWidth) / gWidthTurnX;
    #endif
    #ifdef Y_TURN
    float diff2Speed = fConstSpeedIOVoltMin + (pidAdjustValue *fConstSpeedIOVoltWidth) / gWidthTurnY;
    #endif
    DUMP_F(diff2Speed);
    if(diff2Speed > fConstSpeedIOVoltMax) {
      diff2Speed = fConstSpeedIOVoltMax;
    }
    DUMP_F(diff2Speed);
    
    gISpeedSteering = static_cast<uint8_t>(diff2Speed);
    DUMP_I(gISpeedSteering);
    if(gISpeedSteering > iConstSpeedIOVoltMax) {
      gISpeedSteering = iConstSpeedIOVoltMax;
    }
    if(gISpeedSteering < iConstSpeedIOVoltMin) {
      gISpeedSteering = iConstSpeedIOVoltMin;
    }
    DUMP_I(gISpeedSteering); 
  } else {
    gISpeedSteering = 0;
    static int counterOfSum = 0;
    if(counterOfSum++ > 3) {
      gDiffPIDOfSum = 0;
      counterOfSum = 0;
    }
    gDriveMotorExtend = HIGH;
    gDriveMotorReduce = HIGH;
  }
  DUMP_I(gDriveMotorExtend);
  DUMP_I(gDriveMotorReduce);
}


#include <tuple>
#include <vector>
std::vector<std::tuple<float,float,float>> gStoreMagnet; 

static const long constSteeringCalibrationStage1 = 2000;
static const long constSteeringCalibrationFinnish = constSteeringCalibrationStage1 + 3000;
static const long constSteeringCalibrationCalc = constSteeringCalibrationFinnish + 300;
static auto gMilliSecStartCalibration = millis()  - constSteeringCalibrationCalc;

void refreshExternSteeringCalibration(bool run) {
  gMilliSecStartCalibration = millis();
  gDriveMotorExtend4Calibration = LOW;
  gDriveMotorReduce4Calibration = HIGH;
  gStoreMagnet.clear();
  gIsCalcCalibration = true;
}


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
    storeManget = true;
    gDriveMotorExtend4Calibration = LOW;
    gDriveMotorReduce4Calibration = HIGH;
  } else if(escaped_ms < constSteeringCalibrationFinnish) {
    storeManget = true;
    gDriveMotorExtend4Calibration = HIGH;
    gDriveMotorReduce4Calibration = LOW;
  } else {

  }
  if(storeManget) {
    if(gStoreMagnet.size() == 0) {
      auto magnet = std::make_tuple(magnet4SteeringX,magnet4SteeringY,magnet4SteeringZ);
      gStoreMagnet.push_back(magnet);
    } else {
      static auto prevmagnetX = magnet4SteeringX;
      static auto prevmagnetY = magnet4SteeringY;
      static auto prevmagnetZ = magnet4SteeringZ;
      if( std::abs(prevmagnetX - magnet4SteeringX) > fConstMagnetMin ||
        std::abs(prevmagnetY - magnet4SteeringY) > fConstMagnetMin ||
        std::abs(prevmagnetZ - magnet4SteeringZ) > fConstMagnetMin 
      ) {
        prevmagnetX = magnet4SteeringX;
        prevmagnetY = magnet4SteeringY;
        prevmagnetZ = magnet4SteeringZ;
        //LOG_F(magnet4SteeringX);
        //LOG_F(magnet4SteeringY);
        //LOG_F(magnet4SteeringZ);
        auto magnet = std::make_tuple(magnet4SteeringX,magnet4SteeringY,magnet4SteeringZ);
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
  float minX = std::get<0>(*first);
  float minY = std::get<1>(*first);
  float minZ = std::get<2>(*first);
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
    if(x < minX) {
      minX = x;
    }
    auto y = std::get<1>(magenetIt);
    if(y > maxY) {
      maxY = y;
      yIndexMax = indexOfAll;
    }
    if(y < minY) {
      minY = y;
    }
    auto z = std::get<2>(magenetIt);
    if(z > maxZ) {
      maxZ = z;
      zIndexMax = indexOfAll;
    }
    if(z < minZ) {
      minZ = z;
    }
    indexOfAll ++;
  }
  LOG_F(maxX);
  LOG_F(maxY);
  LOG_F(maxZ);
  LOG_F(minX);
  LOG_F(minY);
  LOG_F(minZ);
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
  preferences.putFloat("minYLeft",minYLeft);
  preferences.putFloat("maxY",maxY);
  preferences.putFloat("minYRight",minYRight);

  gLeftMaxTurn = minYLeft;
  gCenterTurn = maxY;
  gRightMaxTurn = minYRight;


#ifdef X_TURN
  gLeftMaxTurnX = std::min(minXLeft,minXRight);
  gRightMaxTurnX = maxX;
  gWidthTurnX = std::abs(gRightMaxTurnX - gLeftMaxTurnX);
  gCenterTurnX = (gLeftMaxTurnX + gRightMaxTurnX) /2.0;
  fTargetMagnetX = gCenterTurnX;
  preferences.putFloat("gLeftMaxTurnX",gLeftMaxTurnX);
  preferences.putFloat("gRightMaxTurnX",gRightMaxTurnX);
  preferences.putFloat("gCenterTurnX",gCenterTurnX);
  preferences.putFloat("gWidthTurnX",gWidthTurnX);
#endif

#ifdef Y_TURN
  gLeftMaxTurnY = std::min(minYLeft,minYRight);
  gRightMaxTurnY = maxY;
  gWidthTurnY = std::abs(gRightMaxTurnY - gLeftMaxTurnY);
  gCenterTurnY = (gLeftMaxTurnY + gRightMaxTurnY) /2.0;
  fTargetMagnetY = gCenterTurnY;
  preferences.putFloat("gLeftMaxTurnY",gLeftMaxTurnY);
  preferences.putFloat("gRightMaxTurnY",gRightMaxTurnY);
  preferences.putFloat("gCenterTurnY",gCenterTurnY);
  preferences.putFloat("gWidthTurnY",gWidthTurnY);
#endif

}
