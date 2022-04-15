#include <Arduino.h>
#include "debug.hpp"
#include "DFRobot_QMC5883/DFRobot_QMC5883.h"
void setupCompass(void);
void readCompass(void);

DFRobot_QMC5883 compass;



void CompassTask( void * parameter) {
  int core = xPortGetCoreID();
  LOG_I(core);
  setupCompass();
  for(;;) {//
    readCompass();
    delay(1);
  }
}

#include <Wire.h>

void setupCompass(void) {
  auto goodWire = Wire.begin();
  LOG_I(goodWire);
  while (!compass.begin()){
    Serial.println("Could not find a valid QMC5883 sensor, check wiring!");
    delay(500);
  }
  LOG_I(compass.isHMC());
  LOG_I(compass.isQMC());
  LOG_I(compass.isVCM());
}

static const long constReadImuIntervalMS = 16;

extern volatile float magnetX;
extern volatile float magnetY;
extern volatile float magnetZ;

void readCompass(void) {
  static long previousMillis = 0;
  auto nowMS = millis();
  if(nowMS - previousMillis < constReadImuIntervalMS) {
    return;
  }
  previousMillis = nowMS;
  auto magnet = compass.readRaw();
  //LOG_I(magnet.XAxis);
  //LOG_I(magnet.YAxis);
  //LOG_I(magnet.ZAxis);
  magnetX = magnet.XAxis ;
  magnetY = magnet.YAxis ;
  magnetZ = magnet.ZAxis;
}

