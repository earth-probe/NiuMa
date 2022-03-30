#include <Arduino.h>
#include "debug.hpp"
#include <HMC5883L.h>
void setupCompass(void);
void readCompass(void);

HMC5883L compass;



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
  TwoWire selfWire(1);
  auto goodWire = selfWire.begin(GPIO_NUM_21,GPIO_NUM_22,400000);
  LOG_I(goodWire);
  /*
  auto goodWire = Wire.begin(GPIO_NUM_21,GPIO_NUM_22,400000);
  LOG_I(goodWire);
  compass.initCompass();
  auto id = compass.getCompass();
  LOG_I(id);
  */
}

static const long constReadImuIntervalMS = 16;


void readCompass(void) {
  static long previousMillis = 0;
  auto nowMS = millis();
  if(nowMS - previousMillis < constReadImuIntervalMS) {
    return;
  }
  previousMillis = nowMS;
  //auto magnet = compass.readRawAxis();
  //LOG_I(magnet.XAxis);
  //LOG_I(magnet.YAxis);
  //LOG_I(magnet.ZAxis);
  //auto magnet = compass.readScaledAxis();
}

