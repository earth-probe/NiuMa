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
  Wire.setPins(GPIO_NUM_21,GPIO_NUM_22);
  Wire.begin();
  compass.initCompass();
}

static const long constReadImuIntervalMS = 16;
static const float magnetFilterFatorA = 0.2; 
static const float magnetFilterFatorB = 1.0 - magnetFilterFatorA; 


void readCompass(void) {
  static long previousMillis = 0;
  auto nowMS = millis();
  if(nowMS - previousMillis < constReadImuIntervalMS) {
    return;
  }
  previousMillis = nowMS;
  auto magnet = compass.readRawAxis();
  LOG_I(magnet.XAxis);
  LOG_I(magnet.YAxis);
  LOG_I(magnet.ZAxis);
  //auto magnet = compass.readScaledAxis();
}

