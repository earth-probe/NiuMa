#include <Arduino.h>
#include <thread>
#include "debug.hpp"
void SteeringMotorTask_l298n( void * parameter);
void DriveMotorTask( void * parameter);
void BLETask( void * parameter);
void IMUTask( void * parameter);
void HallSpeedTask( void * parameter);

void CompassTask( void * parameter);


volatile SemaphoreHandle_t xMutex = NULL;


void setup() {
  Serial.begin(115200);

  auto freqDefault = getCpuFrequencyMhz();
  LOG_I(freqDefault);
  /*
  auto goodFreq = setCpuFrequencyMhz(240);
  LOG_I(goodFreq);
  auto freqRun = getCpuFrequencyMhz();
  LOG_I(freqRun);
  int core = xPortGetCoreID();
  LOG_I(core);
  */

  xMutex = xSemaphoreCreateMutex();
  xSemaphoreGive(xMutex);
  xTaskCreatePinnedToCore(BLETask, "BLETask", 10000, nullptr, 1, nullptr,  1); 
  //xTaskCreatePinnedToCore(IMUTask, "IMUTask", 10000, nullptr, 1, nullptr,  0); 
  xTaskCreatePinnedToCore(CompassTask, "CompassTask", 10000, nullptr, 1, nullptr,  0); 
  xTaskCreatePinnedToCore(SteeringMotorTask_l298n, "SteeringMotorTask", 10000, nullptr, 1, nullptr,  1); 
  xTaskCreatePinnedToCore(DriveMotorTask, "DriveMotorTask", 10000, nullptr, 1, nullptr,  1); 
  xTaskCreatePinnedToCore(HallSpeedTask, "HallSpeedTask", 10000, nullptr, 1, nullptr,  1); 
}

void loop() {
}