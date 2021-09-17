#include <Arduino.h>
#include <thread>
#include <ArduinoJson.h>
#include "debug.h"
void MotorTask( void * parameter);
void SteeringMotorTask( void * parameter);
void DriveMotorTask( void * parameter);
void BLETask( void * parameter);

void setupBLE(void);

volatile SemaphoreHandle_t xMutex = NULL;


void setup() {
  Serial.begin(115200);
  int core = xPortGetCoreID();
  DUMP_I(core);
  xMutex = xSemaphoreCreateMutex();
  xSemaphoreGive(xMutex);
  xTaskCreatePinnedToCore(BLETask, "BLETask", 10000, nullptr, 1, nullptr,  1); 
  xTaskCreatePinnedToCore(SteeringMotorTask, "SteeringMotorTask", 10000, nullptr, 1, nullptr,  0); 
  xTaskCreatePinnedToCore(DriveMotorTask, "DriveMotorTask", 10000, nullptr, 1, nullptr,  0); 
}

void runBleTransimit(void);
void loop() {
}
