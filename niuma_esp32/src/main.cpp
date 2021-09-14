#include <Arduino.h>
//#include <ArduinoJson.h>
#include "debug.h"
void MotorTask( void * parameter);

void setupBLE(void);

void setup() {
  Serial.begin(115200);
  int core = xPortGetCoreID();
  DUMP_I(core);
  xTaskCreatePinnedToCore(MotorTask, "MotorTask", 10000, nullptr, 1, nullptr,  0); 
  setupBLE();
}

void runBleTransimit(void);
void loop() {
  runBleTransimit();
}




