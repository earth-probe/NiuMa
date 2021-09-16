#include <Arduino.h>
//#include <ArduinoJson.h>
#include "debug.h"
void MotorTask( void * parameter);

void setupBLE(void);

QueueHandle_t axQueue;
QueueHandle_t ayQueue;
QueueHandle_t azQueue;

void setup() {
  Serial.begin(115200);
  int core = xPortGetCoreID();
  DUMP_I(core);
  xTaskCreatePinnedToCore(MotorTask, "MotorTask", 10000, nullptr, 1, nullptr,  0); 
  axQueue = xQueueCreate(2, sizeof(float));
  ayQueue = xQueueCreate(2, sizeof(float));
  azQueue = xQueueCreate(2, sizeof(float));
  setupBLE();
}

void runBleTransimit(void);
void taskVarTrans(void);
void loop() {
  runBleTransimit();
  taskVarTrans();
}

float accXTask0;
float accYTask0;
float accZTask0;
float gyroXTask0;
float gyroYTask0;
float gyroZTask0;
float magnetXTask0;
float magnetYTask0;
float magnetZTask0;

void taskVarTrans(void)
{
  const TickType_t xTicksToWait = 1U;
  auto xStatus = xQueueReceive(axQueue, &accXTask0, xTicksToWait);
  DUMP_I(xStatus);
}
