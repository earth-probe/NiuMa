#include <Arduino.h>
#include "debug.h"
void setupTaskMotor(void);
void setupIMU(void);
void readIMU(void);


void MotorTask( void * parameter) {
  int core = xPortGetCoreID();
  DUMP_I(core);
  setupTaskMotor();
  setupIMU();
  for(;;) {//
    //float degree = temperatureRead();
    //DUMP_F(degree);
    readIMU();  
    //delay(1000);
    delay(1);
  }
}

void setupTaskMotor(void) {
}


