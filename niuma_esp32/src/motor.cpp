#include <Arduino.h>
#include "debug.h"
void MotorTask( void * parameter) {
  int core = xPortGetCoreID();
  DUMP_I(core);
  for(;;) {//
    float degree = temperatureRead();
    //DUMP_F(degree);
    delay(1000);
  }
}
