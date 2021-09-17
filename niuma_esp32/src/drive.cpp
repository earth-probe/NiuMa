#include <Arduino.h>
#include "debug.h"

void DriveMotorTask( void * parameter) {
  int core = xPortGetCoreID();
  DUMP_I(core);
  for(;;) {//
    delay(1);
  }
}
