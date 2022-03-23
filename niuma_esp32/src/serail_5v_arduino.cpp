#include <Arduino.h>
#include <ArduinoJson.h>
#include <sstream>
#include "debug.hpp"

#define RXD2 16
#define TXD2 17


void setupSerial(void) {
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
}

static String gSerialOfArduino = "";
void parseArduino5V(void) {
  LOG_S(gSerialOfArduino);
  StaticJsonDocument<64> doc;
  deserializeJson(doc, gSerialOfArduino);
  serializeJson(doc, Serial);
}

void SerialArduino5VTask( void * parameter) {
  int core = xPortGetCoreID();
  LOG_I(core);
  setupSerial();
  for(;;) {//
    if (Serial2.available() > 0) {
      char incomingByte = Serial2.read();
      if(incomingByte =='\n' || incomingByte =='\r') {
        parseArduino5V();
        gSerialOfArduino = "";
      } else {
        gSerialOfArduino += incomingByte;
      }
      if(gSerialOfArduino.length() > 128) {
        gSerialOfArduino = "";
      }
      //LOG_S(gSerialOfArduino);    
    }
    //Serial2.println("{}\r\n");
    delay(1);
  }
}
