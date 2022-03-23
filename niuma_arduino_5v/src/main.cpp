#include <Arduino.h>
#include <ArduinoJson.h>

#if 0
#define DUMP_VAR(x)  { \
  Serial.print(__LINE__);\
  Serial.print("@@"#x"=<");\
  Serial.print(x);\
  Serial.print(">&$");\
}
#else
#define DUMP_VAR(x)  {}
#endif


/*
{"d":7,"v":1}
{"d":7,"v":0}

{"d":8,"v":1}
{"d":8,"v":0}

{"p":9,"v":0}
{"p":9,"v":127}
{"p":9,"v":255}


{"p":10,"v":0}
{"p":10,"v":127}
{"p":10,"v":255}

{"d":11,"v":1}
{"d":11,"v":0}

{"d":12,"v":1}
{"d":12,"v":0}

*/


// pwm pin 9 is bldc driver motor 
// pwm pin 10 is linear accator for steering

const static uint8_t MOTER_SPD_WHEEL = 9;
const static uint8_t MOTER_DIR_WHEEL = 8;
const static uint8_t MOTER_BRAKE_WHEEL = 7;

const static uint8_t MOTER_HA_WHEEL = 4;
const static uint8_t MOTER_HB_WHEEL = 5;
const static uint8_t MOTER_HC_WHEEL = 6;

const static uint8_t MOTER_SPD_STREERING = 10;
const static uint8_t MOTER_IN1_STREERING = 11;
const static uint8_t MOTER_IN2_STREERING = 12;

void setup() {
  // put your setup code here, to run once:
  
  /*
  Digital 9/10ピン(Timer1に対応)
  TCCR1B = (TCCR1B & 0b11111000) | 0x01; //31.373kHz 
  TCCR1B = (TCCR1B & 0b11111000) | 0x02; // 3.921kHz 
  TCCR1B = (TCCR1B & 0b11111000) | 0x03; //   490.2Hz 
  TCCR1B = (TCCR1B & 0b11111000) | 0x04; //   122.6Hz 
  TCCR1B = (TCCR1B & 0b11111000) | 0x05; //    30.6kHz   */
  TCCR1B = (TCCR1B & 0b11111000) | 0x03; //   490.2Hz 

  pinMode(MOTER_SPD_WHEEL, OUTPUT);
  pinMode(MOTER_DIR_WHEEL, OUTPUT);
  pinMode(MOTER_BRAKE_WHEEL, OUTPUT);

  pinMode(MOTER_HA_WHEEL, INPUT_PULLUP);
  pinMode(MOTER_HB_WHEEL, INPUT_PULLUP);
  pinMode(MOTER_HC_WHEEL, INPUT_PULLUP);

  pinMode(MOTER_SPD_STREERING, OUTPUT);
  pinMode(MOTER_IN1_STREERING, OUTPUT);
  pinMode(MOTER_IN2_STREERING, OUTPUT);

  Serial.begin(115200);
}

static String gSerialInputCommand = "";
void run_comand(void) {
  DUMP_VAR(gSerialInputCommand);
  StaticJsonDocument<64> doc;
  deserializeJson(doc, gSerialInputCommand);
  serializeJson(doc, Serial);
  uint8_t portD = doc["d"].as<uint8_t>();
  DUMP_VAR(portD);
  if(portD >= 7 && portD <=12 ) {
    uint8_t value = doc["v"].as<uint8_t>();
    digitalWrite(portD,value);
  }
  uint8_t portP = doc["p"].as<uint8_t>();
  DUMP_VAR(portD);
  if(portP == MOTER_SPD_WHEEL || portP == MOTER_SPD_STREERING ) {
    uint8_t value = doc["v"].as<uint8_t>();
    analogWrite(portP, value);
  }

  /*
  uint8_t portA = doc["a"].as<uint8_t>();
  DUMP_VAR(portA);
  if(portA >= A0 && portA <= A7 ) {
    uint8_t value = doc["v"].as<uint8_t>();
    analogWrite(portA, value);
  }
  */

}

 static int gHallAValue = 0;
 static int gHallBValue = 0;
 static int gHallCValue = 0;

void readHallsReport(void){
  int HA = digitalRead(MOTER_HA_WHEEL);
  if(HA != gHallAValue) {
    StaticJsonDocument<16> doc;
    doc["d"] = MOTER_HA_WHEEL;
    doc["v"] = HA;
    serializeJson(doc, Serial);
    Serial.println("");
  }
  gHallAValue = HA;

  int HB = digitalRead(MOTER_HB_WHEEL);
  if(HB != gHallBValue) {
    StaticJsonDocument<16> doc;
    doc["d"] = MOTER_HB_WHEEL;
    doc["v"] = HB;
    serializeJson(doc, Serial);
    Serial.println("");
  }
  gHallBValue = HB;

  int HC = digitalRead(MOTER_HC_WHEEL);
  if(HC != gHallCValue) {
    StaticJsonDocument<16> doc;
    doc["d"] = MOTER_HC_WHEEL;
    doc["v"] = HC;
    serializeJson(doc, Serial);
    Serial.println("");
 }
  gHallCValue = HC;
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    char incomingByte = Serial.read();
    if(incomingByte =='\n' || incomingByte =='\r') {
      run_comand();
      gSerialInputCommand = "";
    } else {
      gSerialInputCommand += incomingByte;
    }
    if(gSerialInputCommand.length() > 128) {
      gSerialInputCommand = "";
    }    
  }
  readHallsReport();
}
