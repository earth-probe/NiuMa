#include <Arduino.h>
#include <Adafruit_INA219.h>
Adafruit_INA219 ina219;

int TempAnalogPin = 2;

int FETSWITCHPin = 9;
void setup() {
  pinMode(FETSWITCHPin,OUTPUT);
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(Serial == false) {

  }
  Serial.println("start ...");
  Serial.print(__DATE__);
  Serial.print(" ");
  Serial.println(__TIME__);
  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
  } else {
    Serial.println("Success to find INA219 chip");
  }
  //ina219.setCalibration_32V_1A();
}


#define LOG_I(x) { \
  char buff[256];\
  snprintf(buff,sizeof(buff),"[log] %s::%d:",__func__,__LINE__);\
  Serial.print(buff);\
  Serial.print(#x);\
  Serial.print(":=");\
  Serial.print(x);\
  Serial.println(">");\
}
#define LOG_F(x) { \
  char buff[256];\
  snprintf(buff,sizeof(buff),"[log] %s::%d:",__func__,__LINE__);\
  Serial.print(buff);\
  Serial.print(#x);\
  Serial.print(":=");\
  Serial.print(x);\
  Serial.println(">");\
}
#define LOG_S(x) { \
  char buff[256];\
  snprintf(buff,sizeof(buff),"[log] %s::%d:",__func__,__LINE__);\
  Serial.print(buff);\
  Serial.print(#x);\
  Serial.print(":=");\
  Serial.print(x);\
  Serial.println(">");\
}

void readData(void);

static const int32_t iConstOnOffCounterLoop = 40;
static const int32_t iConstOnOffCounterSwith = iConstOnOffCounterLoop/2;
void loop() {
  //Serial.println("loop");
  delay(100);

  static int32_t counterSkip = 0;
  if(counterSkip% iConstOnOffCounterLoop > iConstOnOffCounterSwith/2) {
    //LOG_I(counterSkip);
    digitalWrite(FETSWITCHPin,LOW);
  } else {
    //LOG_I(counterSkip);
    digitalWrite(FETSWITCHPin,HIGH);
  }
  counterSkip++;
  readData();
}
void readData(void){
  bool isGood = ina219.success();
  if(isGood == false) {
    return;
  }
  float shuntMiliVoltage = ina219.getShuntVoltage_mV();
  float busMiliVoltage = 1000.0f * ina219.getBusVoltage_V();
  float loadMiliVoltage = busMiliVoltage + shuntMiliVoltage;
  float current_mA = ina219.getCurrent_mA();
  float power_mW = ina219.getPower_mW();
  if(power_mW > 0) {
    LOG_F(shuntMiliVoltage);
    LOG_F(busMiliVoltage);
    LOG_F(loadMiliVoltage);
    LOG_F(current_mA);
    LOG_F(power_mW);
    int temperatureRaw = analogRead(TempAnalogPin);
    //LOG_I(temperatureRaw);
    float temeratureVoltage = map(temperatureRaw, 0, 1023, 0, 5000);
    //LOG_F(temeratureVoltage);
    float temerature = map(temeratureVoltage, 100, 1750, -40, 125);
    LOG_F(temerature);
  }
}