#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#include <ArduinoJson.h>

#include <sstream>
#include "debug.hpp"

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      LOG_I(deviceConnected);
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      LOG_I(deviceConnected);
    }
};

void refreshExternDrivecommand(float speed,bool dir,bool brake);
void refreshExternSteeringcommand(float angle,bool brake);

void onExternalCommand(StaticJsonDocument<256> &doc) {
  if(doc.containsKey("forward")) {
    auto forward = doc["forward"];
    if(forward.containsKey("speed")) {
      float speed = forward["speed"].as<float>();
      LOG_F(speed);
      refreshExternDrivecommand(speed,true,false);
    }
  }
  if(doc.containsKey("backward")) {
    auto backward = doc["backward"];
    if(backward.containsKey("speed")) {
      float speed = backward["speed"].as<float>();
      LOG_F(speed);
      refreshExternDrivecommand(speed,false,false);
    }
  }
  if(doc.containsKey("turn")) {
    auto turn = doc["turn"];
    if(turn.containsKey("angle")) {
      float angle = turn["angle"].as<float>();
      LOG_F(angle);
      refreshExternSteeringcommand(angle,false);
    }
  }
  if(doc.containsKey("stop")) {
    auto stop = doc["stop"];
    if(stop.containsKey("all")) {
      bool all = stop["all"].as<bool>();
      LOG_I(all);
      refreshExternDrivecommand(0.0,false,true);
      refreshExternSteeringcommand(0.0,true);
    }
  }
}

class MyCallbacks: public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *pCharacteristic) {
    //pCharacteristic->setValue("Hello World!");
    std::string value = pCharacteristic->getValue();
  }

  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    LOG_S(value);
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, value);
    LOG_S(error);
    if(error == DeserializationError::Ok) {
      onExternalCommand(doc);
    }
  }
};


#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_TX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


void setupBLE(void) {
  BLEDevice::init("ESP32 NiuMa");  // local name
  pServer = BLEDevice::createServer();  // Create the BLE Device
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX,BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX,BLECharacteristic::PROPERTY_WRITE										);
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();
  pServer->getAdvertising()->start();

  Serial.println("Waiting a client connection to notify...");
}

std::string my_to_string(int i) {
  std::stringstream ss;
  ss << i;
  return ss.str();
}

std::string my_to_string_f(float f) {
  std::stringstream ss;
  ss << f;
  return ss.str();
}


extern float accX;
extern float accY;
extern float accZ;
extern float gyroX;
extern float gyroY;
extern float gyroZ;
extern float magnetX;
extern float magnetY;
extern float magnetZ;

static const long constWriteBLEIntervalMS = 100;


void reportIMU(void)
{
/*  
  {
    std::string imuVal = "{\"ax\":";
    imuVal += my_to_string_f(accX);
    imuVal += "}\r\n";
    pTxCharacteristic->setValue(imuVal);
    pTxCharacteristic->notify();
  }
  {
    std::string imuVal = "{\"ay\":";
    imuVal += my_to_string_f(accY);
    imuVal += "}\r\n";
    pTxCharacteristic->setValue(imuVal);
    pTxCharacteristic->notify();
  }
  {
    std::string imuVal = "{\"az\":";
    imuVal += my_to_string_f(accZ);
    imuVal += "}\r\n";
    pTxCharacteristic->setValue(imuVal);
    pTxCharacteristic->notify();
  }
  {
    std::string imuVal = "{\"gx\":";
    imuVal += my_to_string_f(gyroX);
    imuVal += "}\r\n";
    pTxCharacteristic->setValue(imuVal);
    pTxCharacteristic->notify();
  }
  {
    std::string imuVal = "{\"gy\":";
    imuVal += my_to_string_f(gyroY);
    imuVal += "}\r\n";
    pTxCharacteristic->setValue(imuVal);
    pTxCharacteristic->notify();
  }
  {
    std::string imuVal = "{\"gz\":";
    imuVal += my_to_string_f(gyroZ);
    imuVal += "}\r\n";
    pTxCharacteristic->setValue(imuVal);
    pTxCharacteristic->notify();
  }
*/
  {
    std::string imuVal = "{\"mx\":";
    imuVal += my_to_string_f(magnetX);
    imuVal += "}\r\n";
    pTxCharacteristic->setValue(imuVal);
    pTxCharacteristic->notify();
  }
  {
    std::string imuVal = "{\"my\":";
    imuVal += my_to_string_f(magnetY);
    imuVal += "}\r\n";
    pTxCharacteristic->setValue(imuVal);
    pTxCharacteristic->notify();
  }
  {
    std::string imuVal = "{\"mz\":";
    imuVal += my_to_string_f(magnetZ);
    imuVal += "}\r\n";
    pTxCharacteristic->setValue(imuVal);
    pTxCharacteristic->notify();
  }
}

void runBleTransimit(void)
{
  if (deviceConnected) {
    static long previousMillis = 0;
    auto nowMS = millis();
    if(nowMS - previousMillis < constWriteBLEIntervalMS) {
      return;
    }
    previousMillis = nowMS;
    reportIMU();
  }
  if (!deviceConnected && oldDeviceConnected) {
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
  // do stuff here on connecting
      oldDeviceConnected = deviceConnected;
  }
}

void BLETask( void * parameter) {
  int core = xPortGetCoreID();
  LOG_I(core);
  setupBLE();
  for(;;) {//
    runBleTransimit();
    delay(1);
  }
}
