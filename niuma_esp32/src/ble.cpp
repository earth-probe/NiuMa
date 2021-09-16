#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <sstream>
#include "debug.h"

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      DUMP_I(deviceConnected);
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      DUMP_I(deviceConnected);
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *pCharacteristic) {
    //pCharacteristic->setValue("Hello World!");
    std::string value = pCharacteristic->getValue();
  }

  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    DUMP_S(value);
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


extern float accXTask0;
extern float accYTask0;
extern float accZTask0;
extern float gyroXTask0;
extern float gyroYTask0;
extern float gyroZTask0;
extern float magnetXTask0;
extern float magnetYTask0;
extern float magnetZTask0;



static const long constWriteBLEIntervalMS = 100;

void runBleTransimit(void)
{
  if (deviceConnected) {
    /*
    {
      int hallValue = hallRead();
      DUMP_I(hallValue);
      DUMP_I(pTxCharacteristic);
      std::string halVal = "{h:";
      halVal += my_to_string(hallValue);
      halVal += "}\r\n";
      pTxCharacteristic->setValue(halVal);
      pTxCharacteristic->notify();
    }
    */
      static long previousMillis = 0;
      auto nowMS = millis();
      if(nowMS - previousMillis < constWriteBLEIntervalMS) {
        return;
      }
      previousMillis = nowMS;

    {
      std::string imuVal = "{ax:";
      imuVal += my_to_string(accXTask0);
      imuVal += "}\r\n";
      pTxCharacteristic->setValue(imuVal);
      pTxCharacteristic->notify();
    }
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
