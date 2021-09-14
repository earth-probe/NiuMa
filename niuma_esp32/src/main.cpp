#include <Arduino.h>
//#include <ArduinoJson.h>

#define DUMP_I(x) { \
  Serial.printf("%s::%d:%s=<%d>\r\n",__func__,__LINE__,#x,x);\
}
#define DUMP_F(x) { \
  Serial.printf("%s::%d:%s=<%f>\r\n",__func__,__LINE__,#x,x);\
}
#define DUMP_S(x) { \
  Serial.printf("%s::%d:%s=<%s>\r\n",__func__,__LINE__,#x,x.c_str());\
}
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



void MotorTask( void * parameter) {
  int core = xPortGetCoreID();
  DUMP_I(core);
  for(;;) {//
    float degree = temperatureRead();
    //DUMP_F(degree);
    delay(1000);
  }
}



#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
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

void runBleTransimit(void)
{
  int hallValue = hallRead();
  if (deviceConnected) {
    DUMP_I(hallValue);
    DUMP_I(pTxCharacteristic);
    std::string halVal = "fdfdad\r\n";
    pTxCharacteristic->setValue(halVal);
    pTxCharacteristic->notify();
    delay(1000); // bluetooth stack will go into congestion, if too many packets are sent
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
