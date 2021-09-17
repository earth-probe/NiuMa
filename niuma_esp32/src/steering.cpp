#include <Arduino.h>
#include "debug.h"
void setupIMU(void);
void readIMU(void);

void SteeringMotorTask( void * parameter) {
  int core = xPortGetCoreID();
  DUMP_I(core);
  setupIMU();
  for(;;) {//
    readIMU();
    delay(1);
  }
}

#include <Wire.h>
#include <SPI.h>

#include <SparkFunLSM9DS1.h>
LSM9DS1 imu;
#define LSM9DS1_M 0x1C
#define LSM9DS1_AG 0x6A 
void setupIMU(void) {
  Wire.begin();
  auto isGood = imu.begin(LSM9DS1_AG, LSM9DS1_M, Wire);
  DUMP_I(isGood);  
}

static const long constReadImuIntervalMS = 1000;
volatile float accX;
volatile float accY;
volatile float accZ;
volatile float gyroX;
volatile float gyroY;
volatile float gyroZ;
volatile float magnetX;
volatile float magnetY;
volatile float magnetZ;

/*
extern QueueHandle_t axQueue;
extern QueueHandle_t ayQueue;
extern QueueHandle_t azQueue;
*/


void readIMU(void) {
  static long previousMillis = 0;
  auto nowMS = millis();
  if(nowMS - previousMillis < constReadImuIntervalMS) {
    return;
  }
  previousMillis = nowMS;
  if ( imu.gyroAvailable() )
  {
    imu.readGyro();
  }
  if ( imu.accelAvailable() )
  {
    imu.readAccel();
  }
  if ( imu.magAvailable() )
  {
    imu.readMag();
  }
  if ( imu.tempAvailable() )
  {
    imu.readTemp();
  }

  accX = imu.calcAccel(imu.ax);
//  auto axStatus = xQueueSend(axQueue, &accX, 0);
//  DUMP_I(axStatus)
  accY = imu.calcAccel(imu.ay);
//  auto ayStatus = xQueueSend(ayQueue, &accY, 0);  
//  DUMP_I(ayStatus)
  accZ = imu.calcAccel(imu.az);
//  auto azStatus = xQueueSend(azQueue, &accZ, 0);
//  DUMP_I(azStatus)

  magnetX = imu.calcMag(imu.mx);
  magnetY = imu.calcMag(imu.my);
  magnetZ = imu.calcMag(imu.mz);
  gyroX = imu.calcGyro(imu.gx);
  gyroY = imu.calcGyro(imu.gy);
  gyroZ = imu.calcGyro(imu.gz);
#if 1 
  DUMP_F(accX);
  DUMP_F(accY);
  DUMP_F(accZ);

  DUMP_F(gyroX);
  DUMP_F(gyroY);
  DUMP_F(gyroZ);

  DUMP_F(magnetX);
  DUMP_F(magnetY);
  DUMP_F(magnetZ);
#endif

}
