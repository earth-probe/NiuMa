#include <Arduino.h>
#include "debug.hpp"
void setupIMU(void);
void readIMU(void);


void IMUTask( void * parameter) {
  int core = xPortGetCoreID();
  LOG_I(core);
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
  auto goodWire = Wire.begin(SDA,SCL,100*1000);
  LOG_I(goodWire);
  auto isGoodImu = imu.begin(LSM9DS1_AG, LSM9DS1_M, Wire);
  //auto isGoodImu = imu.begin();
  LOG_I(isGoodImu);
  imu.calibrateMag();
  imu.setMagScale(8);
}

static const long constReadImuIntervalMS = 16;
volatile float accX;
volatile float accY;
volatile float accZ;
volatile float gyroX;
volatile float gyroY;
volatile float gyroZ;

volatile float magnetX;
volatile float magnetY;
volatile float magnetZ;

volatile float magnet4SteeringX = 0.0;
volatile float magnet4SteeringY = 0.0;
volatile float magnet4SteeringZ = 0.0;
static const float magnetFilterFatorA = 0.2; 
static const float magnetFilterFatorB = 1.0 - magnetFilterFatorA; 



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

  magnet4SteeringX = magnet4SteeringX * magnetFilterFatorA + magnetX * magnetFilterFatorB;
  magnet4SteeringY = magnet4SteeringY * magnetFilterFatorA + magnetY * magnetFilterFatorB;
  magnet4SteeringZ = magnet4SteeringZ * magnetFilterFatorA + magnetZ * magnetFilterFatorB;
  
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

