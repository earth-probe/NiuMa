#include <Arduino.h>
#include "debug.h"
void setupIMU(void);
void readIMU(void);

void setupSteeringMotor(void);
void execSteeringMotor(void);


void SteeringMotorTask( void * parameter) {
  int core = xPortGetCoreID();
  LOG_I(core);
  setupSteeringMotor();
  setupIMU();
  for(;;) {//
    readIMU();
    execSteeringMotor();
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


static const uint8_t iConstPinExtend = GPIO_NUM_17; 
static const uint8_t iConstPinReduce = GPIO_NUM_18;
static const uint8_t iConstPinSteeringLevelOE = GPIO_NUM_19;

void setupSteeringMotor(void) {
  pinMode(iConstPinExtend, OUTPUT);
  pinMode(iConstPinReduce, OUTPUT);

  pinMode(iConstPinSteeringLevelOE, OUTPUT);
  digitalWrite(iConstPinSteeringLevelOE,1);
}

static auto gMilliSecAtLastCommand = millis();
volatile uint8_t gDriveMotorExtend = 1;
volatile uint8_t gDriveMotorReduce = 1;
void refreshExternSteeringcommand(float angle,bool brake) {
  if(brake) {
    gDriveMotorExtend = 1;
    gDriveMotorReduce = 1;
  } else {
    if(angle > 0) {
      gDriveMotorExtend = 0;
      gDriveMotorReduce = 1;
    } else {
      gDriveMotorExtend = 1;
      gDriveMotorReduce = 0;
    }
  }
  gMilliSecAtLastCommand = millis();
  LOG_I(gDriveMotorExtend);
  LOG_I(gDriveMotorReduce);
}

static const long constSteeringMotorIntervalMS = 200; 


void execSteeringMotor(void) {
  /*
  static long previousMillis = 0;
  auto nowMS = millis();
  if(nowMS - previousMillis < constSteeringMotorIntervalMS) {
    return;
  }
  previousMillis = nowMS;
  */

  DUMP_I(gDriveMotorExtend);  
  DUMP_I(gDriveMotorReduce);  
  digitalWrite(iConstPinExtend,gDriveMotorExtend);
  digitalWrite(iConstPinReduce,gDriveMotorReduce);
}
