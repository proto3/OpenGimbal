#ifndef IMU_H_
#define IMU_H_

#include <Arduino.h>
#include "MPU6050.h"

void initMPU();
void initIMUtc();
void initIMU();
void updateAngleOffset();
void initSensorOrientation();
bool initI2C();
void readGyros();
void updateGyroAttitude();
void updateACCAttitude();
void getAttiduteAngles();
void readACCs();
void updateACC();
void setACCtc(int16_t accTimeConstant);

extern MPU6050 mpu;

#endif
