#include "variables.h"

struct config_t config;

void setDefaultParameters()
{
  config.versEEPROM = VERSION_EEPROM;
  config.configSet = 0;
  config.gyroPitchKp = 20000;
  config.gyroPitchKi = 10000;
  config.gyroPitchKd = 40000;
  config.gyroRollKp = 20000;
  config.gyroRollKi = 8000;
  config.gyroRollKd = 30000;
  config.accTimeConstant = 7;
  config.angleOffsetPitch = 0;
  config.angleOffsetRoll = 0;
  config.dirMotorPitch = 1;
  config.dirMotorRoll = -1;
  config.motorNumberPitch = 0;
  config.motorNumberRoll = 1;
  config.maxPWMmotorPitch = 80;
  config.maxPWMmotorRoll = 80;
  config.refVoltageBat = 800;
  config.cutoffVoltage = 600;
  config.motorPowerScale = 0;
  config.rcAbsolutePitch = true;
  config.rcAbsoluteRoll = true;
  config.maxRCPitch = 30;
  config.minRCPitch = -30;
  config.maxRCRoll = 30;
  config.minRCRoll = -30;
  config.rcGainPitch = 5;
  config.rcGainRoll = 5;
  config.rcLPFPitch = 20;             // 2 sec
  config.rcLPFRoll = 20;              // 2 sec
  config.rcModePPMPitch = false;
  config.rcModePPMRoll = false;
  config.rcModePPMAux = false;
  config.rcModePPMFpvPitch = false;
  config.rcModePPMFpvRoll = false;
  config.rcChannelPitch = 1;
  config.rcChannelRoll = 0;
  config.rcChannelAux = -1;
  config.rcChannelFpvPitch = -1;
  config.rcChannelFpvRoll = -1;
  config.rcChannelPt0 = -1;
  config.rcChannelPt1 = -1;
  config.fpvGainPitch = 0;
  config.fpvGainRoll = 0;
  config.rcPinModeCH0 = 1;      // 1 = use digital control
  config.rcPinModeCH1 = 1;
  config.rcPinModeCH2 = 1;
  config.rcLPFPitchFpv = 10;  // 1 sec
  config.rcLPFRollFpv = 10;  // 1 sec
  config.rcMid = MID_RC;
  config.fTrace=TRC_OFF; // fast trace
  config.sTrace=TRC_OFF; // slow trace
  config.enableGyro=true;
  config.enableACC=true;
  config.axisReverseZ=true;
  config.axisSwapXY=false;
  config.fpvFreezePitch=false;
  config.fpvFreezeRoll=false;
  config.maxPWMfpvPitch=80;
  config.maxPWMfpvRoll=80;
  config.fpvSwPitch=0;
  config.fpvSwRoll=0;
  config.altSwAccTime=0;
  config.accTimeConstant2 = 2;
  config.gyroCal = true;
  config.gyrOffsetX = 0;       // gyyro calibration offset
  config.gyrOffsetY = 0;
  config.gyrOffsetZ = 0;
  config.accOffsetX = 0;       // acc calibration offset
  config.accOffsetY = 0;
  config.accOffsetZ = 0;
  config.crc8 = 0;
}


PIDdata_t pitchPIDpar,rollPIDpar;

void initPIDs()
{
  rollPIDpar.Kp = config.gyroRollKp/10;
  rollPIDpar.Ki = config.gyroRollKi/1000;
  rollPIDpar.Kd = config.gyroRollKd/10/250;  // divide by 250 to keep compatibility to previous version

  pitchPIDpar.Kp = config.gyroPitchKp/10;
  pitchPIDpar.Ki = config.gyroPitchKi/1000;
  pitchPIDpar.Kd = config.gyroPitchKd/10/250;  // divide by 250 to keep compatibility to previous version
}

int8_t pwmSinMotor[256];

int currentStepMotor0 = 0;
int currentStepMotor1 = 0;
bool motorUpdate = false;

int8_t pitchDirection = 1;
int8_t rollDirection = 1;

uint8_t freqCounter = 0;

int pitchMotorDrive = 0;
int rollMotorDrive = 0;

// control motor update in ISR
bool enableMotorUpdates = false;

uint8_t pwm_a_motor0 = 128;
uint8_t pwm_b_motor0 = 128;
uint8_t pwm_c_motor0 = 128;

uint8_t pwm_a_motor1 = 128;
uint8_t pwm_b_motor1 = 128;
uint8_t pwm_c_motor1 = 128;

// battery voltage
float voltageBat = 0;
float uBatValue_f = 0;

//scaled Motor Power
uint16_t maxPWMmotorPitchScaled;
uint16_t maxPWMmotorRollScaled;

// Variables for MPU6050
float gyroPitch;
float gyroRoll; //in deg/s

float resolutionDevider;
int16_t x_val;
int16_t y_val;
int16_t z_val;

float PitchPhiSet = 0;
float RollPhiSet = 0;

int count = 0;

// RC control
rcData_t rcData[RC_DATA_SIZE];

float rcLPFPitch_tc = 1.0;
float rcLPFRoll_tc = 1.0;

float rcLPFPitchFpv_tc = 1.0;
float rcLPFRollFpv_tc = 1.0;

gimStateType gimState = GIM_IDLE;
int stateCount = 0;

// rc fpv mode
bool fpvModePitch = false;
bool fpvModeRoll = false;

bool fpvModeFreezePitch = false;
bool fpvModeFreezeRoll = false;

// rc alternate ACC time
bool altModeAccTime = false;

//*************************************
//
//  IMU
//
//*************************************

//********************
// sensor orientation
//********************

t_sensorOrientationDef sensorDef = {
    {{0, 1}, {1, 1}, {2, 1}},    // Gyro
    {{0, 1}, {1, 1}, {2, 1}}     // Acc
  };

// DEBUG only
uint32_t stackTop = 0xffffffff;
uint32_t stackBottom = 0;

uint32_t heapTop = 0;
uint32_t heapBottom = 0xffffffff;
