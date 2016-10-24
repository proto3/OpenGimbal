#ifndef _VARIABLES_H_
#define _VARIABLES_H_

#include "Trace.h"
#include "definitions.h"

#define VERSION_STATUS B // A = Alpha; B = Beta , N = Normal Release
#define VERSION "v50"
#define REVISION "r217"
#define VERSION_EEPROM 17 // change this number when eeprom data structure has changed

/*************************/
/* Config Structure      */
/*************************/

struct config_t
{
  uint8_t versEEPROM;
  uint8_t configSet;
  int32_t gyroPitchKp;
  int32_t gyroPitchKi;
  int32_t gyroPitchKd;
  int32_t gyroRollKp;
  int32_t gyroRollKi;
  int32_t gyroRollKd;
  int16_t accTimeConstant;
  int16_t angleOffsetPitch;   // angle offset, deg*100
  int16_t angleOffsetRoll;
  int8_t dirMotorPitch;
  int8_t dirMotorRoll;
  uint8_t motorNumberPitch;
  uint8_t motorNumberRoll;
  uint8_t maxPWMmotorPitch;
  uint8_t maxPWMmotorRoll;
  uint16_t refVoltageBat;    // Ubat reference, unit = volts*100
  uint16_t cutoffVoltage;    // Ubat cutoff, unit = volts*100
  bool motorPowerScale;
  bool rcAbsolutePitch;
  bool rcAbsoluteRoll;
  int8_t maxRCPitch;
  int8_t maxRCRoll;
  int8_t minRCPitch;
  int8_t minRCRoll;
  int16_t rcGainPitch;
  int16_t rcGainRoll;
  int16_t rcLPFPitch;        // low pass filter for RC absolute mode, units=1/10 sec
  int16_t rcLPFRoll;
  bool rcModePPMPitch;       // RC mode, true=common RC PPM channel, false=separate RC channels
  bool rcModePPMRoll;
  bool rcModePPMAux;
  bool rcModePPMFpvPitch;
  bool rcModePPMFpvRoll;

  uint8_t rcPinModeCH0;      // 0=Channel OFF, 1=use digital PWM/PPM control, 2=use analog control
  uint8_t rcPinModeCH1;      //
  uint8_t rcPinModeCH2;      //

  int8_t rcChannelPitch;     // input channel for pitch
  int8_t rcChannelRoll;      // input channel for roll
  int8_t rcChannelAux;       // input channel for auxiliary functions
  int8_t rcChannelFpvPitch;  // input channel for fpv channel pitch
  int8_t rcChannelFpvRoll;   // input channel for fpv channel roll

  int8_t rcChannelPt0;      // rc channel for passthrough output 0
  int8_t rcChannelPt1;      // rc channel for passthrough output 1


  int8_t fpvGainPitch;       // gain of FPV channel pitch
  int8_t fpvGainRoll;        // gain of FPV channel roll

  int16_t rcLPFPitchFpv;     // low pass filter in FPV mode
  int16_t rcLPFRollFpv;      // low pass filter in FPV mode

  int16_t rcMid;             // rc channel center ms

  traceModeType fTrace;       // trace output mode (uint8_t)
  traceModeType sTrace;       // trace output mode (uint8_t)

  bool enableGyro;           // enable gyro attitude update
  bool enableACC;            // enable acc attitude update
  bool axisReverseZ;
  bool axisSwapXY;

  bool fpvFreezePitch;
  bool fpvFreezeRoll;

  uint8_t maxPWMfpvPitch;    // motor PWM power in FPV freeze mode
  uint8_t maxPWMfpvRoll;

  int8_t fpvSwPitch;         // fpv switch pitch: -1=always on, 0=off, 1=auxSW1, 2=auxSW2
  int8_t fpvSwRoll;          // fpv switch roll: -1=alwas on, 0=off, 1=auxSW1, 2=auxSW2
  int8_t altSwAccTime;       // switch alternate Acc time: -1=always on, 0=off, 1=auxSW1, 2=auxSW2
  int16_t accTimeConstant2;  // alternate constant

  bool gyroCal;              // gyro calibration at startup
  int16_t  gyrOffsetX;       // gyyro calibration offsets
  int16_t  gyrOffsetY;
  int16_t  gyrOffsetZ;

  int16_t  accOffsetX;       // acc calibration offsets
  int16_t  accOffsetY;
  int16_t  accOffsetZ;

  uint8_t crc8;
};

extern struct config_t config;

void setDefaultParameters();

typedef struct PIDdata {
  int32_t   Kp, Ki, Kd;
} PIDdata_t;

extern PIDdata_t pitchPIDpar,rollPIDpar;

void initPIDs();

static int32_t pitchErrorSum = 0;
static int32_t rollErrorSum = 0;
static int32_t pitchErrorOld = 0;
static int32_t rollErrorOld = 0;


// CRC definitions
#define POLYNOMIAL 0xD8  /* 11011 followed by 0's */
typedef uint8_t crc;



/*************************/
/* Variables             */
/*************************/



// motor drive

extern int8_t pwmSinMotor[256];

extern int currentStepMotor0;
extern int currentStepMotor1;
extern bool motorUpdate;

extern int8_t pitchDirection;
extern int8_t rollDirection;

extern uint8_t freqCounter;

extern int pitchMotorDrive;
extern int rollMotorDrive;

// control motor update in ISR
extern bool enableMotorUpdates;

extern uint8_t pwm_a_motor0;
extern uint8_t pwm_b_motor0;
extern uint8_t pwm_c_motor0;

extern uint8_t pwm_a_motor1;
extern uint8_t pwm_b_motor1;
extern uint8_t pwm_c_motor1;

// battery voltage
extern float voltageBat;
extern float uBatValue_f;


//scaled Motor Power
extern uint16_t maxPWMmotorPitchScaled;
extern uint16_t maxPWMmotorRollScaled;

// Variables for MPU6050
extern float gyroPitch;
extern float gyroRoll; //in deg/s

extern float resolutionDevider;
extern int16_t x_val;
extern int16_t y_val;
extern int16_t z_val;

extern float PitchPhiSet;
extern float RollPhiSet;
static float pitchAngleSet=0;
static float rollAngleSet=0;

static float qLPPitch[3] = {0,0,0};
static float qLPRoll[3] = {0,0,0};


extern int count;

// RC control

struct rcData_t
{
 uint32_t microsRisingEdge;
 uint32_t microsLastUpdate;
 uint16_t rx;
 uint16_t rx1;
 bool     update;
 bool     valid;
 float    rcSpeed;
 float    setpoint;
 bool     rcAuxSwitch1;
 bool     rcAuxSwitch2;
};

extern rcData_t rcData[RC_DATA_SIZE];

extern float rcLPFPitch_tc;
extern float rcLPFRoll_tc;

extern float rcLPFPitchFpv_tc;
extern float rcLPFRollFpv_tc;

// Gimbal State
enum gimStateType {
 GIM_IDLE=0,      // no PID
 GIM_UNLOCKED,    // PID on, fast ACC
 GIM_LOCKED,      // PID on, slow ACC
 GIM_ERROR        // error condition
};

extern gimStateType gimState;
extern int stateCount;

// rc fpv mode
extern bool fpvModePitch;
extern bool fpvModeRoll;

extern bool fpvModeFreezePitch;
extern bool fpvModeFreezeRoll;

// rc alternate ACC time
extern bool altModeAccTime;

//*************************************
//
//  IMU
//
//*************************************
enum axisDef {
  ROLL,
  PITCH,
  YAW
};

typedef struct fp_vector {
  float X;
  float Y;
  float Z;
} t_fp_vector_def;

typedef union {
  float   A[3];
  t_fp_vector_def V;
} t_fp_vector;



//********************
// sensor orientation
//********************
typedef struct sensorAxisDef {
  char idx;
  int  dir;
} t_sensorAxisDef;

typedef struct sensorOrientationDef {
  t_sensorAxisDef Gyro[3];
  t_sensorAxisDef Acc[3];
} t_sensorOrientationDef;

extern t_sensorOrientationDef sensorDef;


static float gyroScale=0;

static int16_t gyroADC[3];
static int16_t accADC[3];

static t_fp_vector EstG;

static float accLPF[3] = {0.0,0.0,0.0};
static float accMag = 0;
static bool disableAccGtest = false;

static float AccComplFilterConst = 0;  // filter constant for complementary filter

static int16_t acc_25deg = 25;      //** TODO: check

static int32_t angle[2]    = {0,0};  // absolute angle inclination in multiple of 0.01 degree    180 deg = 18000

static float angleOffsetRoll_f = 0;
static float angleOffsetPitch_f = 0;

static int32_t angleOffsetRoll = 0;
static int32_t angleOffsetPitch = 0;


// DEBUG only
extern uint32_t stackTop;
extern uint32_t stackBottom;

extern uint32_t heapTop;
extern uint32_t heapBottom;

#endif
