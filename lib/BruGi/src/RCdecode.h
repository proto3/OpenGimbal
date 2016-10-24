// #ifndef _RC_DECODE_H_
// #define _RC_DECODE_H_
//
// #include <Arduino.h>
// #include "variables.h"
//
// // init RC config variables
// void initRC();
//
// // pinChange Int driven Functions
// inline void decodePWM( uint32_t microsIsrEnter, bool risingEdge, rcData_t* rcData );
//
// inline void intDecodePPM( uint32_t microsIsrEnter );
//
// // Connector Channel 1 (A2)
// void intDecodePWM_Ch0();
//
// // Connector Channel 2 (A1)
// void intDecodePWM_Ch1();
//
// // Connector Channel 3 (A0)
// void intDecodePWM_Ch2();
//
// // check for RC timout
// void checkRcTimeouts();
//
// // initialize RC Pin mode
// void initRCPins();
//
// void evalRCChannelProportional(rcData_t* rcData, int16_t rcGain, int16_t rcMid);
//
// void evalRCChannelAbsolute(rcData_t* rcData, int8_t gain, int8_t rcMin, int8_t rcMax, int16_t rcMid);
//
// void evaluateRCPitch();
//
// void evaluateRCRoll();
//
// void getSetpoint(float * setPoint, unsigned char rcChannel, unsigned char rcChannelFpv, bool fpvMode, bool rcAbsolute, int8_t maxRC, int8_t minRC);
//
// // auxiliary channel, decode function switches
// void evalRCChannelAux(rcData_t* rcData, int16_t rcSwThresh, int16_t rcMid);
//
// // auxiliary
// void evaluateRCAux();
//
// // decode fpv switch selector
// bool decodeSWSel(int8_t configSelector);
//
// void decodeModeSwitches();
//
// void readRCAnalogPin(rcData_t* rcData, bool rcModePPM, uint8_t rcChannel);
//
// void readRCAnalog();
//
// #endif
