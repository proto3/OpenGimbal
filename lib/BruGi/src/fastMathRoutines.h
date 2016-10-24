#ifndef _FAST_MATH_ROUTINES_H_
#define _FAST_MATH_ROUTINES_H_

#include <Arduino.h>
#include "variables.h"

//***************************************************************
// contrain function for 32-bit values
//***************************************************************
int32_t constrain_int32(int32_t x , int32_t l, int32_t h);

//***************************************************************
// “Efficient approximations for the arctangent function”,
// Rajan, S. Sichun Wang Inkol, R. Joyal, A., May 2006
//***************************************************************
inline float Rajan_FastArcTan(float x);

// atan2 for all quadrants by A. Hahn
inline float Rajan_FastArcTan2(float y, float x);

// atan2 returnig degrees * 1000
int32_t Rajan_FastArcTan2_deg1000(float y, float x);

/************************/
/* LP Filter            */
/************************/
void utilLP_float(float * q, float i, float coeff);

//************************
// low pass 3rd order
//************************
float utilLP3_float(float * q, float i, float coeff);

/************************/
/* Debugging            */
/************************/

inline void stackCheck();
inline void heapCheck();
inline void stackHeapEval(bool doPrint);

/*
 * The width of the CRC calculation and result.
 * Modify the typedef for a 16 or 32-bit CRC standard.
 * Thanks to
 * http://www.barrgroup.com/Embedded-Systems/How-To/CRC-Calculation-C-Code
 *
 */

#define WIDTH  (8 * sizeof(crc))
#define TOPBIT (1 << (WIDTH - 1))

crc crcSlow(uint8_t const message[], int nBytes);

#endif
