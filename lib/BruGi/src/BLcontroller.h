#ifndef _BL_CONTROLLER_H_
#define _BL_CONTROLLER_H_

#include <Arduino.h>

void initBlController();
void MoveMotorPosSpeed(uint8_t motorNumber, int MotorPos, uint16_t maxPWM);
void calcSinusArray();
void initMotorStuff();

// ISR( TIMER1_OVF_vect )
// {
//   freqCounter++;
//   if(freqCounter==(CC_FACTOR*1000/MOTORUPDATE_FREQ))
//   {
//     freqCounter=0;
//
//     PWM_A_MOTOR0 = pwm_a_motor0;
//     PWM_B_MOTOR0 = pwm_b_motor0;
//     PWM_C_MOTOR0 = pwm_c_motor0;
//
//     PWM_A_MOTOR1 = pwm_a_motor1;
//     PWM_B_MOTOR1 = pwm_b_motor1;
//     PWM_C_MOTOR1 = pwm_c_motor1;
//
//     // update event
//     motorUpdate = true;
//   }
//
//
//   // care for standard timers every 1 ms
//   if ((freqCounter & 0x01f) == 0) {
//     TIMER0_isr_emulation();
//   }
//
//
// }

void voltageCompensation();
void motorPowerOff();

#endif
