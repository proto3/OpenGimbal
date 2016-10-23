#ifndef BL_CONTROLLER_H
#define BL_CONTROLLER_H

void initBlController();
void motorPowerOff();
void MoveMotorPosSpeed(uint8_t motorNumber, int MotorPos, uint16_t maxPWM);

// void MoveMotorPosSpeed(uint8_t motorNumber, int MotorPos, uint16_t maxPWM)
// {
//   uint16_t posStep;
//   uint16_t pwm_a;
//   uint16_t pwm_b;
//   uint16_t pwm_c;
//
//   // fetch pwm from sinus table
//   posStep = MotorPos & 0xff;
//   pwm_a = pwmSinMotor[(uint8_t)posStep];
//   pwm_b = pwmSinMotor[(uint8_t)(posStep + 85)];
//   pwm_c = pwmSinMotor[(uint8_t)(posStep + 170)];
//
//   // apply power factor
//   pwm_a = maxPWM * pwm_a;
//   pwm_a = pwm_a >> 8;
//   pwm_a += 128;
//
//   pwm_b = maxPWM * pwm_b;
//   pwm_b = pwm_b >> 8;
//   pwm_b += 128;
//
//   pwm_c = maxPWM * pwm_c;
//   pwm_c = pwm_c >> 8;
//   pwm_c += 128;
//
//   // set motor pwm variables
//   if (motorNumber == 0)
//   {
//     pwm_a_motor0 = (uint8_t)pwm_a;
//     pwm_b_motor0 = (uint8_t)pwm_b;
//     pwm_c_motor0 = (uint8_t)pwm_c;
//   }
//
//   if (motorNumber == 1)
//   {
//     pwm_a_motor1 = (uint8_t)pwm_a;
//     pwm_b_motor1 = (uint8_t)pwm_b;
//     pwm_c_motor1 = (uint8_t)pwm_c;
//   }
// }
//
//
// void calcSinusArray()
// {
//   for(int i=0; i<N_SIN; i++)
//   {
//     pwmSinMotor[i] =  sin(2.0 * i / N_SIN * 3.14159265) * 127.0;
//   }
// }
//
// void initMotorStuff()
// {
//   cli();
//   calcSinusArray();
//   sei();
// }
//
// /********************************/
// /* Motor Control IRQ Routine    */
// /********************************/
// // is called every 31.5us
// // minimize interrupt code (20 instructions)
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
//
//
// /**********************************************************/
// /* voltage compensation                                   */
// /*   measure power supply voltage and compensate          */
// /*   motor power accordingly                              */
// /**********************************************************/
// void voltageCompensation () {
//   int uBatValue;
//   float pwmMotorScale;
//   static bool cutOffActive = false;
//
//   // measure uBat, 190 us
//   uBatValue = analogRead(ADC_VCC_PIN); // 118 us
//   uBatValue_f = (float)uBatValue * UBAT_ADC_SCALE * UBAT_SCALE;
//   utilLP_float(&voltageBat, uBatValue_f, LOWPASS_K_FLOAT(0.1)); // tau = 1 sec
//
//   if (config.motorPowerScale) {
//     uint16_t cutOffVoltage = config.cutoffVoltage;
//     if (cutOffActive) {
//       cutOffVoltage +=  cutOffVoltage >> 4; // 1/16 = 6,25% hysteresis
//     }
//     // calculate scale factor for motor power (70us)
//     if (voltageBat*100 > cutOffVoltage) {  // switch off if battery voltage below cutoff
//       pwmMotorScale = (config.refVoltageBat * 0.01)/voltageBat;
//       cutOffActive = false;
//     } else {
//       pwmMotorScale = 0;
//       cutOffActive = true;
//     }
//   } else {
//     pwmMotorScale = 1.0;
//   }
//
//   // 44us
//   if (fpvModeFreezePitch==true) {
//     maxPWMmotorPitchScaled = config.maxPWMfpvPitch * pwmMotorScale;  // fpv freeze mode
//   } else {
//     maxPWMmotorPitchScaled = config.maxPWMmotorPitch * pwmMotorScale;
//   }
//   maxPWMmotorPitchScaled = constrain(maxPWMmotorPitchScaled, 0, 255);
//
//   if (fpvModeFreezeRoll==true) {
//     maxPWMmotorRollScaled = config.maxPWMfpvRoll * pwmMotorScale; // fpv freeze mode
//   } else {
//     maxPWMmotorRollScaled = config.maxPWMmotorRoll * pwmMotorScale;
//   }
//   maxPWMmotorRollScaled = constrain(maxPWMmotorRollScaled, 0, 255);
//
// }
//
//
// // switch off motor power
// void motorPowerOff() {
//   MoveMotorPosSpeed(config.motorNumberPitch, 0, 0);
//   MoveMotorPosSpeed(config.motorNumberRoll, 0, 0);
// }
#endif
