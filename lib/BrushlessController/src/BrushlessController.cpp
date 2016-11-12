#include "BrushlessController.h"

#define PWM_A_MOTOR1 OCR2A
#define PWM_B_MOTOR1 OCR1B
#define PWM_C_MOTOR1 OCR1A

#define PWM_A_MOTOR0 OCR0A
#define PWM_B_MOTOR0 OCR0B
#define PWM_C_MOTOR0 OCR2B

#define MOTORUPDATE_FREQ 500
#define CC_FACTOR 32

uint8_t freqCounter = 0;

uint8_t pwm_a_motor0 = 0;
uint8_t pwm_b_motor0 = 0;
uint8_t pwm_c_motor0 = 0;

uint8_t pwm_a_motor1 = 0;
uint8_t pwm_b_motor1 = 0;
uint8_t pwm_c_motor1 = 0;

uint16_t posStep;

ISR(TIMER1_OVF_vect)
{
    freqCounter++;
    if(freqCounter==(CC_FACTOR*1000/MOTORUPDATE_FREQ))
    {
      freqCounter=0;

      posStep ++;
      //
    //   PWM_A_MOTOR0 = pwm_a_motor0;
    //   PWM_B_MOTOR0 = pwm_b_motor0;
    //   PWM_C_MOTOR0 = pwm_c_motor0;
      //
    //   PWM_A_MOTOR1 = pwm_a_motor1;
    //   PWM_B_MOTOR1 = pwm_b_motor1;
    //   PWM_C_MOTOR1 = pwm_c_motor1;

    //   // update event
    //   motorUpdate = true;
    }

    // // care for standard timers every 1 ms
    // if ((freqCounter & 0x01f) == 0) {
    //     TIMER0_isr_emulation();
    // }

}

void BrushlessController::MoveMotorPosSpeed(uint8_t motorNumber, int MotorPos, uint16_t power)
// void MoveMotors(uint8_t motorNumber, uint8_t posStep, uint16_t power)
{
  uint16_t pwm_a;
  uint16_t pwm_b;
  uint16_t pwm_c;

  // lookup sine values from table with 120deg. offsets
  pwm_a = pwmSinMotor[(uint8_t)posStep];
  pwm_b = pwmSinMotor[(uint8_t)(posStep + 85)];
  pwm_c = pwmSinMotor[(uint8_t)(posStep + 170)];

  // scale motor power
  pwm_a = power * pwm_a;
  pwm_a = pwm_a >> 8;
  pwm_a += 128;

  pwm_b = power * pwm_b;
  pwm_b = pwm_b >> 8;
  pwm_b += 128;

  pwm_c = power * pwm_c;
  pwm_c = pwm_c >> 8;
  pwm_c += 128;

  // set motor pwm variables
  if (motorNumber == 0)
  {
    PWM_A_MOTOR0 = (uint8_t)pwm_a;
    PWM_B_MOTOR0 = (uint8_t)pwm_b;
    PWM_C_MOTOR0 = (uint8_t)pwm_c;
  }

  if (motorNumber == 1)
  {
    PWM_A_MOTOR1 = (uint8_t)pwm_a;
    PWM_B_MOTOR1 = (uint8_t)pwm_b;
    PWM_C_MOTOR1 = (uint8_t)pwm_c;
  }
}

// void BrushlessController::MoveMotorPosSpeed(uint8_t motorNumber, int MotorPos, uint16_t maxPWM)
// {
//   uint16_t posStep;
//   uint16_t pwm_a;
//   uint16_t pwm_b;
//   uint16_t pwm_c;
//
//   // fetch pwm from sinus table
//   // posStep = MotorPos & 0xff;
//   posStep = 42;
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

BrushlessController::BrushlessController()
{
    cli();

    pinMode(3, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);
    pinMode(11, OUTPUT);

    TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00);
    TCCR0B = _BV(CS00);
    TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10);
    TCCR1B = _BV(CS10);
    TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20);
    TCCR2B = _BV(CS20);

    // enable Timer 1 interrupt
    TIMSK1 |= _BV(TOIE1);

    // disable arduino standard timer interrupt
    TIMSK0 &= ~_BV(TOIE1);

    // Enable Timer1 Interrupt for Motor Control
    OCR2A = 0;  //11  APIN
    OCR2B = 0;  //D3
    OCR1A = 0;  //D9  CPIN
    OCR1B = 0;  //D10 BPIN
    OCR0A = 0;  //D6
    OCR0B = 0;  //D5

    // Prepare sinus values
    for(int i=0; i<256; i++)
        pwmSinMotor[i] =  sin(2.0 * i / 256 * M_PI) * 127.0;

    sei();
}
