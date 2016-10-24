#ifndef _TIMER1_H_
#define _TIMER1_H_

/* prototypes for Timer1 based timing functions */

void TIMER0_isr_emulation();
unsigned long millisT1();
unsigned long microsT1();
void delayT1(unsigned long ms);

#endif
