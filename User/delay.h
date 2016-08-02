#ifndef _DELAY_H_
#define _DELAY_H_
#include "stm32f10x.h"
//extern __IO uint32_t TimingDelay;

extern volatile uint32_t magdelaycount;
extern volatile uint32_t presdelaycount;

void delay_us(__IO uint32_t usnTime);
void delay_ms(__IO uint32_t msnTime);
void delay_noInt_ms(uint32_t time);

#endif
