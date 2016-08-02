#include"delay.h"

__IO uint32_t TimingDelay;
volatile uint32_t magdelaycount;
volatile uint32_t presdelaycount;

/*��ʱ1us����*/
void delay_us(__IO uint32_t usnTime)
{
  TimingDelay = usnTime;
  while(TimingDelay != 0);
}


void delay_ms(__IO uint32_t msnTime)
{
  TimingDelay = msnTime * 1000;
  while(TimingDelay != 0);
}


/*stm32f10x_it.c�е�void SysTick_Handler(void)������Ϊ*/
void SysTick_Handler(void)
{
  if (TimingDelay != 0x00)
  { 
		magdelaycount--;
		presdelaycount--;
    TimingDelay--;
  }
}

void delay_noInt_ms(uint32_t time)
{
	time *= 1429;//50000 -> 35.006ms
	while(time --);
}
