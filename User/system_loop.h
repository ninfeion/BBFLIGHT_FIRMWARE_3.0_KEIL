#ifndef _SYSTEM_LOOP_H_
#define _SYSTEM_LOOP_H_
#include"stm32f10x.h"

extern uint8_t radioFlag, attitudeUpdateFlag;
extern volatile uint16_t radioPeriodCount,attitudeUpdatePeriodCount;
extern uint8_t USB_DEBUG_flag; 
extern volatile uint16_t USB_DEBUG_flag_count;

void systemLoopTim1Init(void);
void systemLoop(void);

#endif 
