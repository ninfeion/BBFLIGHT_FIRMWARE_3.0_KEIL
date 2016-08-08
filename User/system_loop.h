#ifndef _SYSTEM_LOOP_H_
#define _SYSTEM_LOOP_H_
#include"stm32f10x.h"

extern uint8_t radioFlag, attitudeUpdateFlag, rxTimeOutFlag, batteryCheckFlag;
extern volatile uint16_t radioPeriodCount,attitudeUpdatePeriodCount, rxTimeOutCount, batteryCheckCount;

void systemLoopTim1Init(void);
void systemLoop(void);

#endif 
