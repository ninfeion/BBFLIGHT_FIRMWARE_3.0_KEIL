#ifndef _SysInit_H_
#define _SysInit_H_
#include "stm32f10x.h"
#include "stm32f10x_flash.h"
#include "system_config.h"

uint8_t sysInit(void); 
uint8_t sysclockInit(void);
uint8_t systickInit(void);
uint8_t ledInit(void);

struct SYSTEM_STATE
{
	uint8_t BATTERY;
	uint8_t NRFONLINE;
	uint8_t MPU9250ONLINE;
};

extern struct SYSTEM_STATE BBSYSTEM;

#endif
