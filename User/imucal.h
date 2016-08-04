#ifndef _IMU_CAL_H_
#define _IMU_CAL_H_

#include "stm32f10x.h"
#include "systeminit.h"

void IMUupdate(ImuData *tarData);
float invSqrt(float number);
	
#endif

