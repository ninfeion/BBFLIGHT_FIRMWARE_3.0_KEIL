#ifndef _IMU_CAL_H_
#define _IMU_CAL_H_

#include "stm32f10x.h"
#include "systeminit.h"

float invSqrt(float number);
void imuUpdate(ImuData *tarData);
void imuUpdateWithMag(ImuData *tarData);

void IMUSO3Thread(ImuData *tarData);
	
#endif

