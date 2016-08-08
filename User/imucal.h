#ifndef _IMU_CAL_H_
#define _IMU_CAL_H_

#include "stm32f10x.h"
#include "systeminit.h"

void imuUpdate(ImuData *tarData);
float invSqrt(float number);
void pidControl(ImuData *tarData);
void motorUpdate(ImuData *tarData, RespondMess *tarMessData);
void accelAndGyroOffset(ImuData *tarData);
void imuUpdateWithMag(ImuData *tarData);
	
#endif

