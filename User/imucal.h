#ifndef _IMU_CAL_H_
#define _IMU_CAL_H_

#include "stm32f10x.h"
#include "systeminit.h"

#define RADTODEG      57.295779f
#define EPSINON       0.000001f
#define SAMPLINGFREQ  200.0f
#define LPFCUTOFFFREQ 30.0f

#define Kp           1.6f                  // 比例增益支配率收敛到加速度计/磁强计 2.0
#define Ki           0.001f                // 积分增益支配率的陀螺仪偏见的衔接 0.005f

void imuUpdate(ImuData *tarData);
float invSqrt(float number);
void pidControl(ImuData *tarData);
void motorUpdate(ImuData *tarData, RespondMess *tarMessData);
void accelAndGyroOffset(ImuData *tarData);
void imuUpdateWithMag(ImuData *tarData);
	
#endif

