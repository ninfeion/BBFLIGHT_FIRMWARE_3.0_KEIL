#include "control.h"
#include "pwm.h"
#include "delay.h"

static float derivativedt = 0.0;
static uint32_t timeForPidCalculate = 0;

void pidControl(ImuData *tarData)
{
	derivativedt = (float)(currentTime() - timeForPidCalculate) /1000000.0;
	timeForPidCalculate = currentTime();	
	
	tarData->pidRoll.lastError = tarData->pidRoll.preError;
	tarData->pidRoll.preError  = tarData->actualRoll.newData - tarData->targetRoll;
	// tarData->pidRoll.errorSum += tarData->pidRoll.preError;
	tarData->pidRoll.pidPout = tarData->pidRoll.pidP * tarData->pidRoll.preError;
	/** derivative term method 1 **/
	// tarData->pidRoll.pidDout = tarData->pidRoll.pidD * (tarData->actualRoll.newData - tarData->actualRoll.lastData);
	/** derivative term method 2 **/
	// tarData->pidRoll.pidDout = tarData->pidRoll.pidD * tarData->gyroRaw.newData[0];
	/** derivative term method 3 **/
	tarData->pidRoll.pidDout = tarData->pidRoll.pidD * ((tarData->pidRoll.preError - tarData->pidRoll.lastError) / derivativedt);
	
	tarData->pidPitch.lastError = tarData->pidPitch.preError;
	tarData->pidPitch.preError  = tarData->actualPitch.newData - tarData->targetPitch;
	// tarData->pidPitch.errorSum += tarData->pidPitch.preError;
	tarData->pidPitch.pidPout = tarData->pidPitch.pidP * tarData->pidPitch.preError;
	/** derivative term method 1 **/
	// tarData->pidPitch.pidDout = tarData->pidPitch.pidD * (tarData->actualPitch.newData - tarData->actualPitch.lastData);
	/** derivative term method 2 **/
	// tarData->pidPitch.pidDout = tarData->pidPitch.pidD * tarData->gyroRaw.newData[1];
	/** derivative term method 3 **/
	tarData->pidPitch.pidDout = tarData->pidPitch.pidD * ((tarData->pidPitch.preError - tarData->pidPitch.lastError) / derivativedt);
	
	tarData->pidYaw.lastError = tarData->pidYaw.preError;
	tarData->pidYaw.preError  = tarData->actualYaw.newData - tarData->targetYaw;
	/** derivative term method 1 **/
	// tarData->pidYaw.pidDout = tarData->pidYaw.pidD * (tarData->actualYaw.newData - tarData->actualYaw.lastData);
	/** derivative term method 2 **/
	// tarData->pidYaw.pidDout = tarData->pidYaw.pidD * tarData->gyroRaw.newData[2];
	/** derivative term method 3 **/
	tarData->pidYaw.pidDout = tarData->pidYaw.pidD * ((tarData->pidYaw.preError - tarData->pidYaw.lastError) / derivativedt);
	
	tarData->pidRoll.pidFinalOut  = tarData->pidRoll.pidPout  + tarData->pidRoll.pidIout  + tarData->pidRoll.pidDout;
	tarData->pidPitch.pidFinalOut = tarData->pidPitch.pidPout + tarData->pidPitch.pidIout + tarData->pidPitch.pidDout;
	tarData->pidYaw.pidFinalOut   = tarData->pidYaw.pidDout;
}


void motorUpdate(ImuData *tarData, RespondMess *tarMessData)
{
	float motor[4];
	uint8_t i;
	
	tarMessData->thrust = tarData->targetThrust;
	
	motor[0] = tarData->targetThrust + tarData->pidPitch.pidFinalOut - tarData->pidRoll.pidFinalOut + tarData->pidYaw.pidFinalOut;
	motor[1] = tarData->targetThrust - tarData->pidPitch.pidFinalOut - tarData->pidRoll.pidFinalOut - tarData->pidYaw.pidFinalOut;
	motor[2] = tarData->targetThrust - tarData->pidPitch.pidFinalOut + tarData->pidRoll.pidFinalOut + tarData->pidYaw.pidFinalOut;
	motor[3] = tarData->targetThrust + tarData->pidPitch.pidFinalOut + tarData->pidRoll.pidFinalOut - tarData->pidYaw.pidFinalOut;
	
	motor[0] = motor[0] /(2000);
	motor[1] = motor[1] /(2000);
	motor[2] = motor[2] /(2000);
	motor[3] = motor[3] /(2000);
	
	for(i=0; i<4; i++)
	{
		if(motor[i] <0.0)
		{
			motor[i] = 0;
		}
		if(motor[i] >1.0)
		{
			motor[i] = 1;
		}
	}
	
	if(tarData->targetThrust > 500)
	{
		motorPwmFlash((uint16_t)(motor[0]*Moto_PwmMax),    // M2
					  (uint16_t)(motor[1]*Moto_PwmMax),    // M1
					  (uint16_t)(motor[2]*Moto_PwmMax),    // M3
					  (uint16_t)(motor[3]*Moto_PwmMax));   // M4  ????
	}
	else
	{
		motorPwmFlash(0, 0, 0, 0);
	}

	tarMessData->motor2 = (uint8_t)(motor[0]*100); 
	tarMessData->motor3 = (uint8_t)(motor[2]*100); 
	tarMessData->motor4 = (uint8_t)(motor[3]*100); 
	tarMessData->motor1 = (uint8_t)(motor[1]*100);
}

