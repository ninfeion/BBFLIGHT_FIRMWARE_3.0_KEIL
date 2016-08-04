#ifndef _SysInit_H_
#define _SysInit_H_
#include "stm32f10x.h"
#include "stm32f10x_flash.h"
#include "system_config.h"

uint8_t sysInit(void); 
uint8_t sysclockInit(void);
uint8_t systickInit(void);
uint8_t ledInit(void);

typedef struct
{
	uint8_t BATTERY;
	uint8_t NRFONLINE;
	uint8_t MPU9250ONLINE;
	uint32_t nrfExecPrd;
	uint32_t imuExecPrd;
	uint32_t idlePrd;
}SYSTEM_STATE;

typedef struct
{
	float accelRaw[3];
	float gyroRaw[3];
	float magRaw[3];
	int16_t targetThrust;
	float targetPitch;
	float targetRoll;
	float targetYaw;
	int16_t actualThrust;
	float actualPitch;
	float actualRoll;
	float actualYaw;
	
}ImuData;

typedef struct 
{
	uint8_t head[2];
	int16_t thrust;
	float roll;
	float pitch;
	float yaw;
		
	uint8_t motor1;
	uint8_t motor2;
	uint8_t motor3;
	uint8_t motor4;
	uint16_t battery;
	uint8_t linkQuality;
	uint8_t resevered[1];
		
	float broVal;
	uint8_t haveBro;
	uint8_t connect;
	uint8_t tail[2];
}RespondMess;

typedef struct
{
	uint8_t head[2];
	uint8_t devAdd[5];
	uint8_t tryConnect;
	int16_t thrust;
	float roll;
	float pitch;
	float yaw;
	uint8_t estop;
	uint8_t altHold;
	uint8_t pitchNeg;
	uint8_t pitchPos;
	uint8_t rollNeg;
	uint8_t rollPos;
	uint8_t resevered[2];
	uint8_t tail[2];
}AcceptMess;
	
extern AcceptMess BBCom;
extern RespondMess BBMess;

extern ImuData BBImu;
extern SYSTEM_STATE BBSYSTEM;

#endif
