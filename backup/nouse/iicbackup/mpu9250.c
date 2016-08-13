#include "mpu9250.h"

void MPU9250_Init(void)
{
	IIC_SendByte_Add_Reg(MPU9250_ADDRESS , PWR_MGMT_1 , 0x00);
	IIC_SendByte_Add_Reg(MPU9250_ADDRESS , LF_CONFIG  , 0x1A);
	IIC_SendByte_Add_Reg(MPU9250_ADDRESS , GYRO_CONFIG, 0x18);
	IIC_SendByte_Add_Reg(MPU9250_ADDRESS , ACCEL_CONFIG_RANGE,0x18);
	IIC_SendByte_Add_Reg(MPU9250_ADDRESS , ACCEL_CONFIG_LF   ,0x06);
	IIC_SendByte_Add_Reg(MPU9250_ADDRESS , USER_CTRL  , 0x00);//¿ªiic 
	
}

uint8_t buffer[14];

uint8_t MPU9250_getDeviceID(void) {

    IICreadBytes(MPU9250_ADDRESS, WHO_AM_I , 1, buffer);
    return buffer[0];
}

bool Check_MPU9250(void)
{
	if(MPU9250_getDeviceID()  == 0x71)
		return TRUE;
	else
		return FALSE;
}











