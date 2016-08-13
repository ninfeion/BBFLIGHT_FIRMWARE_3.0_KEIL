#ifndef _IICDEBUG_H_
#define _IICDEBUG_H_
#include "stm32f10x.h"
#include "usb_type.h"

void I2C_GPIO_Config(void);
void I2C_delay(void);
bool I2C_Start(void);
void I2C_Stop(void);
void I2C_Ack(void);
void I2C_NoAck(void);
bool I2C_WaitAck(void);
void I2C_SendByte(u8 SendByte);
unsigned char I2C_RadeByte(void);
bool Single_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data);
unsigned char Single_Read(unsigned char SlaveAddress,unsigned char REG_Address);
void MPU9250_Init_D(void);
bool MPU9250_Check(void);




#endif

