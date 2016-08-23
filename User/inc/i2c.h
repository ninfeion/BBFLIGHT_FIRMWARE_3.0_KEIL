#ifndef _I2C_H_
#define _I2C_H_

#include "stm32f10x.h"
#include "usb_type.h" // include bool

#define SCL_H         GPIOB->BSRR = GPIO_Pin_6 
#define SCL_L         GPIOB->BRR  = GPIO_Pin_6  

#define SDA_H         GPIOB->BSRR = GPIO_Pin_7 
#define SDA_L         GPIOB->BRR  = GPIO_Pin_7

#define SCL_read      GPIOB->IDR  & GPIO_Pin_6 
#define SDA_read      GPIOB->IDR  & GPIO_Pin_7

void i2cInit(void);
bool I2C_Start(void);
void I2C_Stop(void);
void I2C_Ack(void);
void I2C_NoAck(void);
bool I2C_WaitAck(void);
void I2C_SendByte(uint8_t SendByte);
uint8_t I2C_ReceiveByte(void);
bool Single_Write(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t REG_data);
uint8_t Single_Read(uint8_t SlaveAddress,uint8_t REG_Address);
uint8_t Multiple_Read(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t length, uint8_t *DATA);
uint8_t Multiple_Write(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t length, uint8_t *DATA);

#endif

