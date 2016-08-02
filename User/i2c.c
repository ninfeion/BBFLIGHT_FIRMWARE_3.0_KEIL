#include "i2c.h"
#include "delay.h"

void i2cInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;//定义GPIO初始化结构体
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);   

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

bool I2C_Start(void) 
{ 
	SDA_H; 
	SCL_H; 
	delay_us(4); 
	if(!SDA_read)
		return FALSE; //SDA线为低电平则总线忙,退出 
	SDA_L; 
	delay_us(4); 
	if(SDA_read) 
		return FALSE; //SDA线为高电平则总线出错,退出 
	SDA_L; 
	delay_us(4); 
	return TRUE; 
}

void I2C_Stop(void) 
{ 
	SCL_L; 
	delay_us(4); 
	SDA_L; 
	delay_us(4);
	SCL_H; 
	delay_us(4);
	SDA_H; 
	delay_us(4);
}

void I2C_Ack(void) 
{ 
	SCL_L; 
	delay_us(4);
	SDA_L; 
	delay_us(4);
	SCL_H; 
	delay_us(4);
	SCL_L; 
	delay_us(4);
}

void I2C_NoAck(void) 
{ 
	SCL_L; 
	delay_us(4);
	SDA_H; 
	delay_us(4);
	SCL_H; 
	delay_us(4);
	SCL_L; 
	delay_us(4);
}

bool I2C_WaitAck(void)   //返回为:=1有ACK,=0无ACK 
{ 
	SCL_L; 
	delay_us(4);
	SDA_H; 
	delay_us(4);
	SCL_H; 
	delay_us(4);
	if(SDA_read) 
	{ 
		SCL_L; 
		return FALSE; 
	} 
	SCL_L; 
	return TRUE; 
	}

void I2C_SendByte(uint8_t SendByte) //数据从高位到低位// 
{ 
	uint8_t i=8; 
	while(i--) 
	{ 
		SCL_L; 
		delay_us(4);
		if(SendByte&0x80) 
			SDA_H;   
			else  
				SDA_L;    
		SendByte<<=1; 
		delay_us(4);
		SCL_H; 
		delay_us(4);
	} 
	SCL_L; 
}

uint8_t I2C_ReceiveByte(void)  //数据从高位到低位// 
{  
    uint8_t i=8; 
    uint8_t ReceiveByte=0;
    SDA_H; 
    while(i--) 
    { 
      ReceiveByte<<=1;       
      SCL_L; 
			delay_us(4);
			SCL_H; 
      delay_us(4);
      if(SDA_read) 
      { 
        ReceiveByte|=0x01; 
      } 
    } 
    SCL_L; 
    return ReceiveByte; 
}

bool Single_Write(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t REG_data)		  
{
  	if(!I2C_Start())return FALSE;
    I2C_SendByte(SlaveAddress);   //发送设备地址+写信号//I2C_SendByte(((REG_Address & 0x0700) >>7) | SlaveAddress & 0xFFFE);//设置高起始地址+器件地址 
    if(!I2C_WaitAck())
		{
			I2C_Stop(); 
			return FALSE;
		}
    I2C_SendByte(REG_Address );   //设置低起始地址      
    I2C_WaitAck();	
    I2C_SendByte(REG_data);
    I2C_WaitAck();   
    I2C_Stop(); 
    delay_ms(5);
    return TRUE;
}

uint8_t Single_Read(uint8_t SlaveAddress,uint8_t REG_Address)
{   
	uint8_t REG_data;     	
	if(!I2C_Start())
		return FALSE;
  I2C_SendByte(SlaveAddress); //I2C_SendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//设置高起始地址+器件地址 
  if(!I2C_WaitAck())
	{
		I2C_Stop();
		return FALSE;
	}
  I2C_SendByte((uint8_t) REG_Address);   //设置低起始地址      
  I2C_WaitAck();
  I2C_Start();
  I2C_SendByte(SlaveAddress+1);
  I2C_WaitAck();
	REG_data= I2C_ReceiveByte();
  I2C_NoAck();
  I2C_Stop();
	return REG_data;
}						   

uint8_t Multiple_Read(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t length, uint8_t *DATA)
{
	uint8_t count,datatemp;
	I2C_Start();
	I2C_SendByte(SlaveAddress);
  I2C_WaitAck();
	I2C_SendByte(REG_Address); 
	I2C_WaitAck();
	I2C_Start();
	I2C_SendByte(SlaveAddress+1);
	I2C_WaitAck();
	for(count=0;count<length;count++)
	{
		if(count!= length-1 )
		{
			datatemp = I2C_ReceiveByte();
			I2C_Ack();
		}
		else
		{
			datatemp = I2C_ReceiveByte();
			I2C_NoAck();
		}
		DATA[count] = datatemp;
	}
	I2C_Stop();
	return count;
}




