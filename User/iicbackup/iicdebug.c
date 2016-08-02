//----------------------------------------------------------------------|
//  Funtion name: IIC_Config.c                      |
//  Funtion:模拟I2C，PB.06(SCL) PB.07(SDA)                 |                                                               
//  Writer: Fish                                |
//  School: Shenzhen University                     |   
//  Date:       2014.11.25                  |                                                                   
//----------------------------------------------------------------------|
//#include "stm32f10x.h"

#include "delay.h"
#include "iicdebug.h"
#include "mpu9250.h"

char  test=0;            
//************************************
/*模拟i2c端口输入输出定义*/
#define SCL_H         GPIOB->BSRR = GPIO_Pin_6
#define SCL_L         GPIOB->BRR  = GPIO_Pin_6 
    
#define SDA_H         GPIOB->BSRR = GPIO_Pin_7
#define SDA_L         GPIOB->BRR  = GPIO_Pin_7
 
#define SCL_read      GPIOB->IDR  & GPIO_Pin_6
#define SDA_read      GPIOB->IDR  & GPIO_Pin_7
 
/*******************************************************************************
* Function Name  : I2C_GPIO_Config
* Description    : Configration Simulation IIC GPIO
* Input          : None 
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_GPIO_Config(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure; 
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  
  GPIO_Init(GPIOB, &GPIO_InitStructure);
 
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}
 
/*******************************************************************************
* Function Name  : I2C_delay
* Description    : Simulation IIC Timing series delay
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_delay(void)
{
         
   u8 i=30; //速度可优化，最小为5
   while(i) 
   { 
     i--; 
   }  
}
/*******************************************************************************
* Function Name  : I2C_Start
* Description    : Master Start Simulation IIC Communication
* Input          : None
* Output         : None
* Return         : Wheather  Start
****************************************************************************** */
bool I2C_Start(void)
{
    SDA_H;
    SCL_H;
    I2C_delay();
    if(!SDA_read)return FALSE;  //SDA为低电平总线忙，退出
    SDA_L;
    I2C_delay();
    if(SDA_read) return FALSE;  //SDA为高电平总线出错，退出
    SDA_L;
    I2C_delay();
    return TRUE;
}
/*******************************************************************************
* Function Name  : I2C_Stop
* Description    : Master Stop Simulation IIC Communication
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_Stop(void)
{
    SCL_L;
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SDA_H;
    I2C_delay();
} 
/*******************************************************************************
* Function Name  : I2C_Ack
* Description    : Master Send Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_Ack(void)
{   
    SCL_L;
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();
}   
/*******************************************************************************
* Function Name  : I2C_NoAck
* Description    : Master Send No Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_NoAck(void)
{   
    SCL_L;
    I2C_delay();
    SDA_H;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();
} 
/*******************************************************************************
* Function Name  : I2C_WaitAck
* Description    : Master Reserive Slave Acknowledge Single
* Input          : None
* Output         : None
* Return         : Wheather  Reserive Slave Acknowledge Single
****************************************************************************** */
bool I2C_WaitAck(void)   //返回：1有ack，0无ack
{
    SCL_L;
    I2C_delay();
    SDA_H;          
    I2C_delay();
    SCL_H;
    I2C_delay();
    if(SDA_read)
    {
      SCL_L;
      I2C_delay();
      return FALSE;
    }
    SCL_L;
    I2C_delay();
    return TRUE;
}
/*******************************************************************************
* Function Name  : I2C_SendByte
* Description    : Master Send a Byte to Slave
* Input          : Will Send Date
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_SendByte(u8 SendByte) //数据从高位到低位
{
    u8 i=8;
    while(i--)
    {
        SCL_L;
        I2C_delay();
      if(SendByte&0x80)
        SDA_H;  
      else
        SDA_L;   
        SendByte<<=1;
        I2C_delay();
        SCL_H;
        I2C_delay();
    }
    SCL_L;
}  
/*******************************************************************************
* Function Name  : I2C_RadeByte
* Description    : Master Reserive a Byte From Slave
* Input          : None
* Output         : None
* Return         : Date From Slave 
****************************************************************************** */
unsigned char I2C_RadeByte(void)  //数据从高位到低位
{ 
    u8 i=8;
    u8 ReceiveByte=0;
 
    SDA_H;              
    while(i--)
    {
      ReceiveByte<<=1;      
      SCL_L;
      I2C_delay();
      SCL_H;
      I2C_delay();  
      if(SDA_read)
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_L;
    return ReceiveByte;
} 
/*********************************************************************************/     
//单字节写入*******************************************
bool Single_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data)           //void
{
    if(!I2C_Start())return FALSE;
    I2C_SendByte(SlaveAddress);   //发送设备地址+写信号
    //I2C_SendByte(((REG_Address & 0x0700) >>7) | SlaveAddress & 0xFFFE);//设置高起始地址+器件地址
    if(!I2C_WaitAck()){I2C_Stop(); return FALSE;}
    I2C_SendByte(REG_Address );   //设置低起始地址      
    I2C_WaitAck();  
    I2C_SendByte(REG_data);
    I2C_WaitAck();   
    I2C_Stop(); 
    delay_ms(5);
    return TRUE;
}
 
//单字节读取*****************************************
unsigned char Single_Read(unsigned char SlaveAddress,unsigned char REG_Address)
{   unsigned char REG_data;         
    if(!I2C_Start())return FALSE;
    I2C_SendByte(SlaveAddress); //I2C_SendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//设置高起始地址+器件地址
    if(!I2C_WaitAck()){I2C_Stop();test=1; return FALSE;}
    I2C_SendByte((u8) REG_Address);   //设置低起始地址
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte(SlaveAddress+1);
    I2C_WaitAck();
    REG_data= I2C_RadeByte();
    I2C_NoAck();
    I2C_Stop();
    //return TRUE;
    return REG_data;
 
}   

void MPU9250_Init_D(void)
{
	Single_Write( MPU9250_ADDRESS ,PWR_MGMT_1, 0x00);
	Single_Write( MPU9250_ADDRESS ,SMPLRT_DIV, 0x07);
	Single_Write( MPU9250_ADDRESS ,LF_CONFIG , 0x06);
	Single_Write( MPU9250_ADDRESS ,GYRO_CONFIG,0x18);
	Single_Write( MPU9250_ADDRESS ,ACCEL_CONFIG_LF,0x01);
}

	
bool MPU9250_Check(void)
{
	if(Single_Read( MPU9250_ADDRESS , WHO_AM_I) == MAG_DEVICE_ID)
		return TRUE;
	else 
		return FALSE;
}


