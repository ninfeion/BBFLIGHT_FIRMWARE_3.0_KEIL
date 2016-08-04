#include "debug.h"
#include "usb2com.h"
#include "adc.h"
#include "i2c.h"
#include "mpu9250.h"
#include "ms5611.h"
#include "system_config.h"
#include "systeminit.h"
#include "delay.h"
#include "spinrf.h"

#include "stdio.h"
#include "stdarg.h"
//#include "string.h"

/*
void TIM2_IRQHandler(void)
{		
	USB_DEBUG_flag_count ++;
	if(USB_DEBUG_flag_count == 1000) //1hz
	{
		USB_DEBUG_flag=1;
		USB_DEBUG_flag_count = 0;
	}
	
	TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);
}

void DEBUG_TIM2_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; 
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  //打开时钟
    
  TIM_DeInit(TIM2);

  TIM_TimeBaseStructure.TIM_Period = 7200;//定时1ms
  TIM_TimeBaseStructure.TIM_Prescaler = 10-1;//预分频 72/(10-1 +1)
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    
  TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
	
  // NVIC_PriorityGroup 2
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//串口打印定时器，优先级低于姿态解算
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	TIM_ClearFlag(TIM2,TIM_FLAG_Update);
  TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
  TIM_Cmd(TIM2,ENABLE);
}
*/

void USART_printf(uint8_t *TarStr, ...)
{

}

void USB_printf(uint8_t *TarStr, ...)
{
	va_list args;
		
	uint8_t BUF[USB_PRINTF_BUFFER_SIZE];
	uint16_t length;
		
	va_start(args, TarStr);
	length = vsprintf(BUF, TarStr, args);
	USB_TxWrite(BUF,length);
	va_end(args);
}

void usbDebugLoop(void)
{
	if(USB_DEBUG_flag==1)	
	{				
		USB_DEBUG_flag=0;
		USB_DEBUG_RADIO();
	}
}
	
void USB_DEBUG_RADIO(void)
{
	int16_t ACCELDATA[3];
	int16_t GYRODATA[3];
//	int16_t TEMPDATA;
	int16_t MAGDATA[3];
	
	float ACCELDATA_F[3];
	float GYRODATA_F[3];
	float MAGDATA_F[3];
//	float TEMP_F;
	
	uint16_t ADCTEMP;

	static uint16_t MS5611_PROM[7];
	static float MS5611_TEMP_NoOffset[2],MS5611_TaPAfterOffset[2],PRESSUREDATA;
	
	MS5611_PROM_READ(MS5611_PROM);
	MS5611_GetTempture(CMD_MS5611_D2_OSR_4096, MS5611_PROM, MS5611_TEMP_NoOffset);
	MS5611_GetPressure(CMD_MS5611_D1_OSR_4096, MS5611_PROM, MS5611_TEMP_NoOffset, MS5611_TaPAfterOffset);
	PRESSUREDATA = MS5611_TaPAfterOffset[1] * 0.01f;
	
	if(BBSYSTEM.NRFONLINE==1)
		USB_printf("NRF is online!\n");
	
	if(BBSYSTEM.MPU9250ONLINE==1)	
		USB_printf("MPU checking pass!\n");
	
	READ_MPU9250_ACCEL_RAW(ACCELDATA);
	READ_MPU9250_GYRO_RAW(GYRODATA);
	READ_MPU9250_Bypass_MAG_RAW(MAGDATA);
//	TEMPDATA=READ_MPU9250_TEMP_RAW();
	
	ACCELDATA_F[0]=ACCELDATA[0]/ACCELLSB;
	ACCELDATA_F[1]=ACCELDATA[1]/ACCELLSB;
	ACCELDATA_F[2]=ACCELDATA[2]/ACCELLSB;	
	GYRODATA_F[0]=GYRODATA[0]/GYROLSB;
	GYRODATA_F[1]=GYRODATA[1]/GYROLSB;
	GYRODATA_F[2]=GYRODATA[2]/GYROLSB;
	MAGDATA_F[0]=MAGDATA[0]*MAGLSB;
	MAGDATA_F[1]=MAGDATA[1]*MAGLSB;
	MAGDATA_F[2]=MAGDATA[2]*MAGLSB;
//TEMP_F=TEMPDATA/340.0 + 36.53;
	
	ADCTEMP = adcBatteryConversion();
  USB_printf("The BatVolt is: %d mv\n",ADCTEMP);
	USB_printf("The Pressure is: %0.4f mbar\n", PRESSUREDATA);
	USB_printf("X axis Acceleration is: %0.4f g\n",ACCELDATA_F[0]);
	USB_printf("Y axis Acceleration is: %0.4f g\n",ACCELDATA_F[1]);
	USB_printf("Z axis Acceleration is: %0.4f g\n",ACCELDATA_F[2]);
	USB_printf("X axis Gyro is: %0.3f deg/s\n",GYRODATA_F[0]);
	USB_printf("Y axis Gyro is: %0.3f deg/s\n",GYRODATA_F[1]);
	USB_printf("Z axis Gyro is: %0.3f deg/s\n",GYRODATA_F[2]);
	USB_printf("X axis Mag is: %0.3f uT\n",MAGDATA_F[0]);
	USB_printf("Y axis Mag is: %0.3f uT\n",MAGDATA_F[1]);
	USB_printf("Z axis Mag is: %0.3f uT\n",MAGDATA_F[2]);
	USB_printf("==================================\n");
	
	//USB_TxWrite(NRF24L01_RXDATA,strlen(NRF24L01_RXDATA));
	//USB_printf("\n");
}
	

/*
void USB_printf(*TarStr,...)
{	
	uint16_t length=0;
	//while (*TarStr++ != '\0') 
	while (TarStr[length] != '\0')
		{
        ++length;
    }
	USB_TxWrite(TarStr,length);
}

{
	va_list argumenttemp;//说明变量argumenttemp
	
	uint8_t chartemp;
	uint8_t DISTEMP[10];
	uint16_t length;
	
	uint8_t FloatSwitchTemp[10];
	uint8_t FloatSwitchCount;
	uint8_t NewLine[]="\n";
	float Ftypedatatemp;

	uint16_t Dtypedatatemp;
	
	va_start(argumenttemp, TarStr);//argumenttemp被初始化为指向TarStr后的第一个参数
	while(*TarStr)
	{
		chartemp = *TarStr;
		if (chartemp != '%')
			{
				USB_TxWrite(&chartemp,1);
			}
			else
			{
				if (chartemp == '\n')
				{
					++TarStr;
					USB_TxWrite(NewLine,1);
				}
					else
					{
						switch(* ++TarStr)
						{
							case 'd': 
								Dtypedatatemp =(uint16_t) va_arg(argumenttemp, int);//将变量argumenttemp所指向的uint16_t类型的值赋给Dtypedatatemp,同时使argumenttemp指向下一个参数
								sprintf(DISTEMP,"%d",Dtypedatatemp);
								length=0;
								while (DISTEMP[length] != '\0')	++length;									
								USB_TxWrite(DISTEMP,length);								
								break;
							case 'f':
								Ftypedatatemp = (float) va_arg(argumenttemp,double);
								sprintf(DISTEMP,"%f",Ftypedatatemp);
								length=0;
								while (DISTEMP[length] != '\0')	++length;									
								USB_TxWrite(DISTEMP,length);	
								break;
							default : 
								Ftypedatatemp =(float) va_arg(argumenttemp, double);
								FloatSwitchCount = 10;
								while(*TarStr != 'f')
								{
									FloatSwitchTemp[FloatSwitchCount] = *TarStr;
									++TarStr;
									--FloatSwitchCount;
								}
								FloatSwitchTemp[FloatSwitchCount++ ] = *TarStr; //='f'
								FloatSwitchTemp[FloatSwitchCount] = '\0';
								sprintf(DISTEMP,FloatSwitchTemp,Ftypedatatemp);
								length=0;
								while (DISTEMP[length] != '\0')	++length;									
								USB_TxWrite(DISTEMP,length);	
				   	}			
					}
				}
			++TarStr;
		}
	va_end(argumenttemp);
}
*/



