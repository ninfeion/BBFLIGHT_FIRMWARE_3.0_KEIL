#include "ms5611.h"
#include "delay.h"
#include "i2c.h"
#include "math.h"

void ms5611Init(void)
{
	MS5611_Reset();
}

void MS5611_Reset(void)
{
	I2C_Start();
	I2C_SendByte(MS5611_ADDRESS); //写地址
	I2C_WaitAck();
	I2C_SendByte(CMD_MS5611_RESET);//发送复位命令
	I2C_WaitAck();	
	I2C_Stop();
}

void MS5611_PROM_READ(uint16_t *PROM_C)
{
	uint8_t DATATEMP[2],count;
	
	for(count=0;count<=6;count++)
	{
		I2C_Start();
		I2C_SendByte(MS5611_ADDRESS);
		I2C_WaitAck();
		I2C_SendByte(CMD_MS5611_PROM_RD+count*2);
		I2C_WaitAck();
		I2C_Stop();
		delay_us(5);
		I2C_Start();
		I2C_SendByte(MS5611_ADDRESS +1);
		delay_us(1);
		I2C_WaitAck();
		DATATEMP[0]=I2C_ReceiveByte();
		I2C_Ack();
		delay_us(1);
		DATATEMP[1]=I2C_ReceiveByte();
		I2C_NoAck();
		I2C_Stop();
		//delay_ms(5);
		PROM_C[count]=(uint16_t)((DATATEMP[0]<<8) | DATATEMP[1]);
	}
}

uint32_t MS5611_SendCmd_StartConversionAndGetIt(uint8_t OSR_Cmd_SelectTempOrPres)
{
	uint8_t Datatemp[3];
	uint32_t ConversionSequence;
	
	I2C_Start();
	I2C_SendByte(MS5611_ADDRESS); //写地址
	I2C_WaitAck();
	I2C_SendByte(OSR_Cmd_SelectTempOrPres);//发送复位命令
	I2C_WaitAck();	
	I2C_Stop();
	
	delay_us(ConversionTime);
	
	I2C_Start();
	I2C_SendByte(MS5611_ADDRESS); //写地址
	I2C_WaitAck();
	I2C_SendByte(0);// start read sequence
	I2C_WaitAck();	
	I2C_Stop();
	
	I2C_Start();
	I2C_SendByte(MS5611_ADDRESS +1);  //进入接收模式	
	I2C_WaitAck();
	Datatemp[0] = I2C_ReceiveByte(); //带ACK的读数据  bit 23-16
	I2C_Ack();
	Datatemp[1] = I2C_ReceiveByte(); //带ACK的读数据  bit 8-15
	I2C_Ack();
	Datatemp[2] = I2C_ReceiveByte(); //带NACK的读数据 bit 0-7
	I2C_NoAck();
	I2C_Stop();
	ConversionSequence = (uint32_t)Datatemp[0]<<16 | (uint32_t)Datatemp[1]<<8 | (uint32_t)Datatemp[2];
	return ConversionSequence;
}

void MS5611_GetTempture(uint8_t SelectTempOsr, uint16_t *PROM_C, float *TEMPERATURE_DATA)
{
	//TEMPERATURE_DATA[0] dT
	//TEMPERATURE_DATA[1] 初始温度
	
	uint32_t D2_DATA;
	//float D2_dT,TEMPERATURE;
	
	D2_DATA = MS5611_SendCmd_StartConversionAndGetIt(SelectTempOsr);
	TEMPERATURE_DATA[0] = D2_DATA - (((int32_t)PROM_C[5] ) << 8 );//= D2_dT 
	TEMPERATURE_DATA[1] = 2000 + TEMPERATURE_DATA[0] * ((int32_t)PROM_C[6])/8388608;//= TEMPERATURE
}

void MS5611_GetPressure(uint8_t SelectPresOsr, uint16_t *PROM_C, float *TEMPERATURE_DATA, float *AfterOffset_TempPresDATA)
{
	//AfterOffset_TempPresDATA[0] 修正后的温度
	//AfterOffset_TempPresDATA[1] 修正后的气压
	
	uint32_t D1_DATA;
	double SENS,OFF;
	float T2,OFF2,SENS2,Aux;
	//float PRESSURE,TEMPERATURE;
	
	D1_DATA = MS5611_SendCmd_StartConversionAndGetIt(SelectPresOsr);
	OFF = ((int64_t)PROM_C[2]*65536) + (( (int64_t)PROM_C[4] * TEMPERATURE_DATA[0] )/128);
	SENS = ((int64_t)PROM_C[1]*32768) + (( (int64_t)PROM_C[2] * TEMPERATURE_DATA[0] )/256);
	
	if(TEMPERATURE_DATA[1] < 2000)
	{
		T2 = TEMPERATURE_DATA[0] * TEMPERATURE_DATA[0] / 0x80000000 ;
		Aux = (TEMPERATURE_DATA[1] - 2000) * (TEMPERATURE_DATA[1] - 2000);
		OFF2 = 2.5 * Aux;
		SENS2 = 1.25 * Aux; 
		
		if(TEMPERATURE_DATA[1] < -1500)
		{
			//OFF2 = OFF2 + 7* ((int64_t)TEMPERATURE_DATA[1] + 1500)*(TEMPERATURE_DATA[1] + 1500);
			//SENS2 =SENS2 + (11* ((int64_t)TEMPERATURE_DATA[1] + 1500)*(TEMPERATURE_DATA[1] + 1500)>>1);  
		} 
	}
	else//TEMP>20C
	{
		T2=0;
		OFF2=0;
		SENS2=0;
	}

	AfterOffset_TempPresDATA[0]  = TEMPERATURE_DATA[1] - T2;//= TEMPERATURE
	OFF = OFF - OFF2;
	SENS = SENS - SENS2;
	
	AfterOffset_TempPresDATA[1] = (D1_DATA*SENS/2097152 - OFF) / 32768;//= PRESSURE
}

uint8_t MS5611_GetAltitude(float *AfterOffset_TempPresDATA, float *Altitude_DATA)
{
	//Altitude_DATA[0] 上电的气压 0参考高度
	//Altitude_DATA[1] 目前的高度算上上电的高度的修正
	static uint8_t Filtercount;
	double FilterTempData;
	if(Altitude_DATA[0] == 0)	// 是否初始化过0米气压值？
	{ 
		if(Filtercount < DefaultPresInitFilterTime )
		{
			FilterTempData += AfterOffset_TempPresDATA[1];
			Filtercount ++;
			return 0;
		}
		else
		{
		Altitude_DATA[0] = FilterTempData / DefaultPresInitFilterTime ;	            
		Altitude_DATA[1] = 0; 
		return 1;
		}
	}
	else
	//计算相对于上电时的位置的高度值 。单位为m
	{
		Altitude_DATA[1] = Altitude_DATA[1] + 4433000.0 * (1 - pow((AfterOffset_TempPresDATA[1] / Altitude_DATA[0]), 0.1903))*0.01f;
		return 1;
	}
}



