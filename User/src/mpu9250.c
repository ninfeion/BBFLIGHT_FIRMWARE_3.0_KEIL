#include "mpu9250.h"
#include "delay.h"

//AD0=1

void mpu9250Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;//定义GPIO初始化结构体
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);   
	
	//MPU_FSYNC -> PB4 -> set low
	//MPU_INT   -> PB5 
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;       
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOB, GPIO_Pin_4);
	
	Single_Write(MPU9250_ADDRESS,RA_PWR_MGMT_1, 0x00);	//解除休眠状态
	delay_ms(10);//to wait for 9250 reset done
	Single_Write(MPU9250_ADDRESS,RA_SMPLRT_DIV, 0x00);  //0x00	SAMPLE_RATE=Internal_Sample_Rate / (1 + SMPLRT_DIV)
	Single_Write(MPU9250_ADDRESS,RA_CONFIG, 0x02); // 0x02	Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	Single_Write(MPU9250_ADDRESS,RA_GYRO_CONFIG, 0x18); //2000d/s
	Single_Write(MPU9250_ADDRESS,RA_ACCEL_CONFIG_1, 0x18); //16g range
	Single_Write(MPU9250_ADDRESS,RA_ACCEL_CONFIG_2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	
	#ifdef MPU_PASSMODE_ENABLED
		Single_Write(MPU9250_ADDRESS,RA_INT_PIN_CFG,0x02);//turn on Bypass Mode 
		delay_ms(10);
	#endif
}

uint8_t Get_MPU9250_ID(void)
{
	return Single_Read(MPU9250_ADDRESS, RA_WHO_AM_I);
}

uint8_t Get_AK8963_ID_Bypass(void)
{
	Single_Write(MAG_ADDRESS,AK8963_CNTL1,0x11);//single read 
	delay_ms(2);	
	return Single_Read(MAG_ADDRESS, AK8963_WIA);
}

bool MPU9250_Check(void)
{
	if(Get_MPU9250_ID() == VAL_DEVICE_ID)
		return TRUE;
	else
		return FALSE;
}

bool AK8963_Check_Bypass(void)
{
	if(Get_AK8963_ID_Bypass() == VAL_MAG_DEVICE_ID )
		return TRUE;
	else
		return FALSE;
}
	
void READ_MPU9250_ACCEL_RAW(int16_t *ACCELDATA)
{
	uint8_t BUF[6];
	Multiple_Read(MPU9250_ADDRESS ,RA_ACCEL_XOUT_H, 6, BUF);
	ACCELDATA[0] = (int16_t)((BUF[0] << 8) | BUF[1]);
	ACCELDATA[1] = (int16_t)((BUF[2] << 8) | BUF[3]);
	ACCELDATA[2] = (int16_t)((BUF[4] << 8) | BUF[5]);
}

void READ_MPU9250_GYRO_RAW(int16_t *GYRODATA)
{
	uint8_t BUF[6];
	Multiple_Read(MPU9250_ADDRESS ,RA_GYRO_XOUT_H, 6, BUF);
	GYRODATA[0] = (int16_t)((BUF[0] << 8) | BUF[1]);
	GYRODATA[1] = (int16_t)((BUF[2] << 8) | BUF[3]);
	GYRODATA[2] = (int16_t)((BUF[4] << 8) | BUF[5]);
}

int16_t READ_MPU9250_TEMP_RAW(void)
{
	uint8_t BUF[2];
	int16_t TEMPDATA;
	Multiple_Read(MPU9250_ADDRESS ,RA_TEMP_OUT_H, 2, BUF);
	TEMPDATA = (int16_t)((BUF[0] << 8) | BUF[1]);
	return TEMPDATA;
}


void READ_MPU9250_Bypass_MAG_RAW(int16_t *MAGDATA)
{ 
	uint8_t BUF[6];
	// 每读一次，ak8963自动进入powerdown模式,每读一次都要重新设置单测量模式 地磁读的周期不能小于7ms
	// Single_Write(MAG_ADDRESS,AK8963_CNTL1,0x01); // 14位 single read mode
	Single_Write(MAG_ADDRESS,AK8963_CNTL1,0x11);    // 16位 single read mode
	delay_ms(2);

	Multiple_Read(MAG_ADDRESS ,AK8963_HXL , 6, BUF);
	MAGDATA[0] = (int16_t)((BUF[1] << 8) | BUF[0]);
	MAGDATA[1] = (int16_t)((BUF[3] << 8) | BUF[2]);
	MAGDATA[2] = (int16_t)((BUF[5] << 8) | BUF[4]);
}


void readMpu9250BypassMagRawStateMachineReady(uint8_t scale)
{
	if(scale ==0)
	{
		Single_Write(MAG_ADDRESS,AK8963_CNTL1,0x01);   //14位 single read mode
	}
	else if(scale ==1)
	{
		Single_Write(MAG_ADDRESS,AK8963_CNTL1,0x11);   //16位 single read mode
	}
}


void readMpu9250BypassMagRawStateMachineRead(int16_t *MAGDATA)
{ 
	uint8_t BUF[6];

	Multiple_Read(MAG_ADDRESS ,AK8963_HXL , 6, BUF);
	MAGDATA[0] = (int16_t)((BUF[1] << 8) | BUF[0]);
	MAGDATA[1] = (int16_t)((BUF[3] << 8) | BUF[2]);
	MAGDATA[2] = (int16_t)((BUF[5] << 8) | BUF[4]);
}


void accelAndGyroOffset(ImuData *tarData)
{
	uint8_t i,ii;
	int16_t accelRaw[3], gyroRaw[3];
	int32_t accelSum[3] = {0, 0, 0};
	int32_t gyroSum[3]  = {0, 0, 0};
	
	for(i=0; i<30; i++)
	{
		READ_MPU9250_ACCEL_RAW(accelRaw);
		READ_MPU9250_GYRO_RAW(gyroRaw);
		
		for(ii=0; ii<3; ii++)
		{
			accelSum[ii] += accelRaw[ii];
			gyroSum[ii]  += gyroRaw[ii];
		}
	}
	
	for(i=0; i<3; i++)
	{
		tarData->accelOffset[i] = (int16_t)(accelSum[i]/30);
		tarData->gyroOffset[i]  = (int16_t)(gyroSum[i]/30);
	}
	tarData->accelOffset[2] = tarData->accelOffset[2] - (int16_t)ACCELLSB; // Z axis calibration
}
		










