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
	
	#ifdef USE_MAG_PASSMODE
	Single_Write(MPU9250_ADDRESS,RA_INT_PIN_CFG,0x02);//turn on Bypass Mode 
	delay_ms(10);
	Single_Write(MAG_ADDRESS,AK8963_CNTL1,0x11);
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
	uint8_t BUF[6];//缓存
	/*
	BUF[0]=Single_Read(MPU9250_ADDRESS,RA_ACCEL_XOUT_H); 
	BUF[1]=Single_Read(MPU9250_ADDRESS,RA_ACCEL_XOUT_L);
	BUF[2]=Single_Read(MPU9250_ADDRESS,RA_ACCEL_YOUT_H);
	BUF[3]=Single_Read(MPU9250_ADDRESS,RA_ACCEL_YOUT_L);
	BUF[4]=Single_Read(MPU9250_ADDRESS,RA_ACCEL_ZOUT_H);
	BUF[5]=Single_Read(MPU9250_ADDRESS,RA_ACCEL_ZOUT_L);
	*/
	Multiple_Read(MPU9250_ADDRESS ,RA_ACCEL_XOUT_H, 6, BUF);
	ACCELDATA[0] = (int16_t)((BUF[0] << 8) | BUF[1]);
	ACCELDATA[1] = (int16_t)((BUF[2] << 8) | BUF[3]);
	ACCELDATA[2] = (int16_t)((BUF[4] << 8) | BUF[5]);
}

void READ_MPU9250_GYRO_RAW(int16_t *GYRODATA)
{
	uint8_t BUF[6];//缓存
	/*
	BUF[0]=Single_Read(MPU9250_ADDRESS,RA_GYRO_XOUT_H); 
	BUF[1]=Single_Read(MPU9250_ADDRESS,RA_GYRO_XOUT_L);
	BUF[2]=Single_Read(MPU9250_ADDRESS,RA_GYRO_YOUT_H);
	BUF[3]=Single_Read(MPU9250_ADDRESS,RA_GYRO_YOUT_L);
	BUF[4]=Single_Read(MPU9250_ADDRESS,RA_GYRO_ZOUT_H);
	BUF[5]=Single_Read(MPU9250_ADDRESS,RA_GYRO_ZOUT_L);
	*/
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
	//BUF[0]=Single_Read(MPU9250_ADDRESS,RA_TEMP_OUT_H);
	//BUF[1]=Single_Read(MPU9250_ADDRESS,RA_TEMP_OUT_L);
	TEMPDATA = (int16_t)((BUF[0] << 8) | BUF[1]);
	return TEMPDATA;
}

void READ_MPU9250_Bypass_MAG_RAW(int16_t *MAGDATA)
{ 
	uint8_t BUF[6];
	//Single_Write(MAG_ADDRESS,AK8963_CNTL1,0x01);//14位 single read 每读一次，ak8963自动进入powerdown模式,每读一次都要重新设置单测量模式 地磁读的周期不能小于7ms
	Single_Write(MAG_ADDRESS,AK8963_CNTL1,0x11);//16位 single read mode
	delay_ms(2);
	/*
	BUF[0]=Single_Read(MAG_ADDRESS,AK8963_HXL);
	Single_Write(MAG_ADDRESS,AK8963_CNTL1,0x11);
	BUF[1]=Single_Read(MAG_ADDRESS,AK8963_HXH);
	Single_Write(MAG_ADDRESS,AK8963_CNTL1,0x11);
	BUF[2]=Single_Read(MAG_ADDRESS,AK8963_HYL);
	Single_Write(MAG_ADDRESS,AK8963_CNTL1,0x11);
	BUF[3]=Single_Read(MAG_ADDRESS,AK8963_HYH);
	Single_Write(MAG_ADDRESS,AK8963_CNTL1,0x11);
	BUF[4]=Single_Read(MAG_ADDRESS,AK8963_HZL);
	Single_Write(MAG_ADDRESS,AK8963_CNTL1,0x11);
	BUF[5]=Single_Read(MAG_ADDRESS,AK8963_HZH);
	*/
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
		

/*
mag修正yaw
boolean MPU9150Lib::read()
{
    short intStatus;
    int result;
    short sensors;
    unsigned char more;
    unsigned long timestamp;
   
    mpu_get_int_status(&intStatus);                       // get the current MPU state
    if ((intStatus & MPU_INT_STATUS_DMP_0) == 0)          // return false if definitely not ready yet
        return false;
        
    //  get the data from the fifo
        
    if ((result = dmp_read_fifo(m_rawGyro, m_rawAccel, m_rawQuaternion, &timestamp, &sensors, &more)) != 0) {
      return false;
    }      
    
    //  got the fifo data so now get the mag data
    
    if ((result = mpu_get_compass_reg(m_rawMag, &timestamp)) != 0) {
#ifdef MPULIB_DEBUG
         Serial.print("Failed to read compass: "); 
         Serial.println(result); 
         return false;
#endif
    }
    
    // got the raw data - now process
    
    m_dmpQuaternion[QUAT_W] = (float)m_rawQuaternion[QUAT_W];  // get float version of quaternion
    m_dmpQuaternion[QUAT_X] = (float)m_rawQuaternion[QUAT_X];
    m_dmpQuaternion[QUAT_Y] = (float)m_rawQuaternion[QUAT_Y];
    m_dmpQuaternion[QUAT_Z] = (float)m_rawQuaternion[QUAT_Z];
    MPUQuaternionNormalize(m_dmpQuaternion);                 // and normalize
    
    MPUQuaternionQuaternionToEuler(m_dmpQuaternion, m_dmpEulerPose);

        //        *** Note mag axes are changed here to align with gyros: Y = -X, X = Y

    if (m_useMagCalibration) {
      m_calMag[VEC3_Y] = -(short)(((long)(m_rawMag[VEC3_X] - m_magXOffset) * (long)SENSOR_RANGE) / (long)m_magXRange);
      m_calMag[VEC3_X] = (short)(((long)(m_rawMag[VEC3_Y] - m_magYOffset) * (long)SENSOR_RANGE) / (long)m_magYRange);
      m_calMag[VEC3_Z] = (short)(((long)(m_rawMag[VEC3_Z] - m_magZOffset) * (long)SENSOR_RANGE) / (long)m_magZRange);
    } else {
      m_calMag[VEC3_Y] = -m_rawMag[VEC3_X];
      m_calMag[VEC3_X] = m_rawMag[VEC3_Y];
      m_calMag[VEC3_Z] = m_rawMag[VEC3_Z];
    }

    // Scale accel data 

    if (m_useAccelCalibration) {
      m_calAccel[VEC3_X] = -(short)(((long)m_rawAccel[VEC3_X] * (long)SENSOR_RANGE) / (long)m_accelXRange);
      m_calAccel[VEC3_Y] = (short)(((long)m_rawAccel[VEC3_Y] * (long)SENSOR_RANGE) / (long)m_accelYRange);
      m_calAccel[VEC3_Z] = (short)(((long)m_rawAccel[VEC3_Z] * (long)SENSOR_RANGE) / (long)m_accelZRange);
    } else {
      m_calAccel[VEC3_X] = -m_rawAccel[VEC3_X];
      m_calAccel[VEC3_Y] = m_rawAccel[VEC3_Y];
      m_calAccel[VEC3_Z] = m_rawAccel[VEC3_Z];
    }
    dataFusion();
    return true;
}


void dataFusion();                                        // fuse mag data with the dmp quaternion
void MPU9150Lib::dataFusion()
{
  float qMag[4];
  float deltaDMPYaw, deltaMagYaw;
  float newMagYaw, newYaw;
  float temp1[4], unFused[4];
  float unFusedConjugate[4];

  // *** NOTE *** pitch direction swapped here

  m_fusedEulerPose[VEC3_X] = m_dmpEulerPose[VEC3_X];
  m_fusedEulerPose[VEC3_Y] = -m_dmpEulerPose[VEC3_Y];
  m_fusedEulerPose[VEC3_Z] = 0;        
  MPUQuaternionEulerToQuaternion(m_fusedEulerPose, unFused);    // create a new quaternion

  deltaDMPYaw = -m_dmpEulerPose[VEC3_Z] + m_lastDMPYaw;         // calculate change in yaw from dmp
  m_lastDMPYaw = m_dmpEulerPose[VEC3_Z];                        // update that

  qMag[QUAT_W] = 0;
  qMag[QUAT_X] = m_calMag[VEC3_X];
  qMag[QUAT_Y] = m_calMag[VEC3_Y];
  qMag[QUAT_Z] = m_calMag[VEC3_Z];
        
  // Tilt compensate mag with the unfused data (i.e. just roll and pitch with yaw 0)
        
  MPUQuaternionConjugate(unFused, unFusedConjugate);
  MPUQuaternionMultiply(qMag, unFusedConjugate, temp1);
  MPUQuaternionMultiply(unFused, temp1, qMag);

  // Now fuse this with the dmp yaw gyro information

  newMagYaw = -atan2(qMag[QUAT_Y], qMag[QUAT_X]);

  if (newMagYaw != newMagYaw) {                                 // check for nAn
#ifdef MPULIB_DEBUG
    Serial.println("***nAn\n");
#endif
    return;                                                     // just ignore in this case
  }
  if (newMagYaw < 0)
    newMagYaw = 2.0f * (float)M_PI + newMagYaw;                 // need 0 <= newMagYaw <= 2*PI

  newYaw = m_lastYaw + deltaDMPYaw;                             // compute new yaw from change
  if (newYaw > (2.0f * (float)M_PI))                            // need 0 <= newYaw <= 2*PI
    newYaw -= 2.0f * (float)M_PI;
  if (newYaw < 0)
    newYaw += 2.0f * (float)M_PI;

  deltaMagYaw = newMagYaw - newYaw;                             // compute difference
  if (deltaMagYaw >= (float)M_PI)
    deltaMagYaw = deltaMagYaw - 2.0f * (float)M_PI;
  if (deltaMagYaw <= -(float)M_PI)
    deltaMagYaw = (2.0f * (float)M_PI + deltaMagYaw);

  newYaw += deltaMagYaw/4;                                      // apply some of the correction

  if (newYaw > (2.0f * (float)M_PI))                                            // need 0 <= newYaw <= 2*PI
    newYaw -= 2.0f * (float)M_PI;
  if (newYaw < 0)
    newYaw += 2.0f * (float)M_PI;

  m_lastYaw = newYaw;

  if (newYaw > (float)M_PI)
    newYaw -= 2.0f * (float)M_PI;

  m_fusedEulerPose[VEC3_Z] = newYaw;                            // fill in output yaw value
        
  MPUQuaternionEulerToQuaternion(m_fusedEulerPose, m_fusedQuaternion);
}
*/









