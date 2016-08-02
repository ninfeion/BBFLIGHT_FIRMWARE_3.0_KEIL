#include "mpu6050.h"
#include "delay.h"

void MPU6050_Init(void)
{
	Single_Write(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x80);      //PWR_MGMT_1    -- DEVICE_RESET 1
    delay_ms(50);
    Single_Write(MPU6050_ADDRESS, MPU6050_RA_SMPLRT_DIV, 0x00);      //SMPLRT_DIV    -- SMPLRT_DIV = 0  Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    Single_Write(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x03);      //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
    Single_Write(MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 0 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0);  // INT_PIN_CFG   -- INT_LEVEL_HIGH, INT_OPEN_DIS, LATCH_INT_DIS, INT_RD_CLEAR_DIS, FSYNC_INT_LEVEL_HIGH, FSYNC_INT_DIS, I2C_BYPASS_EN, CLOCK_DIS
	Single_Write(MPU6050_ADDRESS, MPU6050_RA_CONFIG, 0x03);  //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
	//MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    // Accel scale 8g (4096 LSB/g)
    Single_Write(MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 2 << 3);
}

unsigned char Get_MPU6050_ID(void)
{
	return Single_Read(MPU6050_ADDRESS , MPU6050_RA_WHO_AM_I);
}

//¶Áacc
void MPU6050AccRead(int16_t *accData)
{
    uint8_t BUF[6];
	BUF[0]=Single_Read(MPU6050_ADDRESS,MPU6050_RA_ACCEL_XOUT_H); 
	BUF[1]=Single_Read(MPU6050_ADDRESS,MPU6050_RA_ACCEL_XOUT_L);
	BUF[2]=Single_Read(MPU6050_ADDRESS,MPU6050_RA_ACCEL_XOUT_H);
	BUF[3]=Single_Read(MPU6050_ADDRESS,MPU6050_RA_ACCEL_YOUT_L);
	BUF[4]=Single_Read(MPU6050_ADDRESS,MPU6050_RA_ACCEL_ZOUT_H);
	BUF[5]=Single_Read(MPU6050_ADDRESS,MPU6050_RA_ACCEL_ZOUT_L);
    accData[0] = (int16_t)((BUF[0] << 8) | BUF[1]);
    accData[1] = (int16_t)((BUF[2] << 8) | BUF[3]);
    accData[2] = (int16_t)((BUF[4] << 8) | BUF[5]);


}
//¶Ágyro
void MPU6050GyroRead(int16_t *gyroData)
{
	uint8_t BUF[6];//»º´æ
	BUF[0]=Single_Read(MPU6050_ADDRESS,MPU6050_RA_GYRO_XOUT_H); 
	BUF[1]=Single_Read(MPU6050_ADDRESS,MPU6050_RA_GYRO_XOUT_L);
	BUF[2]=Single_Read(MPU6050_ADDRESS,MPU6050_RA_GYRO_YOUT_H);
	BUF[3]=Single_Read(MPU6050_ADDRESS,MPU6050_RA_GYRO_YOUT_L);
	BUF[4]=Single_Read(MPU6050_ADDRESS,MPU6050_RA_GYRO_ZOUT_H);
	BUF[5]=Single_Read(MPU6050_ADDRESS,MPU6050_RA_GYRO_ZOUT_L);
    gyroData[0] = (int16_t)((BUF[0] << 8) | BUF[1]) ;
    gyroData[1] = (int16_t)((BUF[2] << 8) | BUF[3]) ;
    gyroData[2] = (int16_t)((BUF[4] << 8) | BUF[5]) ;
}



