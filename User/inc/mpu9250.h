#ifndef _MPU9250_H_
#define _MPU9250_H_
#include "i2c.h"

#include "stm32f10x.h"
#include "systeminit.h"

//寄存器定义
#define RA_SELF_TEST_X_GYRO			0X00
#define RA_SELF_TEST_Y_GYRO			0X01
#define RA_SELF_TEST_Z_GYRO			0X02

#define RA_SELF_TEST_X_ACCEL		0X0D
#define RA_SELF_TEST_Y_ACCEL		0X0E
#define RA_SELF_TEST_Z_ACCEL		0X0F

#define RA_XG_OFFSET_H				0X13
#define RA_XG_OFFSET_L				0X14
#define RA_YG_OFFSET_H				0X15
#define RA_YG_OFFSET_L				0X16
#define RA_ZG_OFFSET_H				0X17
#define RA_ZG_OFFSET_L				0X18

#define RA_SMPLRT_DIV				0X19 //陀螺仪采样率 典型值为0X07  1000/(1+7)=125HZ
#define RA_CONFIG					0X1A //低通滤波器  典型值0x06 5hz
#define RA_GYRO_CONFIG				0X1B //陀螺仪测量范围 0X18 正负2000度 (0x00 +-2g;)  ( 0x08 +-4g;)  (0x10 +-8g;)  (0x18 +-16g)
#define RA_ACCEL_CONFIG_1			0X1C //加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)	
#define RA_ACCEL_CONFIG_2			0X1D //加速度计低通滤波器 0x06 5hz

#define RA_LP_ACCEL_ODR				0X1E
#define RA_WOM_THR					0X1F
#define RA_FIFO_EN					0X23

#define RA_I2C_MST_CTRL       	    0x24
#define RA_I2C_SLV0_ADDR            0x25
#define RA_I2C_SLV0_REG             0x26
#define RA_I2C_SLV0_CTRL            0x27
#define RA_I2C_SLV1_ADDR            0x28
#define RA_I2C_SLV1_REG             0x29
#define RA_I2C_SLV1_CTRL            0x2A
#define RA_I2C_SLV2_ADDR            0x2B
#define RA_I2C_SLV2_REG             0x2C
#define RA_I2C_SLV2_CTRL            0x2D
#define RA_I2C_SLV3_ADDR            0x2E
#define RA_I2C_SLV3_REG             0x2F
#define RA_I2C_SLV3_CTRL            0x30
#define RA_I2C_SLV4_ADDR            0x31
#define RA_I2C_SLV4_REG             0x32
#define RA_I2C_SLV4_DO              0x33
#define RA_I2C_SLV4_CTRL            0x34
#define RA_I2C_SLV4_DI              0x35
#define RA_I2C_MST_STATUS           0x36
#define RA_INT_PIN_CFG              0x37
#define RA_INT_ENABLE               0x38
#define RA_INT_STATUS               0x3A

#define RA_ACCEL_XOUT_H				0X3B  //加速度计输出数据
#define RA_ACCEL_XOUT_L				0X3C
#define RA_ACCEL_YOUT_H				0X3D
#define RA_ACCEL_YOUT_L				0X3E
#define RA_ACCEL_ZOUT_H				0X3F
#define RA_ACCEL_ZOUT_L				0X40

#define RA_TEMP_OUT_H				0X41  //温度计输出数据
#define RA_TEMP_OUT_L				0X42

#define RA_GYRO_XOUT_H				0X43  //陀螺仪输出数据
#define RA_GYRO_XOUT_L				0X44
#define RA_GYRO_YOUT_H				0X45
#define RA_GYRO_YOUT_L				0X46
#define RA_GYRO_ZOUT_H				0X47
#define RA_GYRO_ZOUT_L				0X48

//#define RA_MAG_XOUT_L				0x03
//#define RA_MAG_XOUT_H				0x04
//#define RA_MAG_YOUT_L				0x05
//#define RA_MAG_YOUT_H				0x06
//#define RA_MAG_ZOUT_L				0x07
//#define RA_MAG_ZOUT_H				0x08

#define RA_USER_CTRL        		0X6A //用户配置 当为0X10时使用SPI模式
#define RA_PWR_MGMT_1				0X6B //电源管理1 典型值为0x00
#define RA_PWR_MGMT_2				0X6C //电源管理2 典型值为0X00

#define RA_WHO_AM_I					0X75 //器件ID MPU9250默认ID为0X71
#define VAL_DEVICE_ID       	    0X71

#define MPU9250_ADDRESS    			0xD2 //0x69 Left shift //LSB = AD0 = 1 of front of 7 bits // read + 1

//#define USE_MAG_PASSMODE   

//#ifdef USE_MAG_PASSMODE
	#define MAG_ADDRESS      	 	0x18 //0x0c left shift
	#define VAL_MAG_DEVICE_ID    	0x48
	// Read-only Reg
	#define AK8963_WIA              0x00
	#define AK8963_INFO             0x01
	#define AK8963_ST1              0x02
	#define AK8963_HXL              0x03
	#define AK8963_HXH              0x04
	#define AK8963_HYL              0x05
	#define AK8963_HYH              0x06
	#define AK8963_HZL              0x07
	#define AK8963_HZH              0x08
	#define AK8963_ST2              0x09
	// Write/Read Reg
	#define AK8963_CNTL1            0x0A
	#define AK8963_CNTL2            0x0B
	#define AK8963_ASTC             0x0C
	#define AK8963_TS1              0x0D
	#define AK8963_TS2              0x0E
	#define AK8963_I2CDIS           0x0F
	// Read-only Reg ( ROM )
	#define AK8963_ASAX             0x10
	#define AK8963_ASAY             0x11
	#define AK8963_ASAZ             0x12
	// Status
	#define AK8963_STATUS_DRDY      0x01
	#define AK8963_STATUS_DOR       0x02
	#define AK8963_STATUS_HOFL      0x08
//#endif
/*************************************************************************
|Bit      |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
+---------+-------+-------+-------+-------+-------+-------+-------+-------+
|CONFIG   |       | FIFO_ |  EXT_SYNC_SET[2:0]    |     DLPF_CFG[2:0]     |
|		  |       | MODE  |                       |                       |
+---------+-------+-------+-------+---------------+-------+---------------+
|GYRO_    |       |       |       |  GYRO_FS_SEL  |       |   FCHOICE_B   |[4:3]Gyro Full Scale Select:00 = +250 dps, 01= +500 dps, 10 = +1000 dps, 11 = +2000 dps
|CONFIG   |       |       |       |    [1:0]      |       |     [1:0]     |						       131 LSB		  65.5 LSB      32.8 LSB		16.4 LSB       
+---------+-------+-------+-------+---------------+-------+-------+-------+
|ACCEL_   |       |       |       |  ACCEL_FS_SEL |       |       |       |[4:3]Accel Full Scale Select:±2g (00), ±4g (01), ±8g (10), ±16g (11)
|CONFIG   |       |       |       |    [1:0]      |       |       |       |														 16384 LSB  8192 LSB	 4096 LSB		2048 LSB
+---------+-------+-------+-------+-------+-------+-------+-------+-------+
|ACCEL_   |       |       |       |       | ACCEL_FCHOICE |  A_DLPF_CFG   |        
|CONFIG_2 |       |       |       |       |      _B       |               |		 
+---------+-------+-------+-------+-------+-------+-------+---------------+
|PWR_MGMT |H_RESET| SLEEP | CYCLE | GYRO_ |PD_PTAT|       CLKSEL[2:0]     |
|_1       |       |       |       |STANDBY|       |                       |
+---------+-------+-------+-------+-------+-------+-------+-------+-------+
|PWR_MGMT |       |       |DIS_XA |DIS_YA |DIS_ZA |DIS_XG |DIS_YG |DIS_ZG |
|_2       |       |       |       |       |       |       |       |       |
+---------+-------+-------+-------+-------+-------+-------+-------+-------+
|			    |       |       |       |       |       |       |       |       |
|			    |       |       |       |       |       |       |       |       |
***************************************************************************
|     |            ACCELEROMETER             |
| LPF |  BandW  |  Delay  | Noise  | Sample  |
+-----+---------+---------+--------+---------+
|  x  |  1130Hz |  0.75ms | 250Hz  |  4kHz   |
|  0  |   460Hz |  1.94ms | 250Hz  |  1kHz   |
|  1  |   184Hz |  5.80ms | 250Hz  |  1kHz   |
|  2  |    92Hz |  7.80ms | 250Hz  |  1kHz   |
|  3  |    41Hz | 11.80ms | 250Hz  |  1kHz   |
|  4  |    20Hz | 19.80ms | 250Hz  |  1kHz   |
|  5  |    10Hz | 35.70ms | 250Hz  |  1kHz   |
|  6  |     5Hz | 66.96ms | 250Hz  |  1kHz   |
|  7  |   460Hz |  1.94ms | 250Hz  |  1kHz   |
********************************************************
|     |          GYROSCOPE         |    TEMPERATURE    |
| LPF |  BandW  |  Delay  | Sample |  BandW  |  Delay  |
+-----+---------+---------+--------+---------+---------+
|  x  |  8800Hz | 0.064ms |  32kHz |  4000Hz |  0.04ms |
|  x  |  3600Hz |  0.11ms |  32kHz |  4000Hz |  0.04ms |
|  0  |   250Hz |  0.97ms |   8kHz |  4000Hz |  0.04ms |
|  1  |   184Hz |   2.9ms |   1kHz |   188Hz |   1.9ms |
|  2  |    92Hz |   3.9ms |   1kHz |    98Hz |   2.8ms |
|  3  |    41Hz |   5.9ms |   1kHz |    42Hz |   4.8ms |
|  4  |    20Hz |   9.9ms |   1kHz |    20Hz |   8.3ms |
|  5  |    10Hz | 17.85ms |   1kHz |    10Hz |  13.4ms |
|  6  |     5Hz | 33.48ms |   1kHz |     5Hz |  18.6ms |
|  7  |  3600Hz |  0.17ms |   8kHz |  4000Hz |  0.04ms |
*******************************************************/
 
 /*磁力计最多需要9ms来完成转换*/
 
 
//#define GYROSCALE250DPS
//#define GYROSCALE500DPS
//#define GYROSCALE1000DPS
#define GYROSCALE2000DPS
#ifdef GYROSCALE250DPS
  #define GYROLSB 131.0f
	#else
	#ifdef GYROSCALE1000DPS
		#define GYROLSB 32.8f 
		#else
		#ifdef GYROSCALE500DPS
			#define GYROLSB 65.5f
			#else
			#ifdef GYROSCALE2000DPS
				#define GYROLSB 16.4f
			#endif
		#endif
	#endif
#endif

//#define ACCELSCALE2G
//#define ACCELSCALE4G
//#define ACCELSCALE8G
#define ACCELSCALE16G
#ifdef ACCELSCALE2G // LSB/g
	#define ACCELLSB 16384.0f
	#else
	#ifdef ACCELSCALE4G
		#define ACCELLSB 8192.0f
		#else
		#ifdef ACCELSCALE8G
			#define ACCELLSB	4096.0f
			#else
			#ifdef ACCELSCALE16G
				#define ACCELLSB 2048.0f					
			#endif
		#endif
	#endif
#endif

//#define MAGSCALE14BITS
#define MAGSCALE16BITS
#ifdef MAGSCALE14BITS
	#define MAGLSB 0.6f // uT/LSB
	#else
	#ifdef MAGSCALE16BITS
	#define MAGLSB 0.15f 
	#endif
#endif


void mpu9250Init(void);
uint8_t Get_MPU9250_ID(void);
uint8_t Get_AK8963_ID_Bypass(void);
bool MPU9250_Check(void);
bool AK8963_Check_Bypass(void);
void READ_MPU9250_ACCEL_RAW(int16_t *ACCELDATA);
void READ_MPU9250_GYRO_RAW(int16_t *GYRODATA);
int16_t READ_MPU9250_TEMP_RAW(void);
void READ_MPU9250_Bypass_MAG_RAW(int16_t *MAGDATA);

void accelAndGyroOffset(ImuData *tarData);

#endif
