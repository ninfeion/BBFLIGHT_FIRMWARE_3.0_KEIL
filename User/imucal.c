#include "imucal.h"
#include "math.h"

#include "usb2com.h"

#define Kp 2.0f                  // 比例增益支配率收敛到加速度计/磁强计
#define Ki 0.005f                // 积分增益支配率的陀螺仪偏见的衔接
#define halfT 0.5 * 0.005        // 采样周期的一半
#define Gyro_Gr 0.0010653f	

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;          
float exInt = 0, eyInt = 0, ezInt = 0;        // 按比例缩小积分误差
float gx=0,gy=0,gz=0,ax=0,ay=0,az=0;          // 经过滤波的六轴数据

#define        kfa   0.98
#define        kfan  1.0-kfa
#define        kfg   0.80
#define        kfgn  1.0-kfg

float invSqrt(float number) 
{
    volatile long i;
    volatile float x, y;
    volatile const float f = 1.5F;

    x = number * 0.5F;
    y = number;
    i = * (( long * ) &y);
    i = 0x5f375a86 - ( i >> 1 );
    y = * (( float * ) &i);
    y = y * ( f - ( x * y * y ) );
    return y;
}

void IMUupdate(ImuData *tarData)
{
	float ax = tarData->accelRaw[0];
    float ay = tarData->accelRaw[1];
	float az = tarData->accelRaw[2];
	float gx = tarData->gyroRaw[0];
	float gy = tarData->gyroRaw[1];
	float gz = tarData->gyroRaw[2];
	
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;
	
    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;
	
	//if(ax*ay*az == 0)
	//{
	//	return;
	//}

	//ax=ax*kfa+kfan*axi;
	//ay=ay*kfa+kfan*ayi;
	//az=az*kfa+kfan*azi;

    //gx=gx*kfg+kfgn*gxi;
    //gy=gy*kfg+kfgn*gyi;
    //gz=gz*kfg+kfgn*gzi;
	
	gx *= Gyro_Gr;
	gy *= Gyro_Gr;
	gz *= Gyro_Gr;
	
	norm = 1/invSqrt(ax*ax + ay*ay + az*az);
	ax = ax /norm;
	ay = ay /norm;
	az = az /norm;
	
	vx = 2*(q1q3 - q0q2);
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	
	ex = (ay*vz - az*vy);
	ey = (az*vx - ax*vz);
	ez = (az*vy - ay*vx);
	
	exInt = exInt + ex*Ki;
	eyInt = eyInt + ey*Ki;
	ezInt = ezInt + ez*Ki;
	
	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;
	
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = q1 + ( q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + ( q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + ( q0*gz + q1*gy - q2*gx)*halfT; 

	norm = 1/invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;
	
    tarData->actualPitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;
    tarData->actualRoll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;
	tarData->actualYaw   = atan2f( 2 * (q0 * q1 + q2 * q3), q0q0 - q1q1 - q2q2 + q3q3 )*57.3;
}