#include "imucal.h"
#include "math.h"

#include "pwm.h"
#include "mpu9250.h"
#include "delay.h"


//#define halfT   0.5 * 0.005                    // 采样周期的一半
//#define Gyro_G 	0.0610351f	
//#define Gyro_Gr 0.0010653f	


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


static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;  
static float exInt = 0, eyInt = 0, ezInt = 0;   // 按比例缩小积分误差
static double samplingTime = 0;
static uint32_t timeForYawCalculate = 0;


void imuUpdate(ImuData *tarData)
{
	float ax = tarData->accelRaw[0];
    float ay = tarData->accelRaw[1];
	float az = tarData->accelRaw[2];
	float gx = tarData->gyroRaw.newData[0];
	float gy = tarData->gyroRaw.newData[1];
	float gz = tarData->gyroRaw.newData[2];
	
	float recipNorm;
	float vx, vy, vz;
	float ex, ey, ez;
	
    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q1q1 = q1*q1;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;
	
	if(ax * ay * az == 0)
	{
		return;
	}
	
	gx /= RADTODEG; // Degree transform back to radian
	gy /= RADTODEG;
	gz /= RADTODEG;
	
	recipNorm = invSqrt(ax*ax + ay*ay + az*az);
	ax *= recipNorm;
	ay *= recipNorm;
	az *= recipNorm;
	
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
	
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*(samplingTime /2);
    q1 = q1 + ( q0*gx + q2*gz - q3*gy)*(samplingTime /2);
    q2 = q2 + ( q0*gy - q1*gz + q3*gx)*(samplingTime /2);
    q3 = q3 + ( q0*gz + q1*gy - q2*gx)*(samplingTime /2); 

	recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
	
	samplingTime = (currentTime() - timeForYawCalculate) /1000000.0;
	timeForYawCalculate = currentTime();	
	
	tarData->actualYaw.lastData   = tarData->actualYaw.newData;
	tarData->actualYaw.newData   += samplingTime * tarData->gyroRaw.newData[2];
	
	if(tarData->actualYaw.newData > 180.0)
	{
		tarData->actualYaw.newData = (tarData->actualYaw.newData - 180) - 180;
	}
	if(tarData->actualYaw.newData < -180.0)
	{
		tarData->actualYaw.newData = 180 - (- tarData->actualYaw.newData - 180);
	}
	
	tarData->actualPitch.lastData = tarData->actualPitch.newData;
    tarData->actualPitch.newData  = asin(-2 * q1 * q3 + 2 * q0 * q2)* RADTODEG;
	tarData->actualRoll.lastData  = tarData->actualRoll.newData;
    tarData->actualRoll.newData   = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1)* RADTODEG;
}


void pidControl(ImuData *tarData)
{
	tarData->pidRoll.pidPout = tarData->pidRoll.pidP * (tarData->actualRoll.newData - tarData->targetRoll);
	tarData->pidRoll.pidDout = tarData->pidRoll.pidD * (tarData->actualRoll.newData - tarData->actualRoll.lastData);
	
	tarData->pidPitch.pidPout = tarData->pidPitch.pidP * (tarData->actualPitch.newData - tarData->targetPitch);
	tarData->pidPitch.pidDout = tarData->pidPitch.pidD * (tarData->actualPitch.newData - tarData->actualPitch.lastData);
	
	tarData->pidYaw.pidDout = tarData->pidYaw.pidD * (tarData->actualYaw.newData - tarData->actualYaw.lastData);
	
	tarData->pidRoll.pidFinalOut  = tarData->pidRoll.pidPout  + tarData->pidRoll.pidIout  + tarData->pidRoll.pidDout;
	tarData->pidPitch.pidFinalOut = tarData->pidPitch.pidPout + tarData->pidPitch.pidIout + tarData->pidPitch.pidDout;
	tarData->pidYaw.pidFinalOut   = tarData->pidYaw.pidDout;
}


void motorUpdate(ImuData *tarData, RespondMess *tarMessData)
{
	float motor[4];
	uint8_t i;
	
	tarMessData->thrust = tarData->targetThrust;
	
	motor[0] = tarData->targetThrust + tarData->pidPitch.pidFinalOut - tarData->pidRoll.pidFinalOut - tarData->pidYaw.pidFinalOut;
	motor[1] = tarData->targetThrust - tarData->pidPitch.pidFinalOut - tarData->pidRoll.pidFinalOut + tarData->pidYaw.pidFinalOut;
	motor[2] = tarData->targetThrust - tarData->pidPitch.pidFinalOut + tarData->pidRoll.pidFinalOut + tarData->pidYaw.pidFinalOut;
	motor[3] = tarData->targetThrust + tarData->pidPitch.pidFinalOut + tarData->pidRoll.pidFinalOut - tarData->pidYaw.pidFinalOut;
	
	motor[0] = motor[0] /(2000);
	motor[1] = motor[1] /(2000);
	motor[2] = motor[2] /(2000);
	motor[3] = motor[3] /(2000);
	
	for(i=0; i<4; i++)
	{
		if(motor[i] <0.0)
		{
			motor[i] = 0;
		}
		if(motor[i] >1.0)
		{
			motor[i] = 1;
		}
	}
	
	if(tarData->targetThrust > 500)
	{
		motorPwmFlash((uint16_t)(motor[0]*Moto_PwmMax),    // M2
					  (uint16_t)(motor[1]*Moto_PwmMax),    // M1
					  (uint16_t)(motor[2]*Moto_PwmMax),    // M3
					  (uint16_t)(motor[3]*Moto_PwmMax));   // M4  ????
	}
	else
	{
		motorPwmFlash(0, 0, 0, 0);
	}

	tarMessData->motor2 = (uint8_t)(motor[0]*100); 
	tarMessData->motor3 = (uint8_t)(motor[2]*100); 
	tarMessData->motor4 = (uint8_t)(motor[3]*100); 
	tarMessData->motor1 = (uint8_t)(motor[1]*100);
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
	//tarData->accelOffset[2] = tarData->accelOffset[2] - CONSTANT_G_PARA;
}
			
		
/*
#define twoKp      1.0f
#define twoKi      0.05f
static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;  
static float integralFBx = 0, integralFBy = 0, integralFBz = 0;

void imuUpdate(ImuData *tarData) 
{
	float ax = tarData->accelRaw[0];
    float ay = tarData->accelRaw[1];
	float az = tarData->accelRaw[2];
	float gx = tarData->gyroRaw.newData[0];
	float gy = tarData->gyroRaw.newData[1];
	float gz = tarData->gyroRaw.newData[2];
	
	static uint32_t timeForYawCalculate = 0;
	
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	
	//tarData->actualYaw += (currentTime() - timeForYawCalculate) * gz /1000000;
	//timeForYawCalculate = currentTime();

	// 如果加速计各轴的数均是0，那么忽略该加速度数据。否则在加速计数据归一化处理的时候，会导致除以0的错误。
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!( ((ax >= - EPSINON) && (ax <= EPSINON)) && 
		  ((ay >= - EPSINON) && (ay <= EPSINON)) && 
	      ((az >= - EPSINON) && (az <= EPSINON)))) 
	{
		// 把加速度计的数据进行归一化处理。其中invSqrt是平方根的倒数，
		// 使用平方根的倒数而不是直接使用平方根的原因是使得下面的ax，ay，az的运算速度更快。
		// 通过归一化处理后，ax，ay，az的数值范围变成-1到+1之间。
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// 根据当前四元数的姿态值来估算出各重力分量。用于和加速计实际测量出来的各重力分量进行对比，从而实现对四轴姿态的修正。
		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;

		// 使用叉积来计算估算的重力和实际测量的重力这两个重力向量之间的误差。
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// 把上述计算得到的重力差进行积分运算，积分的结果累加到陀螺仪的数据中，用于修正陀螺仪数据。
		// 积分系数是Ki，如果Ki参数设置为0，则忽略积分运算。
		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) 
		{
			integralFBx += twoKi * halfex * (1.0f / SAMPLINGFREQ); // integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;  // apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else 
		{
			integralFBx = 0.0f; // prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// 把上述计算得到的重力差进行比例运算。比例的结果累加到陀螺仪的数据中，用于修正陀螺仪数据。比例系数为Kp。
		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	// 通过上述的运算，我们得到了一个由加速计修正过后的陀螺仪数据。接下来要做的就是把修正过后的陀螺仪数据整合到四元数中。
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq)); // pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// 把上述运算后的四元数进行归一化处理。得到了物体经过旋转后的新的四元数。
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	
	tarData->actualPitch = asin(-2 * q1 * q3 + 2 * q0* q2) *RADTODEG;
    tarData->actualRoll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1) *RADTODEG;
	tarData->actualYaw   = atan2f( 2 * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3 ) *RADTODEG;
}


void imuUpdateWithMag(ImuData *tarData) 
{
	float ax = tarData->accelRaw[0];
    float ay = tarData->accelRaw[1];
	float az = tarData->accelRaw[2];
	float gx = tarData->gyroRaw.newData[0];
	float gy = tarData->gyroRaw.newData[1];
	float gz = tarData->gyroRaw.newData[2];
	float mx = tarData->magRaw[0];
	float my = tarData->magRaw[1];
	float mz = tarData->magRaw[2];
	
	float recipNorm;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	
	// 如果地磁传感器各轴的数均是0，那么忽略该地磁数据。否则在地磁数据归一化处理的时候，会导致除以0的错误。
    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if(!( ((mx >= - EPSINON) && (mx <= EPSINON)) && 
		  ((my >= - EPSINON) && (my <= EPSINON)) && 
	      ((mz >= - EPSINON) && (mz <= EPSINON)))) 
	{
		imuUpdate(tarData);
		return;
	}

    // 如果加速计各轴的数均是0，那么忽略该加速度数据。否则在加速计数据归一化处理的时候，会导致除以0的错误。
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!( ((ax >= - EPSINON) && (ax <= EPSINON)) && 
		  ((ay >= - EPSINON) && (ay <= EPSINON)) && 
	      ((az >= - EPSINON) && (az <= EPSINON)))) 
	{
		// 把加速度计的数据进行归一化处理。
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// 把地磁的数据进行归一化处理。
		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// 预先进行四元数数据运算，以避免重复运算带来的效率问题。
		// Auxiliary variables to avoid repeated arithmetic
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth’s magnetic field
		hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
		hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
		bx = sqrt(hx * hx + hy * hy);
		bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// 根据当前四元数的姿态值来估算出各重力分量Vx，Vy，Vz和各地磁分量Wx，Wy，Wz。
		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
		halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
		halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
		halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

		// 使用叉积来计算重力和地磁误差。
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// 把上述计算得到的重力和磁力差进行积分运算，
		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) 
		{
			integralFBx += twoKi * halfex * (1.0f / sampleFreq); // integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;  // apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else 
		{
			integralFBx = 0.0f; // prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// 把上述计算得到的重力差和磁力差进行比例运算。
		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	// 把由加速计和磁力计修正过后的陀螺仪数据整合到四元数中。
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq)); // pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// 把上述运算后的四元数进行归一化处理。得到了物体经过旋转后的新的四元数。
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	
	tarData->actualPitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;
    tarData->actualRoll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;
	tarData->actualYaw   = atan2f( 2 * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3 )*57.3;
}
*/	


/*
void Control(ImuData *tarData)
{
	static float rol_i, pit_i, yaw_p;
	float errorRoll, errorPitch;
	
	errorRoll  = tarData->actualRoll  - (tarData->targetRoll  - 150.0) /12 ;
	errorPitch = tarData->actualPitch + (tarData->actualPitch - 150.0) /12;
	
	rol_i += errorRoll;
	if(rol_i>2000)
	{
		rol_i=2000;
	}
	if(rol_i<-2000)
	{
		rol_i=-2000;
	}

	tarData->pidRoll.pidPout =   tarData->pidRoll.pidP * errorRoll;
	tarData->pidRoll.pidDout = - tarData->pidRoll.pidD * tarData->gyroRaw.newData[1];
	tarData->pidRoll.pidIout =   tarData->pidRoll.pidI * tarData->pidRoll.pidDout;

	pit_i += errorPitch;
	if(pit_i>2000)
	{
		pit_i=2000;
	}
	if(pit_i<-2000)
	{
		pit_i=-2000;
	}

	tarData->pidPitch.pidPout = tarData->pidPitch.pidP * errorPitch;
	tarData->pidPitch.pidDout = tarData->pidPitch.pidD * tarData->gyroRaw.newData[0];
	tarData->pidPitch.pidIout = tarData->pidPitch.pidI * pit_i;
	
	if(tarData->targetYaw<140.0||tarData->targetYaw>160.0)
	{
		tarData->gyroRaw.newData[2] = tarData->gyroRaw.newData[2] + (tarData->targetYaw - 150.0)*2;
	}
	yaw_p += tarData->gyroRaw.newData[2]; // * 0.0609756f * 0.002f;
	
	if(yaw_p >20)
	{
		yaw_p = 20;
	}
	if(yaw_p <-20)
	{
		yaw_p = -20;
	}

	tarData->pidYaw.pidPout = tarData->pidYaw.pidP * yaw_p;
	tarData->pidYaw.pidDout = tarData->pidYaw.pidD * tarData->gyroRaw.newData[2];				   
	
	if(tarData->targetThrust <500)
	{		
		pit_i = 0;
		rol_i = 0;
		yaw_p = 0;
	}
	
	tarData->pidRoll.pidFinalOut  = - tarData->pidRoll.pidPout  - tarData->pidRoll.pidIout  + tarData->pidRoll.pidDout;
	tarData->pidPitch.pidFinalOut =   tarData->pidPitch.pidPout + tarData->pidPitch.pidIout + tarData->pidPitch.pidDout;
	tarData->pidYaw.pidFinalOut   =   tarData->pidYaw.pidPout   + tarData->pidYaw.pidIout   + tarData->pidYaw.pidDout;
}
*/

