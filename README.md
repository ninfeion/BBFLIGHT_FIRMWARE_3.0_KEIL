/*************************************************
������Ҫ�ĺ�
 
mpu5611.h
#define GYROSCALE250DPS
#define GYROSCALE500DPS
#define GYROSCALE1000DPS
#define GYROSCALE2000DPS

#define ACCELSCALE2G
#define ACCELSCALE4G
#define ACCELSCALE8G
#define ACCELSCALE16G

#define MAGSCALE14BITS
#define MAGSCALE16BITS

imucal.h
#define SAMPLINGFREQ 200

system_config.h
#define USB_DEBUG
#define USE_LPF_FILTER

====================================================

16Mhz ��USB�д�bug

====================================================
ms5611.h

#define ConversionTime 10000
#define DefaultPresInitFilterTime 50

�߶�ת��Ҫ���й�50���˲�

====================================================
system_config.h

#define _USBDEBUG_


====================================================
�ж��ܽ�
NVIC_PriorityGroup_2 ��ռ����λ ��Ӧ����λ ���������Ա��16�����ȼ�
                 PreemptionPriority(��ռ)  SubPriority����Ӧ)
TIM1 SYSTEM_LOOP 0 								1                 sign
NRFIT			 0								0
TIM2 DEBUG       1              				1				  sign

USB  WAKEUP      0 								1
USB  RXIRQ    	 1 								0
USART            1								0
====================================================

  M1                  M2
  T3C4                T4C4
  MOTO_1              MOTO_0

               Y
               |-X



  M3                  M4
  T3C3                T4C3
  MOTO_2              MOTO_3

**************************************************/
