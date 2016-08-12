    BBFlight project began in the winter of last year which is my first project basing in STM32, 
    significantly,i study electronic control and printed circuit designing through this.Thank to my 
    brother,he taught me so much about progamming and circuit design ,in addition, he influenced my 
    coding ideal deeply, inspired me a lot. In this year, up to now, this summer holiday, i finished 
    the client by python for pc and the software for drone controlling.But it also exist many problems 
    of BBFlight, like unstable command communication, low efficiency about drone cpu using,etc.



/*************************************************

几个重要的宏 
------------------------
ms5611.h

\#define GYROSCALE250DPS<br>
\#define GYROSCALE500DPS<br>
\#define GYROSCALE1000DPS<br>
\#define GYROSCALE2000DPS<br>

\#define ACCELSCALE2G<br>
\#define ACCELSCALE4G<br>
\#define ACCELSCALE8G<br>
\#define ACCELSCALE16G<br>

\#define MAGSCALE14BITS<br>
\#define MAGSCALE16BITS<br>


16Mhz 下USB有大bug:
-----------------------
###This error is not because of the 16Mhz crystal.
    And finally,i found that the usb crashing problem is raise by the overflowing of the USB RX FIFO.
    When the slave misses the connection, the software of slave can not clear the fifo immediately(
    it comes to outtime processing) ,at the same time,the client will not stop send command.
    So that the usb moduel crashing happens if the fifo overflow.
    For slove this problem, i added some methods to interrupt the cilent transmitting when lost connection.


ms5611.h
----------------------
\#define ConversionTime 10000<br>
\#define DefaultPresInitFilterTime 50<br>

`高度转换要运行够50次滤波`


system_config.h
---------------------
\#define _USBDEBUG_<br>


\====================================================
中断总结
----------------------------------------------------
NVIC_PriorityGroup_2
抢占有两位 响应有两位 加起来可以表达16个优先级

                    PreemptionPriority(抢占)  SubPriority（响应)
    TIM1 SYSTEM_LOOP          0                     1                 sign
    NRFIT                     0	                    0
    TIM2 DEBUG                1                     1                 sign

    USB  WAKEUP               0                     1
    USB  RXIRQ    	          1                     0
    USART                     1	                    0
\====================================================

      M1                  M2
      T3C4                T4C4
      MOTO_1              MOTO_0

                  Y
                  |-X



      M3                  M4
      T3C3                T4C3
      MOTO_2              MOTO_3

\***************************************************
