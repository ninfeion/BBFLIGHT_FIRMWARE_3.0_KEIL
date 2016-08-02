#include "systeminit.h"
#include "spinrf.h"
#include "pwm.h"
#include "adc.h"
#include "system_loop.h"
#include "i2c.h"
#ifdef _USE6050_
	#include "mpu6050.h"
#else
	#include "mpu9250.h"
#endif
#include "MS5611.h"
#include "delay.h"
#ifdef _USBDEBUG_
	#include "usb2com.h"
	#include "debug.h"
#else
	#include "usart.h"
#endif

struct SYSTEM_STATE BBSYSTEM;

/*
uint8_t sysInit(void)
{
  sysclockInit();
  systickInit();
  ledInit();
	spi2NrfInit();
	if(NRF24L01_Check() == 1)			BBSYSTEM.NRFONLINE = 1;
	//NRF check后ce会是低电平所以设置模式要在后面
	#ifdef _SLAVEMODE_
		nrfSlaveMode();
	#endif

	adcInit();
	i2cInit();
	mpu9250Init();
	if(MPU9250_Check() == TRUE)		BBSYSTEM.MPU9250ONLINE = 1;
	ms5611Init();
	pwmInit();	
	systemLoopInit();
	#ifdef _USBDEBUG_
		USBVCOM_Init();
		//DEBUG_TIM2_Init();
	#else
		//USARTInit(9600);
	#endif	
	
	//enableNrfIqr();
  return 1;
}
*/

uint8_t sysclockInit(void)
{
    ErrorStatus HSEStartUpStatus;
	
    RCC_DeInit();/* RCC重置 */
    RCC_HSEConfig(RCC_HSE_ON); /*使能HSE*/
    HSEStartUpStatus = RCC_WaitForHSEStartUp();
    if(HSEStartUpStatus == SUCCESS)   //外部晶振使能成功
    {
        RCC_HCLKConfig(RCC_SYSCLK_Div1); /* 配置HCLK = SYSCLK */
        RCC_PCLK2Config(RCC_HCLK_Div1); /* 配置PCLK2 = HCLK */
        RCC_PCLK1Config(RCC_HCLK_Div2); /* 配置PCLK1 = HCLK/2 */

        FLASH_SetLatency(FLASH_Latency_2);
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
		
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);  /* RCC_PLLSource_HSE_Div1为外置晶振的分频系数;RCC_PLLMul_9为倍频数 */
        RCC_PLLCmd(ENABLE);
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET); //等待pll工作
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);   /* 选定PLL为系统主时钟 */
        while(RCC_GetSYSCLKSource() != 0x08); //判断pll是否为系统时钟
		return 1;

	}
	else
		return 0;
}

uint8_t systickInit(void)
{
	/*
	对SysTick的操作，仅保留以下1个函数：  
	void SysTick_CLKSourceConfig(uint32_t SysTick_CLKSource); 
	功能：配置SysTick时钟源  
	参数：
	SysTick_CLKSource：  
	SysTick_CLKSource_HCLK_Div8:根据时钟除以8选为SysTick时钟源。
	SysTick_CLKSource_HCLK:根据SysTick时钟选为时钟源。 

	uint32_t SysTick_Config(uint32_t ticks);  
	功能：配置SysTick重装计数值，使能中断，启动运行
	参数：  ticks：24位以内的重装值 
	返回：1失败,0成功 */
	return SysTick_Config(72); //计1000次是1us
}

uint8_t ledInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;//定义GPIO初始化结构体

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE); //使能GPOIA,GPIOB复用时钟	   
   	

    /*
    RCC_APB2Periph_AFIO 功能复用IO时钟 
    RCC_APB2Periph_GPIOA GPIOA时钟 
    RCC_APB2Periph_GPIOB GPIOB时钟 
    RCC_APB2Periph_GPIOC GPIOC时钟 
    RCC_APB2Periph_GPIOD GPIOD时钟 
    RCC_APB2Periph_GPIOE GPIOE时钟 
    RCC_APB2Periph_ADC1 ADC1时钟 
    RCC_APB2Periph_ADC2 ADC2时钟 
    RCC_APB2Periph_TIM1 TIM1时钟 
    RCC_APB2Periph_SPI1 SPI1时钟 
    RCC_APB2Periph_USART1 USART1时钟 
    RCC_APB2Periph_ALL 全部APB2外设时钟
    */
   	
    //LED GPIO初始化
    //LED_0->D3->PA4
    //LED_3->D4->PA7
    //LED_2->D5->PA6
    //LED_1->D6->PA5
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 ;   
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;   
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
    GPIO_Init(GPIOA, &GPIO_InitStructure); 
  
    LedA_off;
    LedB_off;
    LedC_off;
    LedD_off;
  
    /*
	GPIO_Mode_AIN = 0x0,     //模拟输入   
	GPIO_Mode_IN_FLOATING = 0x04, //悬空输入   
	GPIO_Mode_IPD = 0x28,    //下拉输入   
	GPIO_Mode_IPU = 0x48,    //上拉输入   
	GPIO_Mode_Out_OD = 0x14, //开漏输出   
	GPIO_Mode_Out_PP = 0x10,  //推挽输出   
	GPIO_Mode_AF_OD = 0x1C,   //开漏复用   
	GPIO_Mode_AF_PP = 0x18    //推挽复用 

    1、浮空输入GPIO_IN_FLOATING ——浮空输入，可以做KEY识别，RX1
    2、带上拉输入GPIO_IPU——IO内部上拉电阻输入
    3、带下拉输入GPIO_IPD—— IO内部下拉电阻输入
    4、模拟输入GPIO_AIN ——应用ADC模拟输入，或者低功耗下省电
    5、开漏输出GPIO_OUT_OD ——IO输出0接GND，IO输出1，悬空，需要外接上拉电阻，才能实现输出高电平。当输出为1时，IO口的状态由上拉电阻拉高电平，但由于是开漏输出模式，这样IO口也就可以由外部电路改变为低电平或不变。可以读IO输入电平变化，实现C51的IO双向功能
    6、推挽输出GPIO_OUT_PP ——IO输出0-接GND， IO输出1 -接VCC，读输入值是未知的
    7、复用功能的推挽输出GPIO_AF_PP ——片内外设功能（I2C的SCL,SDA）
    8、复用功能的开漏输出GPIO_AF_OD——片内外设功能（TX1,MOSI,MISO.SCK.SS）

    对于串口，假如最大波特率只需115.2k，那么用2M的GPIO的引脚速度就够了，既省电也噪声小。
    对于I2C接口，假如使用400k波特率，若想把余量留大些，那么用2M的GPIO的引脚速度或许不够，这时可以选用10M的GPIO引脚速度。
    对于SPI接口，假如使用18M或9M波特率，用10M的GPIO的引脚速度显然不够了，需要选用50M的GPIO的引脚速度。

    IIC 推荐10MHZ
    SPI 推荐50MHZ
    */
    return 1;
}
  




  
