#include "usart.h"

uint8_t USARTInit(uint32_t bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;//定义GPIO初始化结构体
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);    
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

  /* Configure the NVIC Preemption Priority Bits */  
  /* Configure one bit for preemption priority */
  /* 优先级组 说明了抢占优先级所用的位数，和子优先级所用的位数   在这里是1， 7 */    
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	  
  
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;			     	//设置串口2中断
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	     	//抢占优先级 0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;				//子优先级为0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					//使能
  NVIC_Init(&NVIC_InitStructure);
	
	//USART2_TX   PA.2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure); //初始化PA2
   
  //USART2_RX	  PA.3
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);  //初始化PA3
	
	//USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART2, &USART_InitStructure); //初始化串口
	
	/* Enable USARTx Receive and Transmit interrupts */
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);                    //使能接收中断
  USART_ITConfig(USART2, USART_IT_TXE, ENABLE);						//使能发送缓冲空中断   
 
  /* Enable the USARTx */
  USART_Cmd(USART2, ENABLE);	
	
	return 1;
}

