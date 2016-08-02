/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PLATFORM_CONFIG_H
#define __PLATFORM_CONFIG_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f10x.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Uncomment the line corresponding to the STMicroelectronics evaluation board
   used to run the example */

/*Unique Devices IDs register set*/


#define         ID1          (0x1FFFF7E8)
#define         ID2          (0x1FFFF7EC)
#define         ID3          (0x1FFFF7F0)


  #define USB_DISCONNECT                      GPIOA
  #define USB_DISCONNECT_PIN                  GPIO_Pin_10
  #define RCC_APB2Periph_GPIO_DISCONNECT      RCC_APB2Periph_GPIOA
//  #define EVAL_COM1_IRQHandler                USART1_IRQHandler 
 

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __PLATFORM_CONFIG_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
