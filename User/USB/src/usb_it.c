/* Includes ------------------------------------------------------------------*/
#include "hw_config.h"
//#include "usb_it.h"
#include "stm32f10x_exti.h"
#include "usb_lib.h"
#include "usb_istr.h"



/*******************************************************************************
* Function Name  : USB_IRQHandler
* Description    : This function handles USB Low Priority interrupts
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  USB_Istr();
}


/*******************************************************************************
* Function Name  : USB_FS_WKUP_IRQHandler
* Description    : This function handles USB WakeUp interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/


void USBWakeUp_IRQHandler(void)

{
  EXTI_ClearITPendingBit(EXTI_Line18);
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

