/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"

void TimingDelay_Decrement(void);

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */

void SysTick_Handler(void)
{
	TimingDelay_Decrement();
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
 * @brief			This function delays the program, in millisecond.
 * @param 		MS: The number of millisecond.
 * @retval		None
 */
void delay_ms(uint16_t  MS){
  uint32_t temp_load,temp_ctrl=0;
	temp_load = MS*(SystemCoreClock/1000);
	SysTick->LOAD = (temp_load & SysTick_LOAD_RELOAD_Msk)-1;
	SysTick->VAL = 0;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
	                 SysTick_CTRL_TICKINT_Msk |
	                 SysTick_CTRL_ENABLE_Msk ;
	do{
		  temp_ctrl = SysTick->CTRL;
    }while(temp_ctrl&0x01&&!(temp_ctrl & SysTick_CTRL_COUNTFLAG_Msk));

		SysTick->CTRL &=~(SysTick_CTRL_ENABLE_Msk |SysTick_CTRL_TICKINT_Msk);
		SysTick->VAL = 0;
}
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

/**
 * @brief			This function delays the program.(STUPID Version).
 * @param 		None
 * @retval		None
 */
void Delay_Stupid(int number){
	for(; number != 0; number--);
}

extern float k;
extern void Delay (__IO uint32_t nTime);
extern int auto_manual;

/**
 * @brief		This function handles external interrupt line 4.
 * @param		None
 * @retval	None
 */
void EXTI4_IRQHandler(void){
    if(EXTI_GetITStatus(EXTI_Line4) != RESET){// Have EXTI line interrupt in PA.04
        delay_ms(500);
        if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4) == Bit_RESET){
            while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4) == Bit_RESET);
            delay_ms(500);
            auto_manual = 1;// Change automatic mode to manual mode.
        }
        EXTI_ClearITPendingBit(EXTI_Line4);
    }

}

/**
 * @brief		This function handles external interrupt line 9..5.
 * @param		None
 * @retval	None
 */
void EXTI9_5_IRQHandler(void){
    if(!auto_manual){// At automatic mode
        if(EXTI_GetITStatus(EXTI_Line7) != RESET){// Have EXTI line interrupt in PA.07
            delay_ms(500);
            if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7) == Bit_RESET){
                while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7) == Bit_RESET);
                delay_ms(500);
                k += 0.1;// Up key
            }
            EXTI_ClearITPendingBit(EXTI_Line7);
        }
        if(EXTI_GetITStatus(EXTI_Line6) != RESET){// Have EXTI line interrupt in PA.06
            delay_ms(500);
            if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6) == Bit_RESET){
                while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6) == Bit_RESET);
                delay_ms(500);
                k -= 0.1;// Down key
            }
            EXTI_ClearITPendingBit(EXTI_Line6);
        }
        if(EXTI_GetITStatus(EXTI_Line5) != RESET){// Have EXTI line interrupt in PA.05
            delay_ms(500);
            if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5) == Bit_RESET){
                while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5) == Bit_RESET);
                delay_ms(500);
                k = 0.6;// Reset key
            }
            EXTI_ClearITPendingBit(EXTI_Line5);
        }
    }
    else{// At manual mode
        if(EXTI_GetITStatus(EXTI_Line7) != RESET){// Have EXTI line interrupt in PA.07
            delay_ms(500);
            if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7) == Bit_RESET){
                while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7) == Bit_RESET){
                  // Motor rotates at right direction.
                }
                delay_ms(500);
            }
            EXTI_ClearITPendingBit(EXTI_Line7);
        }
        if(EXTI_GetITStatus(EXTI_Line6) != RESET){// Have EXTI line interrupt in PA.06
            delay_ms(500);
            if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6) == Bit_RESET){
                while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6) == Bit_RESET){
                  // Motor rotates at left direction.
                }
                delay_ms(500);
            }
            EXTI_ClearITPendingBit(EXTI_Line6);
        }
        if(EXTI_GetITStatus(EXTI_Line5) != RESET){// Have EXTI line interrupt in PA.05
            delay_ms(500);
            if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5) == Bit_RESET){
                while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5) == Bit_RESET);
                delay_ms(500);
                auto_manual = 0;// Change manual mode to automatic mode.
            }
            EXTI_ClearITPendingBit(EXTI_Line5);
        }
    }
}
