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
 * @brief		This function delays the program.(STUPID Version).
 * @param 		None
 * @retval		None
 */
void Delay_Stupid(int number){
	for(; number != 0; number--);
}

extern float k;
extern void Delay (__IO uint32_t nTime);
extern int auto_manual;
extern int isadjust;
extern int stopMotor;
extern int Getk_up(void);
extern int Getk_dp(void);
int test_exti = 0;
/**
 * @brief		This function handles external interrupt line 4.
 * @param		None
 * @retval      None
 */
void EXTI4_IRQHandler(void){
    test_exti = 1;
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
 * @retval      None
 */
extern int count;
extern uint16_t motorDelayTime;
extern int k_dp, k_up;
void LEDSD_ERROR(void);
void LEDSD_UP(void);
void LEDSD_DP(void);
void LEDSD_CLEAR(void);
void EXTI9_5_IRQHandler(void){
    test_exti = 2;
    if(!auto_manual){// At automatic mode
        if(EXTI_GetITStatus(EXTI_Line7) != RESET){// Have EXTI line interrupt in PA.07
            delay_ms(500);
            if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7) == Bit_RESET){
                while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7) == Bit_RESET);
                delay_ms(500);
                k += 0.1;// Up key
                Getk_up();
                Getk_dp();
            }
            EXTI_ClearITPendingBit(EXTI_Line7);
        }
        if(EXTI_GetITStatus(EXTI_Line6) != RESET){// Have EXTI line interrupt in PA.06
            delay_ms(500);
            if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6) == Bit_RESET){
                while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6) == Bit_RESET);
                delay_ms(500);
                k -= 0.1;// Down key
                Getk_up();
                Getk_dp();
            }
            EXTI_ClearITPendingBit(EXTI_Line6);
        }
        if(EXTI_GetITStatus(EXTI_Line5) != RESET){// Have EXTI line interrupt in PA.05
            delay_ms(500);
            if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5) == Bit_RESET){
                while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5) == Bit_RESET);
                delay_ms(500);
                k = 0.6;// Reset key
                stopMotor = 0;// Motor move.
                Getk_up();
                Getk_dp();
            }
            EXTI_ClearITPendingBit(EXTI_Line5);
        }
    }
    else{// At manual mode
        if(EXTI_GetITStatus(EXTI_Line7) != RESET){// Have EXTI line interrupt in PA.07
            delay_ms(500);
            if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7) == Bit_RESET){
                GPIO_SetBits(GPIOB, GPIO_Pin_14);
                if(stopMotor != 2){// Curtain is not fully open.
                    while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7) == Bit_RESET){
                        if(stopMotor == 1){
                            stopMotor = 0;
                        }
                        // Open curtain.
 count+=4;
                    		GPIO_ResetBits(GPIOB,GPIO_Pin_14);
		GPIO_SetBits(GPIOA,GPIO_Pin_0);
	//	GPIO_ResetBits(GPIOA,GPIO_Pin_0);
		delay_ms(1);
		GPIO_ResetBits(GPIOB,GPIO_Pin_14);
		GPIO_SetBits(GPIOA,GPIO_Pin_0);
		//GPIO_SetBits(GPIOA,GPIO_Pin_0);
		delay_ms(3);
			LEDSD_CLEAR();
			LEDSD_UP();
			LEDSD_ERROR();
		GPIO_ResetBits(GPIOB,GPIO_Pin_15);
		GPIO_SetBits(GPIOA,GPIO_Pin_3);
	//	GPIO_ResetBits(GPIOA,GPIO_Pin_0);
		delay_ms(1);
		GPIO_ResetBits(GPIOB,GPIO_Pin_15);
		GPIO_SetBits(GPIOA,GPIO_Pin_3);
	//	GPIO_SetBits(GPIOA,GPIO_Pin_0);
		delay_ms(3);
			LEDSD_CLEAR();
			LEDSD_DP();
			LEDSD_ERROR();
		GPIO_SetBits(GPIOB,GPIO_Pin_14);
		GPIO_ResetBits(GPIOB,GPIO_Pin_4);
	//	GPIO_ResetBits(GPIOA,GPIO_Pin_0);
		delay_ms(1);
		GPIO_SetBits(GPIOB,GPIO_Pin_14);
		GPIO_ResetBits(GPIOA,GPIO_Pin_0);
	//	GPIO_SetBits(GPIOA,GPIO_Pin_0);
		delay_ms(3);
			LEDSD_CLEAR();
			LEDSD_UP();
			LEDSD_ERROR();
		GPIO_SetBits(GPIOB,GPIO_Pin_15);
		GPIO_ResetBits(GPIOA,GPIO_Pin_3);
	//	GPIO_ResetBits(GPIOA,GPIO_Pin_0);
		delay_ms(1);
		GPIO_SetBits(GPIOB,GPIO_Pin_15);
		GPIO_ResetBits(GPIOA,GPIO_Pin_0);
	//	GPIO_SetBits(GPIOA,GPIO_Pin_0);
		delay_ms(3);
			LEDSD_CLEAR();
			LEDSD_DP();
			LEDSD_ERROR();

                        // GPIO_SetBits(GPIOB, GPIO_Pin_15);
                        // LEDSD_CLEAR();
                        // LEDSD_UP();
                        // LEDSD_ERROR();
                        // delay_ms(motorDelayTime);
                        // GPIO_ResetBits(GPIOB, GPIO_Pin_15);
                        // LEDSD_CLEAR();
                        // LEDSD_DP();
                        // LEDSD_ERROR();
                        // delay_ms(motorDelayTime);
                        if(stopMotor == 2){
                            break;
                        }
                        count += 4;
                    }
                    delay_ms(500);
                }
                else{// Curtain is fully open || curtain has been fully open then partially closed.
                    if(count != 21000){
                        stopMotor = 0;// Motor move.
                    }
                }
            }
            EXTI_ClearITPendingBit(EXTI_Line7);
        }
        if(EXTI_GetITStatus(EXTI_Line6) != RESET){// Have EXTI line interrupt in PA.06
            delay_ms(500);
            if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6) == Bit_RESET){
                GPIO_ResetBits(GPIOB, GPIO_Pin_14);
                if(stopMotor != 1){// Curtain is not fully closed.
                    while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6) == Bit_RESET){
                        if(stopMotor == 2){
                            stopMotor = 0;
                        }
                        // Close curtain.
count-=4;
   	GPIO_ResetBits(GPIOB,GPIO_Pin_14);
		GPIO_SetBits(GPIOA,GPIO_Pin_0);
	//	GPIO_ResetBits(GPIOA,GPIO_Pin_0);
		delay_ms(1);
		GPIO_ResetBits(GPIOB,GPIO_Pin_14);
		GPIO_SetBits(GPIOA,GPIO_Pin_0);
	//	GPIO_SetBits(GPIOA,GPIO_Pin_0);
		delay_ms(3);
			LEDSD_CLEAR();
			LEDSD_UP();
			LEDSD_ERROR();
	  GPIO_SetBits(GPIOB,GPIO_Pin_15);
		GPIO_ResetBits(GPIOA,GPIO_Pin_3);
	//	GPIO_ResetBits(GPIOA,GPIO_Pin_0);
		delay_ms(1);
		GPIO_SetBits(GPIOB,GPIO_Pin_15);
		GPIO_ResetBits(GPIOA,GPIO_Pin_3);
	//	GPIO_SetBits(GPIOA,GPIO_Pin_0);
		delay_ms(3);
			LEDSD_CLEAR();
			LEDSD_DP();
			LEDSD_ERROR();
		GPIO_SetBits(GPIOB,GPIO_Pin_14);
		GPIO_ResetBits(GPIOA,GPIO_Pin_0);
	//	GPIO_ResetBits(GPIOA,GPIO_Pin_0);
		delay_ms(1);
		GPIO_SetBits(GPIOB,GPIO_Pin_14);
		GPIO_ResetBits(GPIOA,GPIO_Pin_0);
	//	GPIO_SetBits(GPIOA,GPIO_Pin_0);
		delay_ms(3);
			LEDSD_CLEAR();
			LEDSD_UP();
			LEDSD_ERROR();
		
				GPIO_ResetBits(GPIOB,GPIO_Pin_15);
		GPIO_SetBits(GPIOA,GPIO_Pin_3);
	//	GPIO_ResetBits(GPIOA,GPIO_Pin_0);
		delay_ms(1);
		GPIO_ResetBits(GPIOB,GPIO_Pin_15);
		GPIO_SetBits(GPIOA,GPIO_Pin_3);
	//	GPIO_SetBits(GPIOA,GPIO_Pin_0);
		delay_ms(3);
			LEDSD_CLEAR();
			LEDSD_DP();
			LEDSD_ERROR();

                        // GPIO_SetBits(GPIOB, GPIO_Pin_15);
                        // LEDSD_CLEAR();
                        // LEDSD_UP();
                        // LEDSD_ERROR();
                        // delay_ms(motorDelayTime);
                        // GPIO_ResetBits(GPIOB, GPIO_Pin_15);
                        // LEDSD_CLEAR();
                        // LEDSD_DP();
                        // LEDSD_ERROR();
                        // delay_ms(motorDelayTime);
                        if(stopMotor == 1){
                            break;
                        }
                        count-=4;
                    }
                    delay_ms(500);
                }
                else{// Curtain is fully closed || Curtain is closed & open.
                    if(count != 0){
                        stopMotor = 0;// Motor move.
                    }
                }
            }
            EXTI_ClearITPendingBit(EXTI_Line6);
        }
        if(EXTI_GetITStatus(EXTI_Line5) != RESET){// Have EXTI line interrupt in PA.05
            delay_ms(500);
            if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5) == Bit_RESET){
                while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5) == Bit_RESET);
                delay_ms(500);
                auto_manual = 0;// Change manual mode to automatic mode.
                stopMotor = 0;// Motor move.
                isadjust = 1;// Need to adjust the current impluse after toggling mode.
            }
            EXTI_ClearITPendingBit(EXTI_Line5);
        }
    }
    if(EXTI_GetITStatus(EXTI_Line8) != RESET){// Curtain is closed.

        test_exti = 3;
        count = 0;
        isadjust = 0;// Do not need to adjust.
        stopMotor = 1;// Stop motor.
        EXTI_ClearITPendingBit(EXTI_Line8);
    }
    if(EXTI_GetITStatus(EXTI_Line9) != RESET){// Curtain is open.
        test_exti = 3;
        count = 21000;
        isadjust = 0;// Do not need to adjust.
        stopMotor = 2;// Stop motor.
        EXTI_ClearITPendingBit(EXTI_Line9);
    }
}

/**
 * @brief		This function handles external interrupt line 15..10.
 * @param		None
 * @retval      None
 */
// void EXTI15_10_IRQHandler(void){
//     if(EXTI_GetITStatus(                                   EXTI_Line10) != RESET){
//         test_exti = 4;
//         delay_ms(1000);
//         if(motorDelayTime == 1){
//             motorDelayTime = 3;
//         }
//         else{
//             motorDelayTime = 1;
//         }
//         EXTI_ClearITPendingBit(EXTI_Line10);
//     }
// }

extern u8 btMessage[10];
extern int k_up, k_dp;
extern float k;
extern int charCount;
extern char ch;
void USART1_IRQHandler(void){
    // int RX_status;
    // int i;
    test_exti = 5;
	delay_ms(1000);
	// RX_status = USART_GetFlagStatus(USART1, USART_FLAG_RXNE);
	if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE)) {
        ch = USART_ReceiveData(USART1);
        // if(ch == 's'){// Stop motor.
        //     test = 0;
        // }
        // if(ch == 'c'){// Close the curtain.
        //     if(stopMotor != 1){// Curtain is not fully closed.
        //         for(i = 4200; i > 0; i--){
        //             if(stopMotor == 2){
        //                 stopMotor = 0;
        //             }
        //             // Close curtain motor.
        //
        //             if(stopMotor == 1){
        //                 break;
        //             }
        //             count --;
        //         }
        //     }
        //     else{// Curtain is fully closed || Curtain is closed & open.
        //         if(count != 0){
        //             stopMotor = 0;// Motor move.
        //         }
        //     }
        // }
        // if(ch == 'o'){// Open curtain.
        //     if(stopMotor != 2){// Curtain is not fully open.
        //         for(i = 4200; i > 0; i--){
        //             if(stopMotor == 1){
        //                 stopMotor = 0;
        //             }
        //             // Open curtain motor.
        //
        //             if(stopMotor == 2){
        //                 break;
        //             }
        //             count++;
        //         }
        //     }
        //     else{// Curtain is fully open || curtain has been fully open then partially closed.
        //         if(count != 21000){
        //             stopMotor = 0;// Motor move.
        //         }
        //     }
        // }
        if(ch == 'a'){// Change to automatic mode.
    		auto_manual = 0;// Change manual mode to automatic mode.
    		stopMotor = 0;// Motor move.
    	//	isadjust = 1;// Need to adjust the current impluse after toggling mode.
    	}
    	if(ch == 'm'){// Change to maunal mode.
    		auto_manual = 1;// Change automatic mode to manual mode.
    	}
    	if(ch == 'k'){// Set the scale factor k.
    		charCount = 0;
    		btMessage[charCount] = ch;
    		charCount++;
    		if(charCount == 10){
    			charCount = 0;
    		}
    	}
    	if(ch >= '0' && ch <= '9'){// Set the scale factor k.
    		if(charCount == 1){
    			btMessage[charCount] = ch;
    			charCount++;
    		}
    		else{
    			if(charCount == 2){
    				btMessage[charCount] = ch;
    				charCount++;
    				// Update scale factor k.
    				k_up = btMessage[1] - '0';
    				k_dp = btMessage[2] - '0';
    				k = k_up + 0.1 * k_dp;
    			}
    		}
    	}
		// USART_SendData(USART1 , USART_ReceiveData(USART1)+1);
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
		delay_ms(1000);
	}
}
