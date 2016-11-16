/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
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
#include "stm32f10x.h"

/* Public Defines */
#define temt6000 pa0
#define avertemt6000 averpa0

/* Global Variables */
u16 pa0, averpa0;
int isalive;// The number shows whether STM32 is alive
uint8_t Up_KeyStatus, Down_KeyStatus, Reset_KeyStatus;// Keys status
float k;// Scale factor K
float lux, averlux;// Lux(lx) value


/* Function Declaration */
void AdcInit(void);
void Delay(u32);
u16 GetAdc(u8);
u16 GetAdcAverage(u8, u8);
float CalAverageLux(u16);
float CalLux(u16);
int KeyScan(GPIO_TypeDef*, u16);
void setPA7_IPU(void);
void setPA6_IPU(void);
void SysTickInit(void);
void setPA5_IPU(void);


/**
 * @brief  Main program.
 * @param  None
 * @retval None
 */
int main(void)
{
	/* Initialize */
	SysTickInit();
	AdcInit();
	setPA7_IPU();
	setPA6_IPU();
	setPA5_IPU();
	temt6000 = 0;
	avertemt6000 = 0;
	isalive = 0;
	k = 0.6;
	Up_KeyStatus = Bit_SET;
	Down_KeyStatus = Bit_SET;
	Reset_KeyStatus = Bit_SET;

  /* Infinite loop */
  while (1)
  {
		isalive++;
		Up_KeyStatus = KeyScan(GPIOA, GPIO_Pin_7);
		Down_KeyStatus = KeyScan(GPIOA, GPIO_Pin_6);
		Reset_KeyStatus = KeyScan(GPIOA, GPIO_Pin_5);
		// Judge key value
		if(Up_KeyStatus == Bit_RESET){
			k += 0.1;// k incresements
		}
		if(Down_KeyStatus == Bit_RESET){
			k -= 0.1;// k decresemets
		}
		if(Reset_KeyStatus == Bit_RESET){
			k = 0.6;// Reset
		}
		// Get TEMT6000 output value
		temt6000 = GetAdc(ADC_Channel_1);
		avertemt6000 = GetAdcAverage(ADC_Channel_1, 100);
		if(temt6000){
			lux = CalLux(temt6000);
			averlux = CalAverageLux(avertemt6000);
		}
  }
}
