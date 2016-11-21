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
#include "math.h"
/* Public Defines */
#define temt6000 pa0
#define avertemt6000 averpa0
#define irs pa1
#define averirs averpa1

/* Global Variables */
u16 pa0, averpa0;
u16 pa1, averpa1;
int isalive;// The number shows whether STM32 is alive
float k;// Scale factor K
int k_dp, k_up;
float lux, averlux;// Lux(lx) value
float length, averlength;
int auto_manual = 0;// 0 stands for auto, 1 stands for manual. Default is auto.
//y=-5.648*x^3+35.42*x^2-82.44*x+80.52

/* Function Declaration */
void AdcInit(void);
void Delay(u32);
u16 GetAdc1(u8);
u16 GetAdc2(u8);
u16 GetAdc1Average(u8, u8);
u16 GetAdc2Average(u8, u8);
float CalAverageLux(u16);
float CalLux(u16);
int KeyScan(GPIO_TypeDef*, u16);
void setPA7_IPU(void);
void setPA6_IPU(void);
void SysTickInit(void);
void setPA5_IPU(void);
void EXTIInit(void);
void setPB0_7_OPP(void);
void setPB10_11_OPP(void);
void LEDSD_Test(void);
void setPA11_12_OPP(void);
void LEDSD_X(int);
void delay_ms(uint16_t);
void LEDSD_UP(void);
void LEDSD_DP(void);
void LEDSD_CLEAR(void);
void PWMInit(void);
float CalLength(u16);
float CalAverageLength(u16);


/**
 * @brief  Main program.
 * @param  None
 * @retval None
 */
int main(void)
{
	/* Initialize */
	// Initialize the pins.
	setPA7_IPU();
	setPA6_IPU();
	setPA5_IPU();
	setPB0_7_OPP();
	setPB10_11_OPP();
	setPA11_12_OPP();
	// Initialize the alternative functions.
	EXTIInit();
	SysTickInit();
	AdcInit();
	LEDSD_Test();
	// PWMInit();
	// Motor_Init(10000,1000,1);
	// Initialize the parameters.
	temt6000 = 0;
	avertemt6000 = 0;
	isalive = 0;
	k = 0.6;
	length = 0;
	averlength = 0;

  /* Infinite loop */
	  while (1){
		isalive++;
		// Get TEMT6000 output value
		ADC_Cmd(ADC2, DISABLE);
		ADC_Cmd(ADC1, ENABLE);
		temt6000 = GetAdc1(ADC_Channel_1);
		avertemt6000 = GetAdc1Average(ADC_Channel_1, 100);
		ADC_Cmd(ADC1, DISABLE);
		ADC_Cmd(ADC2, ENABLE);
		irs = GetAdc2(ADC_Channel_2);
		averirs = GetAdc2Average(ADC_Channel_2, 100);
		if(temt6000){
			lux = CalLux(temt6000);
			averlux = CalAverageLux(avertemt6000);
		}
		if(irs){
			length = CalLength(irs);
			averlength = CalAverageLength(averirs);
		}
		k_up = k / 1;
		k_dp = (int)(k * 10) % 10;
		LEDSD_CLEAR();
		LEDSD_UP();
		LEDSD_X(k_up);
		delay_ms(10);
		LEDSD_CLEAR();
		LEDSD_DP();
		LEDSD_X(k_dp);
		delay_ms(10);
	 	// TIM2_TIM3_PWM(1000,10);
	}
}
