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
#include "stdlib.h"
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
uint16_t motorDelayTime = 1;// 1 second or 2 seconds, default is 1 second.
// [Debug]
int auto_manual = 1;// 0 stands for auto, 1 stands for manual. Default is auto.
//y=-5.648*x^3+35.42*x^2-82.44*x+80.52
int count = 0;// Counting for the number of impluse.
int tem = 0;// Temporary parameter.
int adjustLengthImpluse, openLengthImpluse;
// [Debug] DO NOT NEED TO ADJUST
int isadjust = 1;// 1 stands for need to adjust the number of count, 0 stands for do not need.
int stopMotor = 0;// 0 stands for move, 1 stands for close stop, 2 stands for open stop. Default is move.
u8 btMessage[10]={0};// Bluetooth incoming message array.
char ch;
int charCount = 0;

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
void SysTickInit(void);
void EXTIInit(void);
void setPB0_7_OPP(void);
void setPB10_11_OPP(void);
void LEDSD_Test(void);
void LEDSD_ERROR(void);
void setPA11_12_OPP(void);
void LEDSD_X(int);
void delay_ms(uint16_t);
void LEDSD_UP(void);
void LEDSD_DP(void);
void LEDSD_CLEAR(void);
void PWMInit(void);
float CalLength(u16);
float CalAverageLength(u16);
void setPB14_15_OPP(void);
void MotorOpen(int);
void MotorClose(int);
int Getk_up(void);
int Getk_dp(void);
void setPA4_8_IPU(void);
void setPA9_AFPP(void);
void setPA10_IF(void);
void USART1_Config(void);
void setPA0_OPP(void);
void setPA3_OPP(void);
void setPA15_IPU(void);
void setPB3_OPP(void);
void setPB4_OPP(void);
void setPB12_OPP(void);
void setPB13_OPP(void);
void BT_Update(void);

//float volts, length;
/**
 * @brief  Main program.
 * @param  None
 * @retval None
 */
int main(void)
{
	/* Initialize */
	// Initialize the pins.
	setPA15_IPU();
	setPA0_OPP();
	setPB3_OPP();
	setPB4_OPP();
	setPB12_OPP();
	setPB13_OPP();
	setPA3_OPP();
	setPA10_IF();
	setPA9_AFPP();
	USART1_Config();
	setPA4_8_IPU();
	setPA11_12_OPP();
	// setPA7_IPU();
	// setPA6_IPU();
	// setPA5_IPU();
	// setPA4_IPU();
	setPB0_7_OPP();
	setPB10_11_OPP();
	setPB14_15_OPP();
	// Initialize the alternative functions.
	EXTIInit();
	SysTickInit();
	AdcInit();
	LEDSD_Test();
	// Initialize the parameters.
	temt6000 = 0;
	avertemt6000 = 0;
	isalive = 0;
	k = 0.6;
	k_up = 0;
	k_dp = 6;
	length = 0;
	averlength = 0;
	GPIO_ResetBits(GPIOB,GPIO_Pin_4);
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

		BT_Update();

		if(auto_manual){// At manual mode.
			// k_up = Getk_up();
			// k_dp = Getk_dp();
			LEDSD_CLEAR();
			LEDSD_UP();
			LEDSD_ERROR();
			delay_ms(10);
			LEDSD_CLEAR();
			LEDSD_DP();
			LEDSD_ERROR();
			delay_ms(10);
		}
		else{// At automatic mode.
			adjustLengthImpluse = (averlength - 26) * 250;
			// openLengthImpluse = 4500 - k * averlux * 208;
			openLengthImpluse = 21000 - k * averlux * 972;
			if(openLengthImpluse > 21000){
				openLengthImpluse = 21000;
			}
			if(openLengthImpluse < 0){
				openLengthImpluse = 0;
			}
			// if(count > 4500){
			// 	count = 4500;
			// }
			if(count > 21000){
				count = 21000;
			}
			if(count < 0){
				count = 0;
			}

			if(!isadjust){// Do not need to adjust.
				if(openLengthImpluse - 400 > count){
					MotorOpen(100);
				}
				else{
					if(count - 400 > openLengthImpluse){
						MotorClose(100);
					}
					else{
						LEDSD_CLEAR();
						LEDSD_UP();
						LEDSD_X(k_up);
						delay_ms(10);
						LEDSD_CLEAR();
						LEDSD_DP();
						LEDSD_X(k_dp);
						delay_ms(10);
					}
			    }
			}
			else{// Need to adjust.
				if((adjustLengthImpluse - count) > 50){
					count = adjustLengthImpluse;
				}
				isadjust = 0;
			}
		}
	}// while(1)
}
