#include "stm32f10x.h"
#include "math.h"

/**
 * @brief	Set PA7 pin at input pull-up mode.
 * @param 	None
 * @retval	None
 */
void setPA0_OPP(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void setPB12_OPP(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void setPB13_OPP(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
 * @brief	Set PA7 pin at input pull-up mode.
 * @param 	None
 * @retval	None
 */

void setPB3_OPP(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}


/**
 * @brief	Set PA7 pin at input pull-up mode.
 * @param 	None
 * @retval	None
 */

void setPB4_OPP(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
 * @brief	Set PA7 pin at input pull-up mode.
 * @param 	None
 * @retval	None
 */

void setPA3_OPP(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
 * @brief 	Set PA.01 pin at analogue input mode, 2MHz speed.
 * @param  	None
 * @retval 	None
 */
void setPA1_AIN ( void ) {
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // Enable the peripheral clock
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; // bin to be set
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // Analogue input
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
 * @brief 	Set PA.02 pin at analogue input mode, 2MHz speed.
 * @param  	None
 * @retval 	None
 */
void setPA2_AIN ( void ) {
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // Enable the peripheral clock
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; // bin to be set
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // Analogue input
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
 * @brief	Set PA.04~08 pin at input pull-up mode.
 * @param 	None
 * @retval	None
 */
void setPA4_8_IPU(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_7 | GPIO_Pin_6 | GPIO_Pin_5| GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
* @brief	Set PA.09 pin at alternate function push-pull output mode.
* @param 	None
* @retval	None
*/
void setPA9_AFPP(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO, ENABLE);// Enable the USART & AFIO clock.
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
* @brief	Set PA.10 pin at floating input mode.
* @param 	None
* @retval	None
*/
void setPA10_IF(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

// /**
//  * @brief	Set PA7 pin at input pull-up mode.
//  * @param 	None
//  * @retval	None
//  */
// void setPA7_IPU(void){
// 	GPIO_InitTypeDef GPIO_InitStructure;
// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
// 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
// 	GPIO_Init(GPIOA, &GPIO_InitStructure);
// }
//
// /**
//  * @brief	Set PA6 pin at input pull-up mode.
//  * @param 	None
//  * @retval	None
//  */
// void setPA6_IPU(void){
// 	GPIO_InitTypeDef GPIO_InitStructure;
// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
// 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
// 	GPIO_Init(GPIOA, &GPIO_InitStructure);
// }
//
// /**
//  * @brief	Set PA.05 pin at input pull-up mode.
//  * @param 	None
//  * @retval	None
//  */
// void setPA5_IPU(void){
// 	GPIO_InitTypeDef GPIO_InitStructure;
// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
// 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
// 	GPIO_Init(GPIOA, &GPIO_InitStructure);
// }
//
// /**
//  * @brief	Set PA.04 pin at input pull-up mode.
//  * @param 	None
//  * @retval	None
//  */
// void setPA4_IPU(void){
// 	GPIO_InitTypeDef GPIO_InitStructure;
// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
// 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
// 	GPIO_Init(GPIOA, &GPIO_InitStructure);
// }


/**
* @brief	Set PB.10 and PB.11 pins at ouput push-push mode, 2MHz speed.
* @param	None
* @retval	None
*/
void setPA11_12_OPP(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 |
	GPIO_Pin_11 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
 * @brief	Set PA7 pin at input pull-up mode.
 * @param 	None
 * @retval	None
 */
void setPA15_IPU(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
* @brief	Set PB.00~07 pins at ouput push-push mode, 2MHz speed.
* @param	None
* @retval	None
*/
void setPB0_7_OPP(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |
	GPIO_Pin_1 |
	GPIO_Pin_2 |
	GPIO_Pin_3 |
	GPIO_Pin_4 |
	GPIO_Pin_5 |
	GPIO_Pin_6 |
	GPIO_Pin_7 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/**
 * @brief	Set PB.10 and PB.11 pins at ouput push-push mode, 2MHz speed.
 * @param	None
 * @retval	None
 */
void setPB10_11_OPP(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 |
																GPIO_Pin_11 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}


/**
 * @brief	Set PB.14 and PB.15 pins at ouput push-push mode, 2MHz speed.
 * @param	None
 * @retval	None
 */
void setPB14_15_OPP(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/**
 * @brief	Delay the program.
 * @param 	nTime: Specifies the delay time length, in milliseconds.
 * @retval	None
 */
static __IO uint32_t TimingDelay;
void Delay (__IO uint32_t nTime)
{
	//SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;// Enable the systick.
  TimingDelay = nTime;
	while(TimingDelay != 0);
	//SysTick->CTRL &= SysTick_CTRL_ENABLE_Msk;// Disable the systick.
}

/**
 * @brief	Initialize the ADC1 & ADC2.
 * @param 	None
 * @retval	None
 */
void  AdcInit(void)
{
	ADC_InitTypeDef ADC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2, ENABLE );
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);

	setPA1_AIN();
	setPA2_AIN();

	ADC_DeInit(ADC1);
	ADC_DeInit(ADC2);

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC2, &ADC_InitStructure);

	ADC_Cmd(ADC1, ENABLE);
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
	// ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	// ADC_Cmd(ADC1, DISABLE);
	ADC_Cmd(ADC2, ENABLE);
	ADC_ResetCalibration(ADC2);
	while(ADC_GetResetCalibrationStatus(ADC2));
	ADC_StartCalibration(ADC2);
	while(ADC_GetCalibrationStatus(ADC2));
	// ADC_SoftwareStartConvCmd(ADC2, ENABLE);
	// ADC_Cmd(ADC2, DISABLE);
}


/**
 * @brief	Get ADC1 value.
 * @param 	ch: Choose the ADC channel.
 * @retval	ADC conversion value.
 */
u16 GetAdc1(u8 ch){
	u16 temp;
	// ADC_Cmd(ADC1, ENABLE);
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));
	temp = ADC_GetConversionValue(ADC1);
	// ADC_Cmd(ADC1, DISABLE);
	return temp;
}

/**
 * @brief	Get ADC2 value.
 * @param 	ch: Choose the ADC channel.
 * @retval	ADC conversion value.
 */
u16 GetAdc2(u8 ch){
	// ADC_Cmd(ADC2, ENABLE);
	u16 temp;
	ADC_RegularChannelConfig(ADC2, ch, 1, ADC_SampleTime_239Cycles5 );
	ADC_SoftwareStartConvCmd(ADC2, ENABLE);
	while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC ));
	temp = ADC_GetConversionValue(ADC2);
	// ADC_Cmd(ADC2, DISABLE);
	return temp;
}

/**
 * @brief	Get ADC1 average value.
 * @param 	ADC_Channel_x
 * @param	times
 * @retval	ADC1 average value.
 */
u16 GetAdc1Average(u8 ch,u8 times){
	void delay_ms(uint16_t  MS);
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=GetAdc1(ch);
	}
	return temp_val/times;
}

/**
 * @brief	Get ADC1 average value.
 * @param 	ADC_Channel_x
 * @param	times
 * @retval	ADC2 average value.
 */
u16 GetAdc2Average(u8 ch,u8 times){
	// ADC_Cmd(ADC2, ENABLE);
	void delay_ms(uint16_t  MS);
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=GetAdc2(ch);
		//Delay(5);
		//delay_ms(5);
	}
	// ADC_Cmd(ADC2, DISABLE);
	return temp_val/times;
}

/**
 * @brief	Get average lux value from analogue input pin PA1.
 * @param 	Aver_Analogue_In: The average analogue input value.
 * @retval	The average value of lux.
 */
float CalAverageLux(u16 Aver_Analogue_In){
	float avervolts, averamps, avermicroamps, averlux;
	avervolts = Aver_Analogue_In * 3.3 / 4096.0;
	averamps = avervolts / 10000.0;  // across 10,000 Ohms
	avermicroamps = averamps * 1000000;
	// averlux = avermicroamps * 2.0;
	averlux = avermicroamps - sqrt(10);
	return averlux;
}

/**
 * @brief	Get lux value from analogue input pin PA1.
 * @param 	Analogue_In: The analogue input value.
 * @retval	The average value of lux.
 */
float CalLux(u16 Analogue_In){
	float volts, amps, microamps, lux;
	volts = Analogue_In * 3.3 / 4096.0;
	amps = volts / 10000.0;  // across 10,000 Ohms
	microamps = amps * 1000000;
	// lux = microamps * 2.0;
	lux = microamps - sqrt(10);
	return lux;
}

/* ------------ Infrared Ranging Sensor ------------ */

/**
 * @brief	Get length from analogue input pin PA2.
 * @param 	Analogue_In: The analogue input value.
 * @retval	The average value of lux.
 */
float CalLength(u16 Analogue_In){
	float volts, length;
	volts = Analogue_In * 3.3 / 4096.0;
	length = 42.21*pow(volts, 5.0) -323.1*pow(volts, 4.0) + 965.9*pow(volts, 3.0) -1392*pow(volts, 2.0) + 923.4*volts - 160.1;
	return length;
}

/**
 * @brief	Get average lux value from analogue input pin PA1.
 * @param 	Aver_Analogue_In: The average analogue input value.
 * @retval	The average value of lux.
 */
float CalAverageLength(u16 Aver_Analogue_In){
	float avervolts, averlength;
	avervolts = Aver_Analogue_In * 3.3 / 4096.0;
	averlength = 42.21*pow(avervolts, 5.0) -323.1*pow(avervolts, 4.0) + 965.9*pow(avervolts, 3.0) -1392*pow(avervolts, 2.0) + 923.4*avervolts - 160.1;
	return averlength;
}

/**
 * @brief		Check the key's status.[!Low voltage is valid!!!!]
 * @param		GPIOx: GPIO fields(A~G)
 * @param		GPIO_Pin: GPIO pin number(0~15)
 * @retval		Return Bit_RESET is ON, Bit_SET is OFF.
 */
// uint8_t KeyScan(GPIO_TypeDef* GPIOx, u16 GPIO_Pin){
// 	if(GPIO_ReadInputDataBit(GPIOx, GPIO_Pin) == Bit_RESET){
// 		Delay(50);// Delay to clear the jitter
// 		if(GPIO_ReadInputDataBit(GPIOx, GPIO_Pin) == Bit_RESET){
// 			while(GPIO_ReadInputDataBit(GPIOx,GPIO_Pin) == Bit_RESET);
// 			Delay(5);
// 			return Bit_RESET;
// 		}
// 		else{
// 			return Bit_SET;
// 		}
// 	}
// 	else{
// 		return Bit_SET;
// 	}
// }

/**
 * @brief	Initialize SysTick, and unit is milllisecond.
 * @param	None
 * @retval	None
 */
void SysTickInit(void){
	if(SysTick_Config(SystemCoreClock / 1000)){
		while(1);// Capture error
	}
	//NVIC_SetPriority(SysTick_IRQn, 0);
	//SysTick->CTRL &= ~ SysTick_CTRL_ENABLE_Msk;//Unable the systick.
}

/**
 * @brief	Decrements the TimingDelay variable.
 * @param	None
 * @retval	None
 */
void TimingDelay_Decrement(void){
	if(TimingDelay != 0x00){
		TimingDelay--;
	}
}

/**
 * @brief	Initialize the EXTI(External Interrupt).
 * @param	None
 * @retval	None
 */
void EXTIInit(void){
	void NVICInit(void);
	EXTI_InitTypeDef EXTI_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);// Enable the AFIO clock
	// GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource10);
    // GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource9);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource8);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource7);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource6);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource5);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource4);
	// Configure EXTI line 5, 6, 7, 8
	EXTI_InitStructure.EXTI_Line = EXTI_Line5 | EXTI_Line6 | EXTI_Line7 |  EXTI_Line8 | EXTI_Line4 ;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVICInit();

}

/**
 * @brief	Initialize the NVIC(Nested Vector Interrupt Controller).
 * @param	None
 * @retval	None
 */
void NVICInit(void){
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;// The second pre-emption priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;// The second sub priority
	NVIC_Init(&NVIC_InitStructure);


	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;// The second pre-emption priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;// The second sub priority
	NVIC_Init(&NVIC_InitStructure);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;// The second pre-emption priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;// The second sub priority
	NVIC_Init(&NVIC_InitStructure);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// The highest pre-emption priority.
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;// The highest sub priority.
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	// USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	USART_Cmd(USART1, ENABLE);
}


/* Define LED lights */
// Centeral Up light tube (GPIOA)
#define	LED_CU	GPIO_Pin_12
// Centeral Center light tube (GPIOB)
#define	LED_CC	GPIO_Pin_6
// Centeral Down light tube (GPIOB)
#define	LED_CD	GPIO_Pin_0
// Left Up light tube (GPIOB)
#define	LED_LU	GPIO_Pin_5
// Left Down light tube  (GPIOB)
#define	LED_LD	GPIO_Pin_2
// Right Up light tube (GPIOA)
#define	LED_RU	GPIO_Pin_11
// Right Down light tube (GPIOB)
#define	LED_RD	GPIO_Pin_10
// Point light tube (GPIOB)
#define	LED_POINT	GPIO_Pin_1
// Decimal place groud line(GPIOB)
#define LED_DP_GND	GPIO_Pin_11
// Unit place ground line (GPIOB)
#define LED_UP_GND	GPIO_Pin_7

/**
 * @brief		Test the LED segment display.
 * @param		None
 * @retval		None
 */
void LEDSD_Test(void){
	void LEDSD_8(void);
	GPIO_ResetBits(GPIOB, LED_UP_GND |
												LED_DP_GND);
	GPIO_SetBits(GPIOB, LED_POINT);
	LEDSD_8();
}

/**
 * @brief		Light up the decimal place & light off the unit place on LED segment display.
 * @param		None
 * @retval		None
 */
void LEDSD_DP(void){
	// The decimal place.
	GPIO_ResetBits(GPIOB, LED_DP_GND);
	// The unit place.
	GPIO_SetBits(GPIOB, LED_UP_GND);
}

/**
 * @brief		Light up the unit place & light off the decimal place on LED segment display.
 * @param		None
 * @retval		None
 */
void LEDSD_UP(void){
	// The decimal place.
	GPIO_SetBits(GPIOB, LED_DP_GND);
	// The unit place.
	GPIO_ResetBits(GPIOB, LED_UP_GND);
	// The point place.
	GPIO_SetBits(GPIOB, LED_POINT);
}

/**
 * @brief		Light number X on the LED segment display.(Intergrated the following functions)
 * @param		x: Light number x.
 * @retval		None
 */
void LEDSD_X(int x){
	void LEDSD_0(void);
	void LEDSD_1(void);
	void LEDSD_2(void);
	void LEDSD_3(void);
	void LEDSD_4(void);
	void LEDSD_5(void);
	void LEDSD_6(void);
	void LEDSD_7(void);
	void LEDSD_8(void);
	void LEDSD_9(void);
	void LEDSD_8(void);
	void LEDSD_ERROR(void);

	switch(x){
	case 0:
		LEDSD_0();
		break;
	case 1:
		LEDSD_1();
		break;
	case 2:
		LEDSD_2();
		break;
	case 3:
		LEDSD_3();
		break;
	case 4:
		LEDSD_4();
		break;
	case 5:
		LEDSD_5();
		break;
	case 6:
		LEDSD_6();
		break;
	case 7:
		LEDSD_7();
		break;
	case 8:
		LEDSD_8();
		break;
	case 9:
		LEDSD_9();
		break;
	default:
		LEDSD_ERROR();
		break;
	}
}

/**
 * @brief		Light number 0 on LED segment display.
 * @param		None
 * @retval		None
 */
void LEDSD_0(void){
	// The number 0.
	GPIO_SetBits(GPIOB, LED_CD |
						LED_LD |
						LED_LU |
						LED_RD);
	GPIO_SetBits(GPIOA, LED_CU | LED_RU);
}

/**
 * @brief	Light number 1 on LED segment display.
 * @param	None
 * @retval	None
 */
void LEDSD_1(void){
	// The number 1.
	GPIO_SetBits(GPIOB, LED_RD);
	GPIO_SetBits(GPIOA, LED_RU);
}

/**
 * @brief	Light number 2 on LED segment display.
 * @param	None
 * @retval	None
 */
void LEDSD_2(void){
	// The number 2.
	GPIO_SetBits(GPIOB, LED_CD |
						LED_LD |
						LED_CC );
	GPIO_SetBits(GPIOA, LED_CU | LED_RU);
}

/**
 * @brief	Light number 3 on LED segment display.
 * @param	None
 * @retval	None
 */
void LEDSD_3(void){
	// The number 3.
	GPIO_SetBits(GPIOB, LED_CD |
						LED_CC |
						LED_RD);
	GPIO_SetBits(GPIOA, LED_CU | LED_RU);
}

/**
 * @brief	Light number 4 on LED segment display.
 * @param	None
 * @retval	None
 */
void LEDSD_4(void){
	// The number 4.
	GPIO_SetBits(GPIOB, LED_LU |
						LED_CC |
						LED_RD);
	GPIO_SetBits(GPIOA, LED_RU);
}

/**
 * @brief	Light number 5 on LED segment display.
 * @param	None
 * @retval	None
 */
void LEDSD_5(void){
	// The number 5.
	GPIO_SetBits(GPIOB, LED_CD |
						LED_LU |
						LED_CC |
						LED_RD);
	GPIO_SetBits(GPIOA, LED_CU);
}

/**
 * @brief	Light number 6 on LED segment display.
 * @param	None
 * @retval	None
 */
void LEDSD_6(void){
	// The number 6.
	GPIO_SetBits(GPIOB, LED_CD |
						LED_LD |
						LED_LU |
						LED_CC |
						LED_RD);
	GPIO_SetBits(GPIOA, LED_CU);
}

/**
 * @brief	Light number 7 on LED segment display.
 * @param	None
 * @retval	None
 */
void LEDSD_7(void){
	// The number 7.
	GPIO_SetBits(GPIOB, LED_RD);
	GPIO_SetBits(GPIOA, LED_CU | LED_RU);
}

/**
 * @brief	Light number 8 on LED segment display.
 * @param	None
 * @retval	None
 */
void LEDSD_8(void){
	// The number 8.
	GPIO_SetBits(GPIOB, LED_CD |
						LED_LD |
						LED_LU |
						LED_CC |
						LED_RD);
	GPIO_SetBits(GPIOA, LED_CU | LED_RU);
}

/**
 * @brief	Light number 9 on LED segment display.
 * @param	None
 * @retval	None
 */
void LEDSD_9(void){
	// The number 9.
	GPIO_SetBits(GPIOB, LED_CD |
						LED_LU |
						LED_CC |
						LED_RD);
	GPIO_SetBits(GPIOA, LED_CU | LED_RU);
}

/**
 * @brief	Light "-" on LED segment display.
 * @param	None
 * @retval	None
 */
void LEDSD_ERROR(void){
	// The symbol "-".
	GPIO_SetBits(GPIOB,LED_CC);
}

/**
 * @brief	Clear all the display numbers.
 * @param	None
 * @retval	None
 */
void LEDSD_CLEAR(void){
	GPIO_ResetBits(GPIOB, LED_CC | LED_CD | LED_LU | LED_LD | LED_RD | LED_POINT);
	GPIO_ResetBits(GPIOA, LED_CU | LED_RU);
}


/* Motor */
extern int count;
void delay_ms(uint16_t);

/**
 * @brief		This function delays the program, in millisecond.
 * @param 		MS: The number of millisecond.
 * @retval		None
 */
void delay_ms_SB(uint16_t  MS){
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

extern int k_up, k_dp;
extern float k;
/**
 * @brief	Calulate and get k unit place value.
 * @param	None
 * @retval	The unit place value of k.
 */
int Getk_up(void){
	k_up = k / 1;
	return k_up;
}

/**
 * @brief	Calulate and get k decimal place value.
 * @param	None
 * @retval	The decimal place value of k.
 */
int Getk_dp(void){
	k_dp = (int)(k * 10) % 10;
	return k_dp;
}

/**
 * @brief	Using motor to open the curtain when sysyem at automactic mode, the number of impluse is tem.
 * @param	None
 * @retval	None
 */
extern uint16_t motorDelayTime;
void MotorOpen(int tem){
	// int i = 0;
	// GPIO_SetBits(GPIOB, GPIO_Pin_14);
	for(; tem > 0; tem--){
		count += 4;

		GPIO_ResetBits(GPIOB,GPIO_Pin_14);
		GPIO_SetBits(GPIOA,GPIO_Pin_0);
	//	GPIO_ResetBits(GPIOA,GPIO_Pin_0);
		delay_ms_SB(1);
		GPIO_ResetBits(GPIOB,GPIO_Pin_14);
		GPIO_SetBits(GPIOA,GPIO_Pin_0);
	//	GPIO_SetBits(GPIOA,GPIO_Pin_0);
		delay_ms_SB(3);
			LEDSD_CLEAR();
			LEDSD_UP();
			LEDSD_X(k_up);
		GPIO_ResetBits(GPIOB,GPIO_Pin_15);
		GPIO_SetBits(GPIOA,GPIO_Pin_3);
	//	GPIO_ResetBits(GPIOA,GPIO_Pin_0);
		delay_ms_SB(1);
		GPIO_ResetBits(GPIOB,GPIO_Pin_15);
		GPIO_SetBits(GPIOA,GPIO_Pin_3);
	//	GPIO_SetBits(GPIOA,GPIO_Pin_0);
		delay_ms_SB(3);
			LEDSD_CLEAR();
			LEDSD_DP();
			LEDSD_X(k_dp);
		GPIO_SetBits(GPIOB,GPIO_Pin_14);
		GPIO_ResetBits(GPIOA,GPIO_Pin_0);
	//	GPIO_ResetBits(GPIOA,GPIO_Pin_0);
		delay_ms_SB(1);
		GPIO_SetBits(GPIOB,GPIO_Pin_14);
		GPIO_ResetBits(GPIOA,GPIO_Pin_0);
		//GPIO_SetBits(GPIOA,GPIO_Pin_0);
		delay_ms_SB(3);
			LEDSD_CLEAR();
			LEDSD_UP();
			LEDSD_X(k_up);
		GPIO_SetBits(GPIOB,GPIO_Pin_15);
		GPIO_ResetBits(GPIOA,GPIO_Pin_3);
	//	GPIO_ResetBits(GPIOA,GPIO_Pin_0);
		delay_ms_SB(1);
		GPIO_SetBits(GPIOB,GPIO_Pin_15);
		GPIO_ResetBits(GPIOA,GPIO_Pin_3);
	//	GPIO_SetBits(GPIOA,GPIO_Pin_0);
		delay_ms_SB(3);
			LEDSD_CLEAR();
			LEDSD_DP();
			LEDSD_X(k_dp);


		// GPIO_SetBits(GPIOB, GPIO_Pin_15);
		// if(i % 2 == 0){// LEDSD Flash frequency: 25Hz
		// 	LEDSD_CLEAR();
		// 	LEDSD_UP();
		// 	LEDSD_X(k_up);
		// }
		// delay_ms_SB(motorDelayTime);
		// GPIO_ResetBits(GPIOB, GPIO_Pin_15);
		// if(i % 2 == 0){
		// 	LEDSD_CLEAR();
		// 	LEDSD_DP();
		// 	LEDSD_X(k_dp);
		// }
		// delay_ms_SB(motorDelayTime);
		// i++;
	}// for
}

/**
 * @brief	Using motor to close the curtain when system at automatic mode, the number of impluse is tem.
 * @param	None
 * @retval	None
 */
void MotorClose(int tem){
	// int i = 0;
	// GPIO_ResetBits(GPIOB, GPIO_Pin_14);
	for(; tem > 0; tem--){
		count -= 4;

		GPIO_ResetBits(GPIOB,GPIO_Pin_14);
		GPIO_SetBits(GPIOA,GPIO_Pin_0);
		//GPIO_ResetBits(GPIOA,GPIO_Pin_0);
		delay_ms_SB(1);
		GPIO_ResetBits(GPIOB,GPIO_Pin_14);
		GPIO_SetBits(GPIOA,GPIO_Pin_0);
		//GPIO_SetBits(GPIOA,GPIO_Pin_0);
		delay_ms_SB(3);
			LEDSD_CLEAR();
			LEDSD_UP();
			LEDSD_X(k_up);
	GPIO_SetBits(GPIOB,GPIO_Pin_15);
		GPIO_ResetBits(GPIOA,GPIO_Pin_3);
		//GPIO_ResetBits(GPIOA,GPIO_Pin_0);
		delay_ms_SB(1);
		GPIO_SetBits(GPIOB,GPIO_Pin_15);
		GPIO_ResetBits(GPIOA,GPIO_Pin_3);
	//	GPIO_SetBits(GPIOA,GPIO_Pin_0);
		delay_ms_SB(3);
			LEDSD_CLEAR();
			LEDSD_DP();
			LEDSD_X(k_dp);
		GPIO_SetBits(GPIOB,GPIO_Pin_14);
		GPIO_ResetBits(GPIOA,GPIO_Pin_0);
	//	GPIO_ResetBits(GPIOA,GPIO_Pin_0);
		delay_ms_SB(1);
		GPIO_SetBits(GPIOB,GPIO_Pin_14);
		GPIO_ResetBits(GPIOA,GPIO_Pin_0);
	//	GPIO_SetBits(GPIOA,GPIO_Pin_0);
		delay_ms_SB(3);
			LEDSD_CLEAR();
			LEDSD_UP();
			LEDSD_X(k_up);
		
		GPIO_ResetBits(GPIOB,GPIO_Pin_15);
		GPIO_SetBits(GPIOA,GPIO_Pin_3);
		//GPIO_ResetBits(GPIOA,GPIO_Pin_0);
		delay_ms_SB(1);
		GPIO_ResetBits(GPIOB,GPIO_Pin_15);
		GPIO_SetBits(GPIOA,GPIO_Pin_3);
		//GPIO_SetBits(GPIOA,GPIO_Pin_0);
		delay_ms_SB(3);
			LEDSD_CLEAR();
			LEDSD_DP();
			LEDSD_X(k_dp);



		// GPIO_SetBits(GPIOB, GPIO_Pin_15);
		// if(i % 2 == 0){// LEDSD Flash frequency: 25Hz
		// 	LEDSD_CLEAR();
		// 	LEDSD_UP();
		// 	LEDSD_X(k_up);
		// }
		// delay_ms_SB(motorDelayTime);
		// GPIO_ResetBits(GPIOB, GPIO_Pin_15);
		// if(i % 2 == 0){
		// 	LEDSD_CLEAR();
		// 	LEDSD_DP();
		// 	LEDSD_X(k_dp);
		// }
		// delay_ms_SB(motorDelayTime);
		// i++;
	}
}

/* ------------ Bluetooth ------------ */
/**
 * @brief	Configure USART line 1.
 * @param	None
 * @retval	None
 */
void USART1_Config(void){
	// RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
}

extern u8 btMessage[10];
extern int k_up, k_dp;
extern float k;
extern int charCount;
extern char ch;
// int test = 0;
extern int stopMotor;
extern int auto_manual;
extern int isadjust;

void BT_Update(void){
	if(ch == 's'){// Stop motor.
		// test = 0;
	}
	if(ch == 'c'){// Close the curtain.
		if(stopMotor != 1){// Curtain is not fully closed.
			while(1){
				if(stopMotor == 2){
					stopMotor = 0;
				}
				// Close curtain motor.
			
			GPIO_ResetBits(GPIOB,GPIO_Pin_14);
		GPIO_SetBits(GPIOA,GPIO_Pin_0);
		//GPIO_ResetBits(GPIOA,GPIO_Pin_0);
		delay_ms_SB(1);
		GPIO_ResetBits(GPIOB,GPIO_Pin_14);
		GPIO_SetBits(GPIOA,GPIO_Pin_0);
		//GPIO_SetBits(GPIOA,GPIO_Pin_0);
		delay_ms_SB(3);
			LEDSD_CLEAR();
			LEDSD_UP();
			LEDSD_X(k_up);
	GPIO_SetBits(GPIOB,GPIO_Pin_15);
		GPIO_ResetBits(GPIOA,GPIO_Pin_3);
		//GPIO_ResetBits(GPIOA,GPIO_Pin_0);
		delay_ms_SB(1);
		GPIO_SetBits(GPIOB,GPIO_Pin_15);
		GPIO_ResetBits(GPIOA,GPIO_Pin_3);
	//	GPIO_SetBits(GPIOA,GPIO_Pin_0);
		delay_ms_SB(3);
			LEDSD_CLEAR();
			LEDSD_DP();
			LEDSD_X(k_dp);
		GPIO_SetBits(GPIOB,GPIO_Pin_14);
		GPIO_ResetBits(GPIOA,GPIO_Pin_0);
	//	GPIO_ResetBits(GPIOA,GPIO_Pin_0);
		delay_ms_SB(1);
		GPIO_SetBits(GPIOB,GPIO_Pin_14);
		GPIO_ResetBits(GPIOA,GPIO_Pin_0);
	//	GPIO_SetBits(GPIOA,GPIO_Pin_0);
		delay_ms_SB(3);
			LEDSD_CLEAR();
			LEDSD_UP();
			LEDSD_X(k_up);
		
		GPIO_ResetBits(GPIOB,GPIO_Pin_15);
		GPIO_SetBits(GPIOA,GPIO_Pin_3);
		//GPIO_ResetBits(GPIOA,GPIO_Pin_0);
		delay_ms_SB(1);
		GPIO_ResetBits(GPIOB,GPIO_Pin_15);
		GPIO_SetBits(GPIOA,GPIO_Pin_3);
		//GPIO_SetBits(GPIOA,GPIO_Pin_0);
		delay_ms_SB(3);
			LEDSD_CLEAR();
			LEDSD_DP();
			LEDSD_X(k_dp);
				// test++;
				if(ch == 's'){
					break;
				}
				if(stopMotor == 1){
					break;
				}
				count -= 4;
			}
		}
		else{// Curtain is fully closed || Curtain is closed & open.
			if(count != 0){
				stopMotor = 0;// Motor move.
			}
		}
	}

	if(ch == 'o'){// Open curtain.
		if(stopMotor != 2){// Curtain is not fully open.
			while(1){
				if(stopMotor == 1){
					stopMotor = 0;
				}
				// Open curtain motor.
			
		GPIO_ResetBits(GPIOB,GPIO_Pin_14);
		GPIO_SetBits(GPIOA,GPIO_Pin_0);
	//	GPIO_ResetBits(GPIOA,GPIO_Pin_0);
		delay_ms_SB(1);
		GPIO_ResetBits(GPIOB,GPIO_Pin_14);
		GPIO_SetBits(GPIOA,GPIO_Pin_0);
	//	GPIO_SetBits(GPIOA,GPIO_Pin_0);
		delay_ms_SB(3);
			LEDSD_CLEAR();
			LEDSD_UP();
			LEDSD_X(k_up);
		GPIO_ResetBits(GPIOB,GPIO_Pin_15);
		GPIO_SetBits(GPIOA,GPIO_Pin_3);
	//	GPIO_ResetBits(GPIOA,GPIO_Pin_0);
		delay_ms_SB(1);
		GPIO_ResetBits(GPIOB,GPIO_Pin_15);
		GPIO_SetBits(GPIOA,GPIO_Pin_3);
	//	GPIO_SetBits(GPIOA,GPIO_Pin_0);
		delay_ms_SB(3);
			LEDSD_CLEAR();
			LEDSD_DP();
			LEDSD_X(k_dp);
		GPIO_SetBits(GPIOB,GPIO_Pin_14);
		GPIO_ResetBits(GPIOA,GPIO_Pin_0);
	//	GPIO_ResetBits(GPIOA,GPIO_Pin_0);
		delay_ms_SB(1);
		GPIO_SetBits(GPIOB,GPIO_Pin_14);
		GPIO_ResetBits(GPIOA,GPIO_Pin_0);
		//GPIO_SetBits(GPIOA,GPIO_Pin_0);
		delay_ms_SB(3);
			LEDSD_CLEAR();
			LEDSD_UP();
			LEDSD_X(k_up);
		GPIO_SetBits(GPIOB,GPIO_Pin_15);
		GPIO_ResetBits(GPIOA,GPIO_Pin_3);
	//	GPIO_ResetBits(GPIOA,GPIO_Pin_0);
		delay_ms_SB(1);
		GPIO_SetBits(GPIOB,GPIO_Pin_15);
		GPIO_ResetBits(GPIOA,GPIO_Pin_3);
	//	GPIO_SetBits(GPIOA,GPIO_Pin_0);
		delay_ms_SB(3);
			LEDSD_CLEAR();
			LEDSD_DP();
			LEDSD_X(k_dp);


				if(ch == 's'){
					break;
				}
				if(stopMotor == 2){
					break;
				}
				count += 4;
			}
		}
		else{// Curtain is fully open || curtain has been fully open then partially closed.
			if(count != 21000){
				stopMotor = 0;// Motor move.
			}
		}
	}
	// if(ch == 'a'){// Change to automatic mode.
	// 	auto_manual = 0;// Change manual mode to automatic mode.
	// 	stopMotor = 0;// Motor move.
	// 	isadjust = 1;// Need to adjust the current impluse after toggling mode.
	// }
	// if(ch == 'm'){// Change to maunal mode.
	// 	auto_manual = 1;// Change automatic mode to manual mode.
	// }
	// if(ch == 'k'){// Set the scale factor k.
	// 	charCount = 0;
	// 	btMessage[charCount] = ch;
	// 	charCount++;
	// 	if(charCount == 10){
	// 		charCount = 0;
	// 	}
	// }
	// if(ch >= '0' && ch <= '9'){// Set the scale factor k.
	// 	if(charCount >=1 && charCount <= 3){
	// 		btMessage[charCount] = ch;
	// 		charCount++;
	// 	}
	// 	else{
	// 		if(charCount >= 4 && charCount <= 6){
	// 			btMessage[charCount] = ch;
	// 			charCount++;
	// 			// Update scale factor k.
	// 			k_up = btMessage[1] - '0';
	// 			k_dp = btMessage[4] - '0';
	// 			k = k_up + 0.1 * k_dp;
	// 		}
	// 	}
	// }
	delay_ms_SB(800);
}
