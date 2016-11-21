#include "stm32f10x.h"
#include "math.h"
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
 * @brief		Set PA7 pin at input pull-up mode.
 * @param 	None
 * @retval	None
 */
void setPA7_IPU(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
 * @brief		Set PA6 pin at input pull-up mode.
 * @param 	None
 * @retval	None
 */
void setPA6_IPU(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
 * @brief
 * @param
 * @retval
 */
void setPA5_IPU(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
 * @brief		Set PB.00~07 pins at ouput push-push mode, 2MHz speed.
 * @param		None
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
 * @brief		Set PB.10 and PB.11 pins at ouput push-push mode, 2MHz speed.
 * @param		None
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
 * @brief		Set PB.10 and PB.11 pins at ouput push-push mode, 2MHz speed.
 * @param		None
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
 * @brief		Delay the program.
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
 * @brief		Initialize the ADC1 & ADC2.
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
 * @brief		Get ADC1 value.
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
 * @brief		Get ADC1 average value.
 * @param 	ADC_Channel_x
 * @param		times
 * @retval	ADC1 average value.
 */
u16 GetAdc1Average(u8 ch,u8 times){
	// ADC_Cmd(ADC1, ENABLE);
	void delay_ms(uint16_t  MS);
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=GetAdc1(ch);
		//Delay(5);
		//delay_ms(5);
	}
	// ADC_Cmd(ADC1, DISABLE);
	return temp_val/times;
}

/**
 * @brief		Get ADC1 average value.
 * @param 	ADC_Channel_x
 * @param		times
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
 * @brief		Get average lux value from analogue input pin PA1.
 * @param 	Aver_Analogue_In: The average analogue input value.
 * @retval	The average value of lux.
 */
float CalAverageLux(u16 Aver_Analogue_In){
	float avervolts, averamps, avermicroamps, averlux;
	avervolts = Aver_Analogue_In * 3.3 / 4096.0;
	averamps = avervolts / 10000.0;  // across 10,000 Ohms
	avermicroamps = averamps * 1000000;
	averlux = avermicroamps * 2.0;
	return averlux;
}

/**
 * @brief		Get lux value from analogue input pin PA1.
 * @param 	Analogue_In: The analogue input value.
 * @retval	The average value of lux.
 */
float CalLux(u16 Analogue_In){
	float volts, amps, microamps, lux;
	volts = Analogue_In * 3.3 / 4096.0;
	amps = volts / 10000.0;  // across 10,000 Ohms
	microamps = amps * 1000000;
	lux = microamps * 2.0;
	return lux;
}

/**
 * @brief		Get length from analogue input pin PA2.
 * @param 	Analogue_In: The analogue input value.
 * @retval	The average value of lux.
 */
float CalLength(u16 Analogue_In){
	float volts, length;
	volts = Analogue_In * 3.3 / 4096.0;
	length = -5.648*pow(volts,3)+35.42*pow(volts,2)-82.44*volts+80.52;
	return length;
}

/**
 * @brief		Get average lux value from analogue input pin PA1.
 * @param 	Aver_Analogue_In: The average analogue input value.
 * @retval	The average value of lux.
 */
float CalAverageLength(u16 Aver_Analogue_In){
	float avervolts, averlength;
	avervolts = Aver_Analogue_In * 3.3 / 4096.0;
	averlength = -5.648*pow(avervolts,3)+35.42*pow(avervolts,2)-82.44*avervolts+80.52;
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
 * @brief		Initialize SysTick, and unit is milllisecond.
 * @param		None
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
 * @brief		Decrements the TimingDelay variable.
 * @param		None
 * @retval	None
 */
void TimingDelay_Decrement(void){
	if(TimingDelay != 0x00){
		TimingDelay--;
	}
}

/**
 * @brief		Initialize the EXTI(External Interrupt).
 * @param		None
 * @retval	None
 */
void EXTIInit(void){
	void NVICInit(void);
	EXTI_InitTypeDef EXTI_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);// Enable the AFIO clock
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource7);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource6);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource5);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource4);
	// Configure EXTI line 5, 6, 7
	EXTI_InitStructure.EXTI_Line = EXTI_Line4 | EXTI_Line5 | EXTI_Line6 | EXTI_Line7;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVICInit();
}

/**
 * @brief		Initialize the NVIC(Nested Vector Interrupt Controller).
 * @param		None
 * @retval	None
 */
void NVICInit(void){
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn | EXTI4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;// The highest pre-emption priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;// The highest sub priority
	NVIC_Init(&NVIC_InitStructure);
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
 * @retval	None
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
 * @brief		Light number 1 on LED segment display.
 * @param		None
 * @retval	None
 */
void LEDSD_1(void){
	// The number 1.
	GPIO_SetBits(GPIOB, LED_RD);
	GPIO_SetBits(GPIOA, LED_RU);
}

/**
 * @brief		Light number 2 on LED segment display.
 * @param		None
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
 * @brief		Light number 3 on LED segment display.
 * @param		None
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
 * @brief		Light number 4 on LED segment display.
 * @param		None
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
 * @brief		Light number 5 on LED segment display.
 * @param		None
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
 * @brief		Light number 6 on LED segment display.
 * @param		None
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
 * @brief		Light number 7 on LED segment display.
 * @param		None
 * @retval	None
 */
void LEDSD_7(void){
	// The number 7.
	GPIO_SetBits(GPIOB, LED_RD);
	GPIO_SetBits(GPIOA, LED_CU | LED_RU);
}

/**
 * @brief		Light number 8 on LED segment display.
 * @param		None
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
 * @brief		Light number 9 on LED segment display.
 * @param		None
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
 * @brief		Light "-" on LED segment display.
 * @param		None
 * @retval	None
 */
void LEDSD_ERROR(void){
	// The symbol "-".
	GPIO_SetBits(GPIOB,LED_CC);
}

/**
 * @brief		Clear all the display numbers.
 * @param		None
 * @retval		None
 */
void LEDSD_CLEAR(void){
	GPIO_ResetBits(GPIOB, LED_CC | LED_CD | LED_LU | LED_LD | LED_RD | LED_POINT);
	GPIO_ResetBits(GPIOA, LED_CU | LED_RU);
}

/* Infrared Ranging Sensor*/



/* Motor */
void PWMInit(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
  TIM_OCInitTypeDef TIM_OCInitStruct;
  GPIO_InitTypeDef GPIO_InitStruct;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);//?????2???
  //GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE);

  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;//??????
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;//PA0
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA,&GPIO_InitStruct);

  //TIM2???????
  TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;//??????
  TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;//??????
  TIM_TimeBaseInitStruct.TIM_Prescaler = 0;//????,?100us????
  TIM_TimeBaseInitStruct.TIM_Period = 10000;//???
  TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStruct);

  TIM_SelectOnePulseMode(TIM2,TIM_OPMode_Single);//??TIM2??????,???????,??????????
  TIM_OC1PreloadConfig(TIM2,TIM_OCPreload_Enable);//?????2???1??????
  TIM_SelectOutputTrigger(TIM2,TIM_TRGOSource_OC1Ref);

  TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;//??????,??TIMx_CNT<TIMx_CCR1???1?????,???????
  TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;//OC1????
  TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;//??????
  TIM_OCInitStruct.TIM_Pulse = 990;//????1?????
  TIM_OC1Init(TIM2,&TIM_OCInitStruct);
  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM2, ENABLE);

  TIM_Cmd(TIM2,ENABLE);//?????TIM2
}
/*void Motor_Init(u16 TIM2per, u16 TIM3per, u16 TIM3Compare1)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
  TIM_OCInitTypeDef TIM_OCInitStruct;
  GPIO_InitTypeDef GPIO_InitStruct;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);//?????2???
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);//?????3???
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//??GPIOA??
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//????IO??


GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;//??????
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_6;//PA0
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA,&GPIO_InitStruct);

  //TIM2???????
  TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;//??????
  TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;//??????
  TIM_TimeBaseInitStruct.TIM_Prescaler = 7200;//????,?100us????
  TIM_TimeBaseInitStruct.TIM_Period = TIM2per;//???
  TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStruct);

  TIM_SelectOnePulseMode(TIM2,TIM_OPMode_Single);//??TIM2??????,???????,??????????
  TIM_OC1PreloadConfig(TIM2,TIM_OCPreload_Enable);//?????2???1??????
  TIM_SelectOutputTrigger(TIM2,TIM_TRGOSource_OC1Ref);

  TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;//??????,??TIMx_CNT<TIMx_CCR1???1?????,???????
  TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;//OC1????
  TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;//??????
  TIM_OCInitStruct.TIM_Pulse = 1;//????1?????
  TIM_OC1Init(TIM2,&TIM_OCInitStruct);

  TIM_Cmd(TIM2,DISABLE);//?????TIM2


  //TIM3?????????????PWM????
  TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;//??????
  TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;//??????
  TIM_TimeBaseInitStruct.TIM_Prescaler = 720;//????,10us????
  TIM_TimeBaseInitStruct.TIM_Period = TIM3per;//???
  TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStruct);

  TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Gated);//TIM3?????
  TIM_SelectMasterSlaveMode(TIM3,TIM_MasterSlaveMode_Enable);//??TIM3?????
  TIM_SelectInputTrigger(TIM3,TIM_TS_ITR1);//????,?TIM2??

  TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;//??????,??TIMx_CNT<TIMx_CCR1???1?????,???????
  TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;//OC1????
  TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;//??????
  TIM_OCInitStruct.TIM_Pulse = TIM3Compare1;//????1?????
  TIM_OC1Init(TIM3,&TIM_OCInitStruct);

  TIM_Cmd(TIM3,ENABLE);//??TIM3
}

//??PWM???
//Cycle:???,??(us)
//Pulse_Num:?????(??3200)
void TIM2_TIM3_PWM(u16 Cycle, u16 Pulse_Num)
{
  u16 TIM3per = 0;
  u32 Time = 0;
  //??TIM3????????????????????50%
  //??TIM2???????????????

  Time = Cycle * Pulse_Num;
  Time /= 100;              //????7200,100us????
  TIM3per = Cycle/10;       //????720,10us????

  TIM_SetAutoreload(TIM2, Time+1);//??TIM2????
  TIM_SetAutoreload(TIM3, TIM3per-1);//??TIM3????
  TIM_SetCompare1(TIM3,TIM3per/2);//??????50%
  TIM_Cmd(TIM2,ENABLE);//??TIM2
}*/
