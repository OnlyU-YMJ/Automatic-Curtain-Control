#include "stm32f10x.h"

/**
 * @brief 	Set PA1 pin at analogue input mode, 2MHz speed.
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
 * @brief		Initialize the ADC.
 * @param 	None
 * @retval	None
 */
void  AdcInit(void)
{
	ADC_InitTypeDef ADC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE );	  //使锟斤拷ADC1通锟斤拷时锟斤拷
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //锟斤拷锟斤拷ADC锟斤拷频锟斤拷锟斤拷6 72M/6=12,ADC锟斤拷锟斤拷时锟戒不锟杰筹拷锟斤拷14M

	//PA7 锟斤拷为模锟斤拷通锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
	setPA1_AIN();

	ADC_DeInit(ADC1);  //锟斤拷位ADC1,锟斤拷锟斤拷锟斤拷 ADC1 锟斤拷全锟斤拷锟侥达拷锟斤拷锟斤拷锟斤拷为缺省值

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC锟斤拷锟斤拷模式:ADC1锟斤拷ADC2锟斤拷锟斤拷锟节讹拷锟斤拷模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//模锟斤拷转锟斤拷锟斤拷锟斤拷锟节碉拷通锟斤拷模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//模锟斤拷转锟斤拷锟斤拷锟斤拷锟节碉拷锟斤拷转锟斤拷模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟解部锟斤拷锟斤拷锟斤拷锟斤拷
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC锟斤拷锟斤拷锟揭讹拷锟斤拷
	ADC_InitStructure.ADC_NbrOfChannel = 1;	//顺锟斤拷锟斤拷锟叫癸拷锟斤拷转锟斤拷锟斤拷ADC通锟斤拷锟斤拷锟斤拷目
	ADC_Init(ADC1, &ADC_InitStructure);	//锟斤拷锟斤拷ADC_InitStruct锟斤拷指锟斤拷锟侥诧拷锟斤拷锟斤拷始锟斤拷锟斤拷锟斤拷ADCx锟侥寄达拷锟斤拷

	ADC_Cmd(ADC1, ENABLE);	//使锟斤拷指锟斤拷锟斤拷ADC1

	ADC_ResetCalibration(ADC1);	//使锟杰革拷位校准

	while(ADC_GetResetCalibrationStatus(ADC1));	//锟饺达拷锟斤拷位校准锟斤拷锟斤拷

	ADC_StartCalibration(ADC1);	 //锟斤拷锟斤拷AD校准

	while(ADC_GetCalibrationStatus(ADC1));	 //锟饺达拷校准锟斤拷锟斤拷

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使锟斤拷指锟斤拷锟斤拷ADC1锟斤拷锟斤拷锟斤拷转锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷

}


/**
 * @brief		Get ADC value.
 * @param 	ch: Choose the ADC channel.
 * @retval	ADC conversion value.
 */
u16 GetAdc(u8 ch)
{
  	//锟斤拷锟斤拷指锟斤拷ADC锟侥癸拷锟斤拷锟斤拷通锟斤拷锟斤拷一锟斤拷锟斤拷锟叫ｏ拷锟斤拷锟斤拷时锟斤拷
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADC通锟斤拷,锟斤拷锟斤拷时锟斤拷为239.5锟斤拷锟斤拷

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使锟斤拷指锟斤拷锟斤拷ADC1锟斤拷锟斤拷锟斤拷转锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷

	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//锟饺达拷转锟斤拷锟斤拷锟斤拷

	return ADC_GetConversionValue(ADC1);	//锟斤拷锟斤拷锟斤拷锟斤拷一锟斤拷ADC1锟斤拷锟斤拷锟斤拷锟斤拷转锟斤拷锟斤拷锟斤拷
}

/**
 * @brief		Get ADC average value.
 * @param 	ADC_Channel_x
 * @param		times
 * @retval	ADC average value.
 */
u16 GetAdcAverage(u8 ch,u8 times)
{
	void delay_ms(uint16_t  MS);
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=GetAdc(ch);
		//Delay(5);
		//delay_ms(5);
	}
	return temp_val/times;
}

/**
 * @brief		Get average lux value from analogue input pin PA0.
 * @param 	Aver_Analogue_In: The average analogue input value.
 * @retval	The average value of lux.
 */
float CalAverageLux(u16 Aver_Analogue_In){
	float avervolts, averamps, avermicroamps, averlux;
	avervolts = Aver_Analogue_In * 5.0 / 1024.0;
	averamps = avervolts / 10000.0;  // across 10,000 Ohms
	avermicroamps = averamps * 1000000;
	averlux = avermicroamps * 2.0;
	return averlux;
}

/**
 * @brief		Get lux value from analogue input pin PA0.
 * @param 	Analogue_In: The analogue input value.
 * @retval	The average value of lux.
 */
float CalLux(u16 Analogue_In){
	float volts, amps, microamps, lux;
	volts = Analogue_In * 5.0 / 1024.0;
	amps = volts / 10000.0;  // across 10,000 Ohms
	microamps = amps * 1000000;
	lux = microamps * 2.0;
	return lux;
}

/**
 * @brief		Check the key's status.[!Low voltage is valid!!!!]
 * @param		GPIOx: GPIO fields(A~G)
 * @param		GPIO_Pin: GPIO pin number(0~15)
 * @retval		Return Bit_RESET is ON, Bit_SET is OFF.
 */
uint8_t KeyScan(GPIO_TypeDef* GPIOx, u16 GPIO_Pin){
	if(GPIO_ReadInputDataBit(GPIOx, GPIO_Pin) == Bit_RESET){
		Delay(50);// Delay to clear the jitter
		if(GPIO_ReadInputDataBit(GPIOx, GPIO_Pin) == Bit_RESET){
			while(GPIO_ReadInputDataBit(GPIOx,GPIO_Pin) == Bit_RESET);
			Delay(5);
			return Bit_RESET;
		}
		else{
			return Bit_SET;
		}
	}
	else{
		return Bit_SET;
	}
}

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

	// Configure EXTI line 5, 6, 7
	EXTI_InitStructure.EXTI_Line = EXTI_Line5 | EXTI_Line6 | EXTI_Line7;
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
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
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
