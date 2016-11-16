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
 * @brief		Delay the program.
 * @param 	nTime: Specifies the delay time length, in milliseconds.
 * @retval	None
 */
static __IO uint32_t TimingDelay;
void Delay (__IO uint32_t nTime)
{
  TimingDelay = nTime;
	while(TimingDelay != 0);
}

/**
 * @brief		Initialize the ADC.
 * @param 	None
 * @retval	None
 */
void  AdcInit(void)
{ 	
	ADC_InitTypeDef ADC_InitStructure; 
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE );	  //ʹ��ADC1ͨ��ʱ��
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M

	//PA7 ��Ϊģ��ͨ����������                         
	setPA1_AIN();

	ADC_DeInit(ADC1);  //��λADC1,������ ADC1 ��ȫ���Ĵ�������Ϊȱʡֵ

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC����ģʽ:ADC1��ADC2�����ڶ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//ģ��ת�������ڵ�ͨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//ģ��ת�������ڵ���ת��ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//ת��������������ⲿ��������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 1;	//˳����й���ת����ADCͨ������Ŀ
	ADC_Init(ADC1, &ADC_InitStructure);	//����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ���   

	ADC_Cmd(ADC1, ENABLE);	//ʹ��ָ����ADC1
	
	ADC_ResetCalibration(ADC1);	//ʹ�ܸ�λУ׼  
	 
	while(ADC_GetResetCalibrationStatus(ADC1));	//�ȴ���λУ׼����
	
	ADC_StartCalibration(ADC1);	 //����ADУ׼
 
	while(ADC_GetCalibrationStatus(ADC1));	 //�ȴ�У׼����
 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//ʹ��ָ����ADC1�����ת����������

}				  


/**
 * @brief		Get ADC value.
 * @param 	ch: Choose the ADC channel.
 * @retval	ADC conversion value.
 */
u16 GetAdc(u8 ch)   
{
  	//����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADCͨ��,����ʱ��Ϊ239.5����	  			    
  
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//ʹ��ָ����ADC1�����ת����������	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//�ȴ�ת������

	return ADC_GetConversionValue(ADC1);	//�������һ��ADC1�������ת�����
}

/**
 * @brief		Get ADC average value.
 * @param 	ADC_Channel_x
 * @param		times
 * @retval	ADC average value.
 */
u16 GetAdcAverage(u8 ch,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=GetAdc(ch);
		Delay(5);
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
 * @retval	Return Bit_RESET is ON, Bit_SET is OFF.
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
