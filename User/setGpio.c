#include "stm32f10x.h"
#include "stm32_eval.h"
#include <stdio.h>

// set Port A bit 15 as input with pull-up
void setPA15_IPU ( void ) {
      GPIO_InitTypeDef GPIO_InitStructure;
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15; // bit to be set
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // Input with pull-up resistor
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
      GPIO_Init(GPIOA, &GPIO_InitStructure);
} 

