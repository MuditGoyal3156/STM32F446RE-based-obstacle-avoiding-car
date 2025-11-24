/*
 * int.c
 *
 *  Created on: Nov 4, 2025
 *      Author: mudit
 */


#include <stdint.h>
#include "int.h"
#include "stm32f4xx.h"


void GPIO(void)
{
	 /*
	  * Clock enable for GPIOC
	  */
	RCC->AHB1ENR |= ( 1U << 2 );

	//Mode selection

	GPIOC->MODER &= ~( 3U << 26 ); //PC13 INPUT


	// Pull-up

	GPIOC->PUPDR |= ( 1U << 26 ); //PC13 Pull-up

}

void INT(void)
{
	//SYSCFG Clk EN

	RCC->APB2ENR |= ( 1U << 14 );


	//falling edge trigger

	EXTI->FTSR |= ( 1U << 13);


	//Port Code in SYSCFG_EXTICRx registers


	SYSCFG->EXTICR[3] &= ~(0xF << 4);
	SYSCFG->EXTICR[3] |= (0x2 << 4);   // 0010 = Port C


	 //IMR enable

	EXTI->IMR |= ( 1U << 13);


	 // interrupt set enable

	NVIC->ISER[1] |=( 1U << 8);


}


