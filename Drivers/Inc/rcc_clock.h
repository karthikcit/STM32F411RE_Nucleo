/*
 * rcc_clock.h
 *
 *  Created on: 18-Apr-2020
 *      Author: KarthikM
 */

#ifndef INC_RCC_CLOCK_H_
#define INC_RCC_CLOCK_H_

#include "stm32f411xx.h"


RTC_RegDef_t *pRCC = RCC;

#define GPIOA_PCLK_EN()	(pRCC->RCC_AHB1ENR |= (SET<<0))
#define GPIOB_PCLK_EN()	(pRCC->RCC_AHB1ENR |= (SET<<1))
#define GPIOC_PCLK_EN()	(pRCC->RCC_AHB1ENR |= (SET<<2))
#define GPIOD_PCLK_EN()	(pRCC->RCC_AHB1ENR |= (SET<<3))
#define GPIOE_PCLK_EN()	(pRCC->RCC_AHB1ENR |= (SET<<4))
#define GPIOH_PCLK_EN()	(pRCC->RCC_AHB1ENR |= (SET<<7))

#define GPIOA_PCLK_DIS()	(pRCC->RCC_AHB1ENR &= (RESET<<0))
#define GPIOB_PCLK_DIS()	(pRCC->RCC_AHB1ENR &= (RESET<<1))
#define GPIOC_PCLK_DIS()	(pRCC->RCC_AHB1ENR &= (RESET<<2))
#define GPIOD_PCLK_DIS()	(pRCC->RCC_AHB1ENR &= (RESET<<3))
#define GPIOE_PCLK_DIS()	(pRCC->RCC_AHB1ENR &= (RESET<<4))
#define GPIOH_PCLK_DIS()	(pRCC->RCC_AHB1ENR &= (RESET<<7))


#define GPIOA_REG_RESET()	do{pRCC->RCC_AHB1RSTR |= SET<<0; pRCC->RCC_AHB1RSTR &= ~(SET<<0);}while(0)   //dont add ; at the end here
#define GPIOB_REG_RESET()	do{pRCC->RCC_AHB1RSTR |= SET<<0; pRCC->RCC_AHB1RSTR &= ~(SET<<1);}while(0)
#define GPIOC_REG_RESET()	do{pRCC->RCC_AHB1RSTR |= SET<<0; pRCC->RCC_AHB1RSTR &= ~(SET<<2);}while(0)
#define GPIOD_REG_RESET()	do{pRCC->RCC_AHB1RSTR |= SET<<0; pRCC->RCC_AHB1RSTR &= ~(SET<<3);}while(0)
#define GPIOE_REG_RESET()	do{pRCC->RCC_AHB1RSTR |= SET<<0; pRCC->RCC_AHB1RSTR &= ~(SET<<4);}while(0)
#define GPIOH_REG_RESET()	do{pRCC->RCC_AHB1RSTR |= SET<<0; pRCC->RCC_AHB1RSTR &= ~(SET<<7);}while(0)
/*
void GPIOA_Pclk_Enable(void)
{
	pRCC->RCC_AHB1ENR |= (SET<<0);
}

void GPIOA_Pclk_Disable(void)
{
	pRCC->RCC_AHB1ENR &= (RESET<<0);
}

void GPIOB_Pclk_Enable(void)
{
	pRCC->RCC_AHB1ENR |= (SET<<1);
}

void GPIOB_Pclk_Disable(void)
{
	pRCC->RCC_AHB1ENR &= (RESET<<1);
}

void GPIOC_Pclk_Enable(void)
{
	pRCC->RCC_AHB1ENR |= (SET<<2);
}

void GPIOC_Pclk_Disable(void)
{
	pRCC->RCC_AHB1ENR &= (RESET<<2);
}

void GPIOD_Pclk_Enable(void)
{
	pRCC->RCC_AHB1ENR |= (SET<<3);
}

void GPIOD_Pclk_Disable(void)
{
	pRCC->RCC_AHB1ENR &= (RESET<<3);
}

void GPIOE_Pclk_Enable(void)
{
	pRCC->RCC_AHB1ENR |= (SET<<4);
}

void GPIOE_Pclk_Disable(void)
{
	pRCC->RCC_AHB1ENR &= (RESET<<4);
}

void GPIOH_Pclk_Enable(void)
{
	pRCC->RCC_AHB1ENR |= (SET<<7);
}

void GPIOH_Pclk_Disable(void)
{
	pRCC->RCC_AHB1ENR &= (RESET<<7);
}

*/




#endif /* INC_RCC_CLOCK_H_ */