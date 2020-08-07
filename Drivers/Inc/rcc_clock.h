/*
 * rcc_clock.h
 *
 *  Created on: 18-Apr-2020
 *      Author: KarthikM
 */

#ifndef INC_RCC_CLOCK_H_
#define INC_RCC_CLOCK_H_

#include "stm32f411xx.h"

#if 0
RCC_RegDef_t *pRCC = RCC;


/* GPIOx Peripheral Clock Enable MACROs*/
#define GPIOA_PCLK_EN()	(pRCC->RCC_AHB1ENR |= (SET<<0))
#define GPIOB_PCLK_EN()	(pRCC->RCC_AHB1ENR |= (SET<<1))
#define GPIOC_PCLK_EN()	(pRCC->RCC_AHB1ENR |= (SET<<2))
#define GPIOD_PCLK_EN()	(pRCC->RCC_AHB1ENR |= (SET<<3))
#define GPIOE_PCLK_EN()	(pRCC->RCC_AHB1ENR |= (SET<<4))
#define GPIOH_PCLK_EN()	(pRCC->RCC_AHB1ENR |= (SET<<7))

/* GPIOx Peripheral Clock Disable MACROs*/
#define GPIOA_PCLK_DIS()	(pRCC->RCC_AHB1ENR &= (RESET<<0))
#define GPIOB_PCLK_DIS()	(pRCC->RCC_AHB1ENR &= (RESET<<1))
#define GPIOC_PCLK_DIS()	(pRCC->RCC_AHB1ENR &= (RESET<<2))
#define GPIOD_PCLK_DIS()	(pRCC->RCC_AHB1ENR &= (RESET<<3))
#define GPIOE_PCLK_DIS()	(pRCC->RCC_AHB1ENR &= (RESET<<4))
#define GPIOH_PCLK_DIS()	(pRCC->RCC_AHB1ENR &= (RESET<<7))

/* GPIOx Peripheral Clock Reset MACROs*/
#define GPIOA_REG_RESET()	do{pRCC->RCC_AHB1RSTR |= SET<<0; pRCC->RCC_AHB1RSTR &= ~(SET<<0);}while(0)   //dont add ; at the end here
#define GPIOB_REG_RESET()	do{pRCC->RCC_AHB1RSTR |= SET<<0; pRCC->RCC_AHB1RSTR &= ~(SET<<1);}while(0)
#define GPIOC_REG_RESET()	do{pRCC->RCC_AHB1RSTR |= SET<<0; pRCC->RCC_AHB1RSTR &= ~(SET<<2);}while(0)
#define GPIOD_REG_RESET()	do{pRCC->RCC_AHB1RSTR |= SET<<0; pRCC->RCC_AHB1RSTR &= ~(SET<<3);}while(0)
#define GPIOE_REG_RESET()	do{pRCC->RCC_AHB1RSTR |= SET<<0; pRCC->RCC_AHB1RSTR &= ~(SET<<4);}while(0)
#define GPIOH_REG_RESET()	do{pRCC->RCC_AHB1RSTR |= SET<<0; pRCC->RCC_AHB1RSTR &= ~(SET<<7);}while(0)


/* SYSCFG Peripheral Clock Enable MACROs*/
#define SYSCFG_PCLK_EN()	(pRCC->RCC_APB2ENR |= (SET<<14))


/* SPI Peripheral Clock Enable MACROs*/
#define SPI1_PCLK_EN()	(pRCC->RCC_APB2ENR |= (SET<<12))
#define SPI2_PCLK_EN()	(pRCC->RCC_APB1ENR |= (SET<<14))
#define SPI3_PCLK_EN()	(pRCC->RCC_APB1ENR |= (SET<<15))
#define SPI4_PCLK_EN()	(pRCC->RCC_APB2ENR |= (SET<<13))
#define SPI5_PCLK_EN()	(pRCC->RCC_APB2ENR |= (SET<<20))

/* SPI Peripheral Clock Disable MACROs*/
#define SPI1_PCLK_DIS()	(pRCC->RCC_APB2ENR &= (RESET<<12))
#define SPI2_PCLK_DIS()	(pRCC->RCC_APB1ENR &= (RESET<<14))
#define SPI3_PCLK_DIS()	(pRCC->RCC_APB1ENR &= (RESET<<15))
#define SPI4_PCLK_DIS()	(pRCC->RCC_APB2ENR &= (RESET<<13))
#define SPI5_PCLK_DIS()	(pRCC->RCC_APB2ENR &= (RESET<<20))

#else
void GPIOA_PCLK_EN(void);
void GPIOB_PCLK_EN(void);
void GPIOC_PCLK_EN(void);
void GPIOD_PCLK_EN(void);
void GPIOE_PCLK_EN(void);
void GPIOH_PCLK_EN(void);

void GPIOA_PCLK_DIS(void);
void GPIOB_PCLK_DIS(void);
void GPIOC_PCLK_DIS(void);
void GPIOD_PCLK_DIS(void);
void GPIOE_PCLK_DIS(void);
void GPIOH_PCLK_DIS(void);

/* SYSCFG Peripheral Clock Enable Functions*/
void SYSCFG_PCLK_EN(void);


/* SPI Peripheral Clock Enable Functions*/
void SPI1_PCLK_EN(void);
void SPI2_PCLK_EN(void);
void SPI3_PCLK_EN(void);
void SPI4_PCLK_EN(void);
void SPI5_PCLK_EN(void);

/* SPI Peripheral Clock Disable Functions*/
void SPI1_PCLK_DIS(void);
void SPI2_PCLK_DIS(void);
void SPI3_PCLK_DIS(void);
void SPI4_PCLK_DIS(void);
void SPI5_PCLK_DIS(void);

/* USART Pclk Enable  */
void USART1_PCLK_EN(void);
void USART6_PCLK_EN(void);
void USART2_PCLK_EN(void);

/* USART Pclk Disable  */
void USART1_PCLK_DIS(void);
void USART6_PCLK_DIS(void);
void USART2_PCLK_DIS(void);

#endif
#endif /* INC_RCC_CLOCK_H_ */
