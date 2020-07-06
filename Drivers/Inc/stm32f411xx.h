/*
 * stm32f411xx.h
 *
 *  Created on: Apr 17, 2020
 *      Author: KarthikM
 */

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

#include <stdint.h>

/* Processor Specific info for ARM Cortex Mx  NVIC ISER Registers */
#define NVIC_ISER0	((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1	((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2	((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3	((volatile uint32_t*)0xE000E10C)
#define NVIC_ISER4	((volatile uint32_t*)0xE000E110)
#define NVIC_ISER5	((volatile uint32_t*)0xE000E114)
#define NVIC_ISER6	((volatile uint32_t*)0xE000E118)
#define NVIC_ISER7	((volatile uint32_t*)0xE000E11c)

/* Processor Specific info for ARM Cortex Mx  NVIC ICER Registers */
#define NVIC_ICER0	((volatile uint32_t*)0xE000E180)
#define NVIC_ICER1	((volatile uint32_t*)0xE000E184)
#define NVIC_ICER2	((volatile uint32_t*)0xE000E188)
#define NVIC_ICER3	((volatile uint32_t*)0xE000E18C)
#define NVIC_ICER4	((volatile uint32_t*)0xE000E190)
#define NVIC_ICER5	((volatile uint32_t*)0xE000E194)
#define NVIC_ICER6	((volatile uint32_t*)0xE000E198)
#define NVIC_ICER7	((volatile uint32_t*)0xE000E19c)

/* Processor Specific info for ARM Cortex Mx  NVIC Priority Registers */
#define NVIC_PR_BASE_ADDR ((volatile uint32_t*)0xE000E400)

#define NO_OF_PRIORITYBITS_IMPLEMENTED 4

/*  MACROs for MCU Memories FLASH,SRAM */
#define SRAM1_BASE_ADDR	0X20000000U
#define SRAM			SRAM1_BASE_ADDR
#define ROM				0x1FFF0000U
#define FLASH_BASE_ADDR	0x08000000U

/* MACROs for AHB,APB Bus Peripheral BASEADDRESSES */
#define APB1PERIPH_BASE 0x40000000U
#define APB2PERIPH_BASE 0x40010000U
#define AHB1PERIPH_BASE 0x40020000U
#define AHB2PERIPH_BASE 0x50000000U
#define PERIPH_BASE APB1PERIPH_BASE

/* MACROs for Peripherals on APB1 Bus */
#define TIM2_BASEADDR	(APB1PERIPH_BASE+0X0000u)
#define TIM3_BASEADDR	(APB1PERIPH_BASE+0X0400u)
#define TIM4_BASEADDR	(APB1PERIPH_BASE+0X0800u)
#define TIM5_BASEADDR	(APB1PERIPH_BASE+0X0c00u)

#define RTC_BASEADDR	(APB1PERIPH_BASE+0X2800u)
#define WWDG_BASEADDR	(APB1PERIPH_BASE+0X2C00u)
#define IWDG_BASEADDR	(APB1PERIPH_BASE+0X3000u)

#define I2S2EXT_BASEADDR	(APB1PERIPH_BASE+0X3400u)
#define SPI2_I2S2_BASEADDR	(APB1PERIPH_BASE+0X3800u)
#define SPI3_I2S3_BASEADDR	(APB1PERIPH_BASE+0X3C00u)
#define I2S3EXT_BASEADDR	(APB1PERIPH_BASE+0X4000u)

#define USART2_BASEADDR	(APB1PERIPH_BASE+0X0400u)

#define I2C1_BASEADDR	(APB1PERIPH_BASE+0X5400u)
#define I2C2_BASEADDR	(APB1PERIPH_BASE+0X5800u)
#define I2C3_BASEADDR	(APB1PERIPH_BASE+0X5C00u)

#define PWR_BASEADDR	(APB1PERIPH_BASE+0X7000u)


/* MACROs for Peripherals on APB2 Bus */
#define TIM1_BASEADDR	(APB2PERIPH_BASE+0X0000u)
#define USART1_BASEADDR	(APB2PERIPH_BASE+0X1000u)
#define USART6_BASEADDR	(APB2PERIPH_BASE+0X1400u)
#define ADC1_BASEADDR	(APB2PERIPH_BASE+0X2000u)
#define SDIO_BASEADDR	(APB2PERIPH_BASE+0X2C00u)
#define SPI1_I2S1_BASEADDR	(APB2PERIPH_BASE+0X3000u)
#define SPI4_I2S4_BASEADDR	(APB2PERIPH_BASE+0X3400u)
#define SYSCFG_BASEADDR	(APB2PERIPH_BASE+0X3800u)
#define EXTI_BASEADDR	(APB2PERIPH_BASE+0X3C00u)
#define TIM9_BASEADDR	(APB2PERIPH_BASE+0X4000u)
#define TIM10_BASEADDR	(APB2PERIPH_BASE+0X4400u)
#define TIM11_BASEADDR	(APB2PERIPH_BASE+0X4800u)
#define SPI5_I2S5_BASEADDR	(APB2PERIPH_BASE+0X5000u)


/* MACROs for Peripherals on AHB1 Bus */
#define GPIOA_BASEADDR (AHB1PERIPH_BASE + 0x0000u)
#define GPIOB_BASEADDR (AHB1PERIPH_BASE + 0x0400u)
#define GPIOC_BASEADDR (AHB1PERIPH_BASE + 0x0800u)
#define GPIOD_BASEADDR (AHB1PERIPH_BASE + 0x0C00u)
#define GPIOE_BASEADDR (AHB1PERIPH_BASE + 0x1000u)
#define GPIOH_BASEADDR (AHB1PERIPH_BASE + 0x1C00u)

#define CRC_BASEADDR	(AHB1PERIPH_BASE + 0x3000u)
#define RCC_BASEADDR	(AHB1PERIPH_BASE + 0x3800u)
#define FIR_BASEADDR	(AHB1PERIPH_BASE + 0x3C00u)  //Flash Interface Reg
#define DMA1_BASEADDR	(AHB1PERIPH_BASE + 0x6000u)
#define DMA2_BASEADDR	(AHB1PERIPH_BASE + 0x6400u)


/* MACROs for Peripherals on AHB2 Bus */
#define USBOTG_FS_BASEADDR AHB2PERIPH_BASE


/* Peripheral Register Definition Structure for GPIO */
typedef struct
{
	volatile uint32_t MODER;			/* offset 0x00 */
	volatile uint32_t GPIOx_OTYPER;		/* offset 0x04 */
	volatile uint32_t GPIOx_OSPEEDR;	/* offset 0x08 */
	volatile uint32_t GPIOx_PUPDR;		/* offset 0x0C */
	volatile uint32_t GPIOx_IDR;		/* offset 0x10 */
	volatile uint32_t GPIOx_ODR;		/* offset 0x14 */
	volatile uint32_t GPIOx_BSRR;		/* offset 0x18 */
	volatile uint32_t GPIOx_LCKR;		/* offset 0x1C */
	volatile uint32_t GPIOx_AFRL;		/* offset 0x20 */
	volatile uint32_t GPIOx_AFRH;		/* offset 0x24 */
	//volatile uint32_t GPIOx_AFR[2];  we can declare like this also
	}GPIO_RegDef_t;

//	GPIOA_RegDef_t *pGPIOA = (GPIOA_RegDef_t*)GPIOA_BASEADDR; TO USE THIS REG DEF IN APPLICATION

/* MACROs for Peripharals (peripheral base addr typecasted to regdef structures) */
#define GPIOA ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH ((GPIO_RegDef_t*)GPIOH_BASEADDR)


/* Peripheral Register Definition Structure for RCC */
typedef struct
	{
		volatile uint32_t RCC_CR;     		/* offset 0x00 */
		volatile uint32_t RCC_PLLCFGR;		/* 	0x04 */
		volatile uint32_t RCC_CFGR;   		/*	0x08 */
		volatile uint32_t RCC_CIR; 			/*	0x0C */
		volatile uint32_t RCC_AHB1RSTR; 	/*	0x10 */
		volatile uint32_t RCC_AHB2RSTR; 	/*	0x14 */
		volatile uint32_t Reserved1;		/*	0x18 */
		volatile uint32_t Reserved2;       	/*	0x1C */
		volatile uint32_t RCC_APB1RSTR;		/*	0x20 */
		volatile uint32_t RCC_APB2RSTR;		/*	0x24 */
		volatile uint32_t Reserved3;		/*	0x28 */
		volatile uint32_t Reserved4; 		/*	0x2C */
		volatile uint32_t RCC_AHB1ENR;		/*	0x30 */
		volatile uint32_t RCC_AHB2ENR;		/*	0x34 */
		volatile uint32_t Reserved5;		/*	0x38 */
		volatile uint32_t Reserved6;		/*	0x3c */
		volatile uint32_t RCC_APB1ENR;		/*  0x40 */
		volatile uint32_t RCC_APB2ENR;		/*  0x44 */
		volatile uint32_t Reserved7;		/*	0x48 */
		volatile uint32_t Reserved8;		/*	0x4c */
		volatile uint32_t RCC_AHB1LPENR;	/*	0x50 */
		volatile uint32_t RCC_AHB2LPENR;	/*	0x54 */
		volatile uint32_t Reserved9;		/*	0x58 */
		volatile uint32_t Reserved10;		/*	0x5c */
		volatile uint32_t RCC_APB1LPENR;	/*	0x60 */
		volatile uint32_t RCC_APB2LPENR;	/*	0x64 */
		volatile uint32_t Reserved11;		/*	0x68 */
		volatile uint32_t Reserved12;		/*	0x6C */
		volatile uint32_t RCC_BDCR;			/*	0x70 */
		volatile uint32_t RCC_CSR;			/*	0x74 */
		volatile uint32_t Reserved13;		/*	0x78 */
		volatile uint32_t Reserved14;		/*	0x7C */
		volatile uint32_t RCC_SSCGR;		/*	0x80 */
		volatile uint32_t RCC_PLLI2SCFGR;	/*	0x84 */
		volatile uint32_t Reserved15;		/*	0x88 */
		volatile uint32_t RCC_DCKCFGR;		/*	0x8C */
	}RCC_RegDef_t;

/* MACROs for RCC (peripheral base addr typecasted to regdef structures) */
#define RCC ((RCC_RegDef_t*)(RCC_BASEADDR))


/* Peripheral Register Definition Structure for EXTI */
typedef struct
{
	volatile uint32_t EXTI_IMR;   /* Offset 0x00 */
	volatile uint32_t EXTI_EMR;   /* Offset 0x04 */
	volatile uint32_t EXTI_RTSR;   /* Offset 0x08 */
	volatile uint32_t EXTI_FTSR;   /* Offset 0x0C */
	volatile uint32_t EXTI_SWIER;   /* Offset 0x10 */
	volatile uint32_t EXTI_PR;   /* Offset 0x14 */

}EXTI_RegDef_t;

/* MACROs for EXTI (peripheral base addr typecasted to regdef structures) */
#define EXTI ((EXTI_RegDef_t*)(EXTI_BASEADDR))

/* Peripheral Register Definition Structure for syscfg */
typedef struct
{
	volatile uint32_t SYSCFG_MEMRMP;    /* Offset 0x00 */
	volatile uint32_t SYSCFG_PMC;	    /* Offset 0x04 */
//	volatile uint32_t SYSCFG_EXTICR1;   /* Offset 0x08 */
//	volatile uint32_t SYSCFG_EXTICR2;   /* Offset 0x0C */
//	volatile uint32_t SYSCFG_EXTICR3;   /* Offset 0x10 */
//	volatile uint32_t SYSCFG_EXTICR4;  	/* Offset 0x14 */
	volatile uint32_t SYSCFG_EXTICR[4];
	volatile uint32_t Reserved1;		/* Offset 0x18 */
	volatile uint32_t Reserved2;		/* Offset 0x1C */
	volatile uint32_t SYSCFG_CMPCR; 	/* Offset 0x20 */
}SYSCFG_RegDef_t;

/* MACROs for EXTI (peripheral base addr typecasted to regdef structures) */
#define SYSCFG ((SYSCFG_RegDef_t*)(SYSCFG_BASEADDR))

#define GPIO_BASEADDR_TO_CODE(x)	((x==GPIOA) ? 0 :\
									 (x==GPIOB) ? 1 :\
									 (x==GPIOC) ? 2 :\
									 (x==GPIOD) ? 3 :\
									 (x==GPIOE) ? 4 :\
									 (x==GPIOA) ? 7 :0)


#define IRQ_NO_EXTI0		6	//exti line 0
#define IRQ_NO_EXTI1		7	//exti line 1
#define IRQ_NO_EXTI2		8	//exti line 2
#define IRQ_NO_EXTI3		9	//exti line 3
#define IRQ_NO_EXTI4		10	//exti line 4
#define IRQ_NO_EXTI9_5		23	//exti lines 5 - 9
#define IRQ_NO_EXTI15_10	40  //exti lines 10 - 15

/* Peripheral Register Definition Structure for SPI */
typedef struct
{
	volatile uint32_t SPI_CR1;		/* Offset 0x00 */
	volatile uint32_t SPI_CR2;		/* Offset 0x04 */
	volatile uint32_t SPI_SR;		/* Offset 0x08 */
	volatile uint32_t SPI_DR;		/* Offset 0x0C */
	volatile uint32_t SPI_CRCPR;	/* Offset 0x10 */
	volatile uint32_t SPI_RXCRCR;	/* Offset 0x14 */
	volatile uint32_t SPI_TXCRCR; 	/* Offset 0x18 */
	volatile uint32_t SPI_I2SCFGR;	/* Offset 0x1c */
	volatile uint32_t SPI_I2SPR;	/* Offset 0x20 */
}SPI_RegDef_t;

/* MACROs for SPI (peripheral base addr typecasted to regdef structures) */
#define SPI1 ((SPI_RegDef_t*)(SPI1_I2S1_BASEADDR))
#define SPI2 ((SPI_RegDef_t*)(SPI2_I2S2_BASEADDR))
#define SPI3 ((SPI_RegDef_t*)(SPI3_I2S3_BASEADDR))
#define SPI4 ((SPI_RegDef_t*)(SPI4_I2S4_BASEADDR))
#define SPI5 ((SPI_RegDef_t*)(SPI5_I2S5_BASEADDR))


/* Peripheral Register Definition Structure for I2C */
typedef struct
{
	volatile uint32_t I2C_CR1;		/* Offset 0x00 */
	volatile uint32_t I2C_CR2;		/* Offset 0x04 */
	volatile uint32_t I2C_OAR1;		/* Offset 0x08 */
	volatile uint32_t I2C_OAR2;		/* Offset 0x0C */
	volatile uint32_t I2C_DR;		/* Offset 0x10 */
	volatile uint32_t I2C_SR1;		/* Offset 0x14 */
	volatile uint32_t I2C_SR2;	 	/* Offset 0x18 */
	volatile uint32_t I2C_CCR;		/* Offset 0x1c */
	volatile uint32_t I2C_TRISE;	/* Offset 0x20 */
	volatile uint32_t I2C_FLTR;		/* Offset 0x24 */
}I2C_RegDef_t;

/* MACROs for I2C (peripheral base addr typecasted to regdef structures) */
#define I2C1 ((I2C_RegDef_t*)(I2C1_BASEADDR))
#define I2C2 ((I2C_RegDef_t*)(I2C2_BASEADDR))
#define I2C3 ((I2C_RegDef_t*)(I2C3_BASEADDR))


/* Peripheral Register Definition Structure for USART */
typedef struct
{
	volatile uint32_t USART_SR;		/* Offset 0x00 */
	volatile uint32_t USART_DR;		/* Offset 0x04 */
	volatile uint32_t USART_BRR;	/* Offset 0x08 */
	volatile uint32_t USART_CR1;	/* Offset 0x0C */
	volatile uint32_t USART_CR2;	/* Offset 0x10 */
	volatile uint32_t USART_CR3;	/* Offset 0x14 */
	volatile uint32_t USART_GTPR; 	/* Offset 0x18 */
}USART_RegDef_t;


/* MACROs for USART (peripheral base addr typecasted to regdef structures) */
#define USART1 ((USART_RegDef_t*)(USART1_BASEADDR))
#define USART2 ((USART_RegDef_t*)(USART2_BASEADDR))
#define USART6 ((USART_RegDef_t*)(USART6_BASEADDR))


/* Peripheral Register Definition Structure for ADC */
typedef struct
{
	volatile uint32_t ADC_SR;		/* Offset 0x00 */
	volatile uint32_t ADC_CR1;		/* Offset 0x04 */
	volatile uint32_t ADC_CR2;		/* Offset 0x08 */
	volatile uint32_t ADC_SMPR1;	/* Offset 0x0C */
	volatile uint32_t ADC_SMPR2;	/* Offset 0x10 */
	volatile uint32_t ADC_JOFR1;	/* Offset 0x14 */
	volatile uint32_t ADC_JOFR2;	/* Offset 0x18 */
	volatile uint32_t ADC_JOFR3;	/* Offset 0x1C */
	volatile uint32_t ADC_JOFR4;	/* Offset 0x20 */
	volatile uint32_t ADC_HTR;		/* Offset 0x24 */
	volatile uint32_t ADC_LTR;		/* Offset 0x28 */
	volatile uint32_t ADC_SQR1;		/* Offset 0x2C */
	volatile uint32_t ADC_SQR2;		/* Offset 0x30 */
	volatile uint32_t ADC_SQR3;		/* Offset 0x34 */
	volatile uint32_t ADC_JSQR;		/* Offset 0x38 */
	volatile uint32_t ADC_JDR1;		/* Offset 0x3C */
	volatile uint32_t ADC_JDR2;		/* Offset 0x40 */
	volatile uint32_t ADC_JDR3;		/* Offset 0x44 */
	volatile uint32_t ADC_JDR4;		/* Offset 0x48 */
	volatile uint32_t ADC_DR;		/* Offset 0x4C */

}ADC_RegDef_t;

/* MACROs for ADC (peripheral base addr typecasted to regdef structures) */
#define ADC1 ((ADC_RegDef_t*)(ADC1_BASEADDR))

typedef struct
{
	volatile uint32_t Res1;		/* Offset 0x00 */
	volatile uint32_t ADC_CCR;	/* Offset 0x04 */
}ADC_CCR_RegDef_t;

#define ADC_CCR ((ADC_CCR_RegDef_t*)(ADC1_BASEADDR+0x300))

/* Generic MACROs */
#define ENABLE			1
#define DISABLE			0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET 	SET
#define GPIO_PIN_RESET 	RESET
#define FLAG_RESET      RESET
#define FLAG_SET 		SET




#include "rcc_clock.h"


#endif /* INC_STM32F411XX_H_ */
