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


/* Peripheral Register Definition Structure for RTC */
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
	}RTC_RegDef_t;

/* MACROs for RCC (peripheral base addr typecasted to regdef structures) */
#define RCC ((RTC_RegDef_t*)(RCC_BASEADDR))


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


/* RCC Clock Enable MACROS */
/*
#define GPIOA_PCLK_EN()	(RCC->RCC_APB1ENR |= (1<<0));
#define GPIOB_PCLK_EN()	(RCC->RCC_APB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()	(RCC->RCC_APB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()	(RCC->RCC_APB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()	(RCC->RCC_APB1ENR |= (1<<4))
#define GPIOH_PCLK_EN()	(RCC->RCC_APB1ENR |= (1<<7))
*/
/* RCC Clock Disable MACROS */
/*
#define GPIOA_PCLK_DIS()	(pRCC->RCC_APB1ENR &= (0<<0))
#define GPIOB_PCLK_DIS()	(RCC->RCC_APB1ENR &= (0<<1))
#define GPIOC_PCLK_DIS()	(RCC->RCC_APB1ENR &= (0<<2))
#define GPIOD_PCLK_DIS()	(RCC->RCC_APB1ENR &= (0<<3))
#define GPIOE_PCLK_DIS()	(RCC->RCC_APB1ENR &= (0<<4))
#define GPIOH_PCLK_DIS()	(RCC->RCC_APB1ENR &= (0<<7))
*/

/* Generic MACROs */
#define ENABLE			1
#define DISABLE			0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET 	SET
#define GPIO_PIN_RESET 	RESET

#endif /* INC_STM32F411XX_H_ */
