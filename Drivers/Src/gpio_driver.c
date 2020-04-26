/*
 * gpio_driver.c
 *
 *  Created on: 18-Apr-2020
 *      Author: KarthikM
 */
#include "stm32f411xx.h"
#include "gpio_driver.h"
#include "rcc_clock.h"

/*
 * Function: GPIO_Pclk_Control
 * Definition: To enable.disable pclk
 * param1 : base address of GPIO peripheral
 * param2 : en/disable input
 * return : void
 */
void GPIO_Pclk_Control(GPIO_RegDef_t *pGPIO,uint8_t ENorDIS)
{
	if(ENorDIS == ENABLE)
	{
		if(pGPIO == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIO == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIO == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIO == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIO == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIO == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if(pGPIO == GPIOA)
		{
			GPIOA_PCLK_DIS();
		}
		else if(pGPIO == GPIOB)
		{
			GPIOB_PCLK_DIS();
		}
		else if(pGPIO == GPIOC)
		{
			GPIOC_PCLK_DIS();
		}
		else if(pGPIO == GPIOD)
		{
			GPIOD_PCLK_DIS();
		}
		else if(pGPIO == GPIOE)
		{
			GPIOE_PCLK_DIS();
		}
		else if(pGPIO == GPIOH)
		{
			GPIOH_PCLK_DIS();
		}

	}

}

//void GPIO_Init(GPIO_RegDef_t *pGPIO,GPIO_PinConfig_t pGPIO_PinConfig)
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp=0;
	//1 Config Pinmode
	if(pGPIOHandle->pGPIO_PinConfig.GPIO_PinMode <= GPIO_PINMODE_AN)
	{
		//non interrupt mode
		//pinnum mul by 2 bcoz each pin takes 2 bitfields in the reg
		temp = (pGPIOHandle->pGPIO_PinConfig.GPIO_PinMode)<<(2 * (pGPIOHandle->pGPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIO->MODER &= (~(0x3))<<(2 * (pGPIOHandle->pGPIO_PinConfig.GPIO_PinNumber));  //clear bit
		pGPIOHandle->pGPIO->MODER |= temp ;
	}
	else
	{
		//interrupt mode do later
	}
	temp = 0;

	//2 Config PinSpeed
	//pinnum mul by 2 bcoz each pin takes 2 bitfields in the reg
	temp = (pGPIOHandle->pGPIO_PinConfig.GPIO_PinSpeed)<<(2 * (pGPIOHandle->pGPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIO->GPIOx_OSPEEDR &= (~(0x3))<<(2 * (pGPIOHandle->pGPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIO->GPIOx_OSPEEDR |= temp ;
	temp = 0;

	//3 Confing PUPD
	//pinnum mul by 2 bcoz each pin takes 2 bitfields in the reg
	temp = (pGPIOHandle->pGPIO_PinConfig.GPIO_PinPuPdControl)<<(2 * (pGPIOHandle->pGPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIO->GPIOx_PUPDR &= (~(0x3))<<(2 * (pGPIOHandle->pGPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIO->GPIOx_PUPDR |= temp ;
	temp = 0;

	//4 Config op settings
	temp = (pGPIOHandle->pGPIO_PinConfig.GPIO_PinOPType)<<(pGPIOHandle->pGPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIO->GPIOx_OTYPER &= (~(0x1))<<(pGPIOHandle->pGPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIO->GPIOx_OTYPER |= temp ;
	temp = 0;

	//5 Config ALT Functionality
	uint8_t temp1=0,temp2 =0;
	temp1 = (pGPIOHandle->pGPIO_PinConfig.GPIO_PinNumber)/8;  //for low/high altfun reg fixing
	temp2 = (pGPIOHandle->pGPIO_PinConfig.GPIO_PinNumber)%8;  //for bit position fixing
	if(temp1 == 0)
	{
		//AFRL
		temp = (pGPIOHandle->pGPIO_PinConfig.GPIO_PinAltFunMode)<<(4 * temp2);
		pGPIOHandle->pGPIO->GPIOx_AFRL &= (~(0xF))<<(4 * temp2);
		pGPIOHandle->pGPIO->GPIOx_AFRL |= temp ;

	}
	else
	{
		//AFRH
		temp = (pGPIOHandle->pGPIO_PinConfig.GPIO_PinAltFunMode)<<(4 * temp2);
		pGPIOHandle->pGPIO->GPIOx_AFRH &= (~(0xF))<<(4 * temp2);
		pGPIOHandle->pGPIO->GPIOx_AFRH |= temp ;
	}
	temp = 0;
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIO)
{
	//if(pGPIOHandle->pGPIO == GPIOA)
	if(pGPIO == GPIOA)
	{
		GPIOA_PCLK_EN();
	}
	else if(pGPIO == GPIOB)
	{
		GPIOB_PCLK_EN();
	}
	else if(pGPIO == GPIOC)
	{
		GPIOC_PCLK_EN();
	}
	else if(pGPIO == GPIOD)
	{
		GPIOD_PCLK_EN();
	}
	else if(pGPIO == GPIOE)
	{
		GPIOE_PCLK_EN();
	}
	else if(pGPIO == GPIOH)
	{
		GPIOH_PCLK_EN();
	}
}

uint8_t GPIO_PinRead(GPIO_RegDef_t *pGPIO,uint8_t PinNumber)
{
	uint8_t value=0;
	value = (pGPIO->GPIOx_IDR >> PinNumber) & (0x00000001);
	return value;
}

uint16_t GPIO_PortRead(GPIO_RegDef_t *pGPIO)
{
	uint16_t value=0;
	value = pGPIO->GPIOx_IDR;
	return value;

}

void GPIO_PinWrite(GPIO_RegDef_t *pGPIO,uint8_t PinNumber,uint8_t value)
{
	if(value == GPIO_PIN_SET)
	{
		pGPIO->GPIOx_ODR |= (value << PinNumber);
	}
	else
	{
		pGPIO->GPIOx_ODR &= (value << PinNumber);
	}
}

void GPIO_PortWrite(GPIO_RegDef_t *pGPIO,uint16_t value)
{
	pGPIO->GPIOx_ODR |= value ;
}

void GPIO_PinToggle_ByRead(GPIO_RegDef_t *pGPIO,uint8_t PinNumber)
{
	/* method 1 */
	uint8_t value = 0;
	value = ((pGPIO->GPIOx_ODR >> PinNumber) & (0x00000001));
	pGPIO->GPIOx_ODR |=  (~value) << PinNumber ;
}

void GPIO_PinToggle(GPIO_RegDef_t *pGPIO,uint8_t PinNumber)
{
	/* method 2 */
	pGPIO->GPIOx_ODR ^= (1<<(PinNumber));
}


void GPIO_IRQConfig(uint8_t IRQNumber,uint8_t IRQPriority,uint8_t ENorDIS)
{

}

void GPIO_IRQHandler(uint8_t PinNumbers)
{

}
