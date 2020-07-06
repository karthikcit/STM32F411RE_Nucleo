/*
 * gpio_driver.c
 *
 *  Created on: 18-Apr-2020
 *      Author: KarthikM
 */
#include "gpio_driver.h"

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
		if(pGPIOHandle->pGPIO_PinConfig.GPIO_PinMode == GPIO_PINMODE_IT_FT )
		{
			//1. Configur FTSR
			EXTI->EXTI_FTSR |= (1<< pGPIOHandle->pGPIO_PinConfig.GPIO_PinNumber);
			EXTI->EXTI_RTSR &= ~(1<< pGPIOHandle->pGPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->pGPIO_PinConfig.GPIO_PinMode == GPIO_PINMODE_IT_RT)
		{
			//1. Configur RTSR
			EXTI->EXTI_RTSR |= (1<< pGPIOHandle->pGPIO_PinConfig.GPIO_PinNumber);
			EXTI->EXTI_FTSR &= ~(1<< pGPIOHandle->pGPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->pGPIO_PinConfig.GPIO_PinMode == GPIO_PINMODE_IT_RFT)
		{
			//1. Configur Both FTSR & RTSR
			EXTI->EXTI_FTSR |= (1<< pGPIOHandle->pGPIO_PinConfig.GPIO_PinNumber);
			EXTI->EXTI_RTSR |= (1<< pGPIOHandle->pGPIO_PinConfig.GPIO_PinNumber);
		}

		//2. Configure gpio pprt selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->pGPIO_PinConfig.GPIO_PinNumber /4; //to fix the EXTICR reg
		uint8_t temp2 = pGPIOHandle->pGPIO_PinConfig.GPIO_PinNumber %4;

		//Enable Peripheral Clock
		SYSCFG_PCLK_EN();


		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIO);

		SYSCFG->SYSCFG_EXTICR[temp1] |= portcode << (4*temp2);

		//3. Configure exti interrupt delivery in IMR
		EXTI->EXTI_IMR |= (1<< pGPIOHandle->pGPIO_PinConfig.GPIO_PinNumber);


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
		pGPIOHandle->pGPIO->GPIOx_AFRH &= ~((0xF)<<(4 * temp2));
		pGPIOHandle->pGPIO->GPIOx_AFRH |= temp ;
	}
	temp = 0;
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIO)
{
	//if(pGPIOHandle->pGPIO == GPIOA)
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


void GPIO_IRQ_Interrupt_Config(uint8_t IRQNumber,uint8_t ENorDIS)
{
	if(ENorDIS == ENABLE)
	{
		if(IRQNumber < 32)
		{
			*NVIC_ISER0 = 1<<IRQNumber;
		}
		else if((IRQNumber >= 32) && (IRQNumber < 64))
		{
			*NVIC_ISER1 = 1<<(IRQNumber%32);
		}
		else if((IRQNumber >= 64) && (IRQNumber < 96))
		{
			*NVIC_ISER2 = 1<<(IRQNumber%64);
		}
		//STM32F411RE controller has interrupt numbers till 85 so configuring upto ISER2 is enough
	}
	else if(ENorDIS == DISABLE)
	{
		if(IRQNumber < 32)
		{
			*NVIC_ICER0 = 1<<IRQNumber;
		}
		else if((IRQNumber >= 32) && (IRQNumber < 64))
		{
			*NVIC_ICER1 = 1<<(IRQNumber%32);
		}
		else if((IRQNumber >= 64) && (IRQNumber < 96))
		{
			*NVIC_ICER2 = 1<<(IRQNumber%64);
		}
		//STM32F411RE controller has interrupt numbers till 85 so configuring upto ICER2 is enough
	}

}


void GPIO_IRQ_ITPriority(uint8_t IRQNumber,uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;  //to select the register
	uint8_t iprx_selectiom = IRQNumber % 4; //to select the Byte in the selected register

	//To shift in proper place since NO_OF_PRIORITYBITS_IMPLEMENTED differs controller to controller
	uint8_t Shift_Val = (8 *  iprx_selectiom) + (8 - NO_OF_PRIORITYBITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx) |= IRQPriority << Shift_Val ;

}

void GPIO_IRQHandler(uint8_t PinNumber)
{
	//Clear the EXTI PR reg  corresponding to the pin number
	if(EXTI->EXTI_PR & (1<<PinNumber))
	{
		EXTI->EXTI_PR |= 1<<PinNumber;
	}

}

