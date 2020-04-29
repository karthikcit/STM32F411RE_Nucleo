/*
 * pushbtn_it.c
 *
 *  Created on: 29-Apr-2020
 *      Author: KarthikM
 */

#include "stdio.h"
#include "stdbool.h"
#include "string.h"
#include "stm32f411xx.h"
#include "gpio_driver.h"

void delay(void) {
	for (uint32_t i = 0; i < 1000000; i++)
		;

}

int main(void) {

	GPIO_Handle_t GpioLed, GpioButton;

	memset(&GpioLed,0,sizeof(GpioLed));
	memset(&GpioButton,0,sizeof(GpioButton));


	GpioLed.pGPIO = GPIOA;
	GpioLed.pGPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	GpioLed.pGPIO_PinConfig.GPIO_PinMode = GPIO_PINMODE_OUT;
	GpioLed.pGPIO_PinConfig.GPIO_PinSpeed = GPIO_OUTSPEED_FAST;
	GpioLed.pGPIO_PinConfig.GPIO_PinOPType = GPIO_OT_OPENDRAIN;
	GpioLed.pGPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_PU;

	GPIO_Pclk_Control(GPIOA, ENABLE);

	GPIO_Init(&GpioLed);

	GpioButton.pGPIO = GPIOC;
	GpioButton.pGPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GpioButton.pGPIO_PinConfig.GPIO_PinMode = GPIO_PINMODE_IT_FT;
	GpioButton.pGPIO_PinConfig.GPIO_PinSpeed = GPIO_OUTSPEED_FAST;
	GpioButton.pGPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_PU;

	GPIO_Pclk_Control(GPIOC, ENABLE);

	GPIO_Init(&GpioButton);


	//IRQ CONFIGURATIONS
	//GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRIORITY_15);
	GPIO_IRQ_Interrupt_Config(IRQ_NO_EXTI15_10, ENABLE);


	while(1)
	{
		printf(" %d \n ",GPIO_PinRead(GPIOC,GPIO_PIN_13));
	}


}

void EXTI15_10_IRQHandler(void){
	GPIO_IRQHandler(GPIO_PIN_13);
	GPIO_PinToggle(GPIOA, GPIO_PIN_5);
}
