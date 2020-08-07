/*
 * usart_main.c
 *
 *  Created on: 29-Jul-2020
 *      Author: KarthikM
 */


#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#include "stdio.h"
#include "stdbool.h"
#include "string.h"
#include "stm32f411xx.h"
#include "gpio_driver.h"
#include "spi_driver.h"
#include "usart_driver.h"
#include "test.h"

/* Init MACROs */
#define LED_INIT		0
#define SWITCH_INIT		0
#define BTN_INIT		0
#define SPI_INIT		0
#define USART_INIT		1

#define LED_TEST 		0
#define SWITCH_TEST 	0
#define BTN_INT_TEST 	0 //!SWITCH_TEST
#define SPI_TEST		0
#define USART_TEST		1

#define SAMPLE_IRQ		0

#if BTN_INT_TEST
#define GPIO_IT_PORT	GPIOC
#define GPIO_IT_PIN		GPIO_PIN_13
volatile _Bool USER_KEY = true;
#endif


void sw_delay(void)
{
	for(uint32_t i=0;i<5000;i++);
}

#if SAMPLE_IRQ
void EXTI15_10_IRQHandler(void)
{
	GPIO_IRQHandler(GPIO_PIN_13);  //clear PR
	GPIO_PinToggle(GPIOA, GPIO_PIN_5); //toggle led
}
#endif

#if BTN_INT_TEST
void EXTI15_10_IRQHandler(void)
{
	GPIO_IRQHandler(GPIO_PIN_13);  //clear PR
	USER_KEY = false;
}
#endif


void USART2_GPIOInits(void)
{
	GPIO_Handle_t USARTPins;

	//find uart pinmux details nd complete this function
	GPIO_Pclk_Control(GPIOA,ENABLE);
	USARTPins.pGPIO = GPIOA;

	USARTPins.pGPIO_PinConfig.GPIO_PinMode = GPIO_PINMODE_ALTFN;
	USARTPins.pGPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF_7;
	USARTPins.pGPIO_PinConfig.GPIO_PinOPType = GPIO_OT_PUSHPULL;
	USARTPins.pGPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NO_PUPD;
	USARTPins.pGPIO_PinConfig.GPIO_PinSpeed = GPIO_OUTSPEED_HIGH;

	USARTPins.pGPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_2;  //usart2 Tx
	GPIO_Init(&USARTPins);

	USARTPins.pGPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_3;	//usart2 Rx
	GPIO_Init(&USARTPins);

}

void USART2_Inits(void)
{
	USART_Handle_t USART2Handle;
	USART2Handle.pUSARTx = USART2;
	USART2Handle.USART_Config.USART_Baud = USART_STD_BAUD_9600;
	USART2Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	USART2Handle.USART_Config.USART_Mode = USART_MODE_TXRX;
	USART2Handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_0_5;
	USART2Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART2Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;

	USART_Init(&USART2Handle);

}


int main(void)
{
	printf("hello world\n");

	printf("USART DRIVER TEST APP CODE\n");

	USART_Handle_t USART2Handle;

	//this function is used to initialize the GPIO pins to behave as SPI2 pins
	USART2_GPIOInits();


	USART2Handle.pUSARTx = USART2;
	USART2Handle.USART_Config.USART_Baud = USART_STD_BAUD_9600;
	USART2Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	USART2Handle.USART_Config.USART_Mode = USART_MODE_TXRX;
	USART2Handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_0_5;
	USART2Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART2Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;

	USART_PeriClockControl(USART2 , ENABLE);
	USART_Disable(USART2);

	USART2Handle.pUSARTx->USART_CR1 |= 12;
	while(1);

	//This function is used to initialize the SPI2 peripheral parameters
	USART2_Inits();

	uint8_t uart_txbuff[32] = {0};
	memset(&uart_txbuff,3,32);
	while(1)
	{
		sw_delay();
		USART_SendData(&USART2Handle, &uart_txbuff, 32);
		sw_delay();
	}

#if SWITCH_INIT
	/* Initialize Config Structure for PushButon */
	gpio_Pushbtn.pGPIO = GPIOC;
	gpio_Pushbtn.pGPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	gpio_Pushbtn.pGPIO_PinConfig.GPIO_PinMode = GPIO_PINMODE_IN;
	gpio_Pushbtn.pGPIO_PinConfig.GPIO_PinSpeed = GPIO_OUTSPEED_FAST;
	gpio_Pushbtn.pGPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NO_PUPD;
	gpio_Pushbtn.pGPIO_PinConfig.GPIO_PinOPType = GPIO_OT_OPENDRAIN;

	/* Enable Peripheral Clock*/
	GPIO_Pclk_Control(GPIOC,ENABLE);

	/* Init GPIO */
	GPIO_Init(&gpio_Pushbtn);
#endif

#if BTN_INIT
	/* Initialize Config Structure for PushButon */
	gpio_Pushbtn.pGPIO = GPIOC;
	gpio_Pushbtn.pGPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	gpio_Pushbtn.pGPIO_PinConfig.GPIO_PinMode = GPIO_PINMODE_IT_FT;
	gpio_Pushbtn.pGPIO_PinConfig.GPIO_PinSpeed = GPIO_OUTSPEED_FAST;
	gpio_Pushbtn.pGPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_PU;
	//led_gpio.pGPIO_PinConfig.GPIO_PinOPType = GPIO_OT_OPENDRAIN;

	/* Enable Peripheral Clock*/
	GPIO_Pclk_Control(GPIOC,ENABLE);

	/* Init GPIO */
	GPIO_Init(&gpio_Pushbtn);

	//IRQ Configurations
	GPIO_IRQHandler(GPIO_PIN_13); //clear pending INT
	GPIO_IRQ_Interrupt_Config(IRQ_NO_EXTI15_10,ENABLE);

#endif


	/* Infinit Loop */
	while(1)
	{

#if SWITCH_TEST
		uint8_t Pbtn = 0;
		printf(" %d \n ",GPIO_PinRead(GPIOC,GPIO_PIN_13));
		Pbtn = GPIO_PinRead(GPIOC,GPIO_PIN_13);
		if(Pbtn == 0)
		{
			sw_delay();
			printf("Pushbutton : ON \n");
			/* LED ON */
			GPIO_PinWrite(GPIOA,GPIO_PIN_5,ENABLE);
		}
		else
		{
			sw_delay();
			printf("Pushbutton : OFF \n");
			/* LED OFF */
			GPIO_PinWrite(GPIOA,GPIO_PIN_5,DISABLE);
		}

#endif

#if BTN_INT_TEST
		if(USER_KEY == false)
		{
			printf(" USER PUSHBUTTON PRESSED \n ");
			printf(" %d \n ",GPIO_PinRead(GPIOC,GPIO_PIN_13));
			USER_KEY = true;
			GPIO_PinToggle(GPIOA,GPIO_PIN_5);
			printf(" %d \n ",GPIO_PinRead(GPIOA,GPIO_PIN_5));
		}
#endif

	}//end of while(1)

	return 0;
}//end of main
