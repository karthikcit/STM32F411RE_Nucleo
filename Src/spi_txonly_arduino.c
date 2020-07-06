/*
 * spi_txonly_arduino.c
 *
 *  Created on: 01-Jul-2020
 *      Author: KarthikM
 */


/*
 * 006spi_tx_testing.c
 *
 *  Created on: Feb 10, 2019
 *      Author: admin
 */

//#include<string.h>
//#include "stm32f407xx.h"

#include "stdio.h"
#include "stdbool.h"
#include "string.h"
#include "stm32f411xx.h"
#include "gpio_driver.h"
#include "spi_driver.h"
#include "test.h"

#define USE_CHAR_BUFF	0

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 -> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT function mode : 5
 */

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	GPIO_Pclk_Control(GPIOB,ENABLE);
	SPIPins.pGPIO = GPIOB;

	SPIPins.pGPIO_PinConfig.GPIO_PinMode = GPIO_PINMODE_ALTFN;//GPIO_MODE_ALTFN;
	SPIPins.pGPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.pGPIO_PinConfig.GPIO_PinOPType = GPIO_OT_PUSHPULL;//GPIO_OP_TYPE_PP;
	SPIPins.pGPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_PU;//GPIO_PIN_PU;
	SPIPins.pGPIO_PinConfig.GPIO_PinSpeed = GPIO_OUTSPEED_MED;//GPIO_SPEED_FAST;

	//SCLK
	SPIPins.pGPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.pGPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.pGPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&SPIPins);


	//NSS
	SPIPins.pGPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&SPIPins);

	//SPIPins.pGPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	//GPIO_Init(&SPIPins);

}

void SPI2_Inits(void)
{

	SPI_Handle_t SPI2handle;

	SPI_Pclk_Control(SPI2,ENABLE);//spi clk en

	SPI2handle.pSPI_PinConfig.SPI_BusConfig = SPI_BCFG_FD;
	SPI2handle.pSPI = SPI2;
	//SPI2handle.pSPIx = SPI2;
	//SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.pSPI_PinConfig.SPI_DeviceMode = SPI_DEV_MODE_MASTER;
	SPI2handle.pSPI_PinConfig.SPI_ClkSpeed = SPI_PCLK_DIV8;//generates sclk of 2MHz
	SPI2handle.pSPI_PinConfig.SPI_DFF = SPI_DFF_8BIT;
	SPI2handle.pSPI_PinConfig.SPI_CPOL = SPI_CPOL_ZERO;
	SPI2handle.pSPI_PinConfig.SPI_CPHA = SPI_CPHA_ZERO;
	SPI2handle.pSPI_PinConfig.SPI_SSM = SPI_SSM_DIS; //Hardware slave management enabled for NSS pin

	SPI_Init(&SPI2handle);
}

void GPIO_ButtonInit(void)
{
#if 0
	GPIO_Handle_t GPIOBtn;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);
#else
	GPIO_Handle_t GpioButton;
	GpioButton.pGPIO = GPIOC;
	GpioButton.pGPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GpioButton.pGPIO_PinConfig.GPIO_PinMode = GPIO_PINMODE_IN;//GPIO_PINMODE_IT_FT;
	GpioButton.pGPIO_PinConfig.GPIO_PinSpeed = GPIO_OUTSPEED_FAST;
	GpioButton.pGPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_PU;

	GPIO_Pclk_Control(GPIOC,ENABLE);
	GPIO_Init(&GpioButton);

	//GPIO_IRQ_Interrupt_Config(IRQ_NO_EXTI15_10, ENABLE);
#endif
}


int main(void)
{
	printf("SPI MASTER CODE\n");
#if USE_CHAR_BUFF
	char user_data[] = "stm32f4 nucleo - Spi Master Board ";
#else
	uint8_t test_str[32] = {0};
	uint8_t i=0,Length=32;
	for(i=0;i<32;i++)
	{
		test_str[i]=i;
	}
#endif

	GPIO_ButtonInit();

	//this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	//This function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	/*
	* making SSOE 1 does NSS output enable.
	* The NSS pin is automatically managed by the hardware.
	* i.e when SPE=1 , NSS will be pulled to low
	* and NSS pin will be high when SPE=0
	*/
	SPI_SSOE_Enable(SPI2);

	while(1)
	{
		//wait till button is pressed
		while(  GPIO_PinRead(GPIOC,GPIO_PIN_13) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		//enable the SPI2 peripheral
		SPI_Enable(SPI2);


#if USE_CHAR_BUFF
		//first send length information
		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI2,&dataLen,1);
		//to send data
		SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));
#else
		//first send length information
		SPI_SendData(SPI2,&Length,1);
		//to send data
		SPI_SendData(SPI2,(uint8_t*)test_str,Length);
#endif
		//lets confirm SPI is not busy
		while( SPI_GetFlagStatus(SPI2,SPI_SR_BUSY_FLAG) );

		//Disable the SPI2 peripheral
		SPI_Disable(SPI2);
	}

	return 0;

}
