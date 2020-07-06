/*
 * main_spi.c
 *
 *  Created on: 05-Jun-2020
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
#include "test.h"

/* master read/write buffers */
uint8_t master_write_data[]={ 0xa, 0xb, 0xc, 0xd };

uint8_t master_read_buffer[4];


void sw_delay(void)
{
	for(uint32_t i=0;i<5000;i++);
}

#if 1
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
	//SPIPins.pGPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	//GPIO_Init(&SPIPins);


	//NSS
	SPIPins.pGPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&SPIPins);


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
#else
void SPI2GPIO_Init()
{
	GPIO_Handle_t SPI2_gpio;
	memset(&SPI2_gpio,0,sizeof(SPI2_gpio));

	/* Enable Peripheral Clock*/
	GPIO_Pclk_Control(GPIOB,ENABLE);

	SPI2_gpio.pGPIO = GPIOB;
	SPI2_gpio.pGPIO_PinConfig.GPIO_PinMode = GPIO_PINMODE_ALTFN;
	SPI2_gpio.pGPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPI2_gpio.pGPIO_PinConfig.GPIO_PinSpeed = GPIO_OUTSPEED_MED;
	SPI2_gpio.pGPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_PU;//GPIO_PUPD_NO_PUPD;
	SPI2_gpio.pGPIO_PinConfig.GPIO_PinOPType = GPIO_OT_PUSHPULL;


	//Slave Select Pin
	//SPI2_gpio.pGPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	//GPIO_Init(&SPI2_gpio);/* Init GPIO */

	//Sclk Pin
	SPI2_gpio.pGPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&SPI2_gpio);/* Init GPIO */

	//MISO
	//SPI2_gpio.pGPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	//GPIO_Init(&SPI2_gpio);/* Init GPIO */

	//MOSI
	SPI2_gpio.pGPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&SPI2_gpio);/* Init GPIO */

}

void SPI2_Init()
{
	SPI_Handle_t peri_spi2;
	memset(&peri_spi2,0,sizeof(peri_spi2));

	//SPI 2 as master
	peri_spi2.pSPI_PinConfig.SPI_BusConfig = SPI_BCFG_FD;
	peri_spi2.pSPI_PinConfig.SPI_DeviceMode = SPI_DEV_MODE_MASTER;
	peri_spi2.pSPI_PinConfig.SPI_ClkSpeed = SPI_PCLK_DIV4;  //Fclk/2
	peri_spi2.pSPI_PinConfig.SPI_DFF = SPI_DFF_16BIT;
	peri_spi2.pSPI_PinConfig.SPI_SSM = SPI_SSM_ENA;
	peri_spi2.pSPI_PinConfig.SPI_CPHA = SPI_CPHA_ONE;
	peri_spi2.pSPI_PinConfig.SPI_CPOL = SPI_CPHA_ONE;

	//enable pclk
	SPI_Pclk_Control(SPI2,ENABLE);

	//INIT SPI
	SPI_Init(&peri_spi2);

}
#endif
int main(void)
{
	printf("hello world\n");

#if 0
	SPI2GPIO_Init();
	SPI_Pclk_Control(SPI2,ENABLE);//spi clk en

	//SPI2_Init();
	SPI2->SPI_CR1 &= ~(SET << SPI_CR1_SPE_BPOS);//disable spi

	uint32_t tempreg_val=0;
	tempreg_val |= SPI_CPHA_ZERO << SPI_CR1_CPHA_BPOS;  //clk phase
	tempreg_val |= SPI_CPOL_ZERO << SPI_CR1_CPOL_BPOS;	//clk polarity
	tempreg_val |= (SET << SPI_CR1_MSTR_BPOS); //dev mode : SPI_MASTER
	tempreg_val |= SPI_PCLK_DIV2 << SPI_CR1_BR_BPOS;	//clk speed
	tempreg_val |= SPI_DFF_8BIT << SPI_CR1_DFF_BPOS;	//frame format
	tempreg_val |= SPI_SSM_DIS << SPI_CR1_SSM_BPOS;		//sw slave mngmnt
	//set BIDI MODE bit
	tempreg_val &= ~(SET << SPI_CR1_BIDI_MODE_BPOS);
	tempreg_val |= (SET <<SPI_CR1_BIDI_OE_BPOS);
	SPI2->SPI_CR1 |= tempreg_val;

	//SPI2->SPI_CR1 = 0;
	//SPI2->SPI_CR1 |= 0x8204;


/*	if(pSPIHandle->pSPI_PinConfig.SPI_DeviceMode == SPI_BCFG_FD)
	{
		//clear BIDI MODE bit
		tempreg_val |= ~(SET << SPI_CR1_BIDI_MODE_BPOS);

	}
	else if(pSPIHandle->pSPI_PinConfig.SPI_DeviceMode == SPI_BCFG_HD)
	{
		//set BIDI MODE bit
		tempreg_val |= (SET << SPI_CR1_BIDI_MODE_BPOS);
	}
	else if(pSPIHandle->pSPI_PinConfig.SPI_DeviceMode == SPI_BCFG_SIMPLEX_RX)
	{
		//clear BIDI MODE bit
		tempreg_val &= ~(SET << SPI_CR1_BIDI_MODE_BPOS);
		tempreg_val |= (SET << SPI_CR1_RX_ONLY_BPOS);  //rx only mode
	}
*/

	SPI2->SPI_CR2 |= SPI_CR2_SSOE;  //Slav Sel O/p Enable

	SPI_SSI_SET(SPI2);
	SPI2->SPI_CR1 |= (SET << SPI_CR1_SPE_BPOS);//enable spi
	printf("CR1 : 0x%X \n",SPI2->SPI_CR1);


	while(1){
	SPI2->SPI_DR |= 0x0f;//write data t0 Data Reg
	}
	while(1);
#endif


	SPI2_GPIOInits();
	SPI2_Inits();
	SPI_SSOE_Enable(SPI2);

	uint8_t test_str[32] = {0};
	uint8_t i=0;
	//memset(&test_str,7,32);
	for(i=0;i<32;i++)
	{
		test_str[i]=i;
	}

	/* Infinit Loop */
	while(1)
	{
		//SPI_DataSend(SPI2,test_str,32);
		uint32_t Len = 32,stat=0;
		uint32_t dread[1]={0};
		i = 0;
		while(Len > 0)
			{
				//1. check TXE flag to set
				while(!(SPI2->SPI_SR & (1<<1)));
				//2. check frame formate
				if(((SPI2->SPI_CR1) & (1<<SPI_CR1_DFF_BPOS))  == SPI_DFF_8BIT)
				{
					//3. put data in DR
					SPI2->SPI_DR |= 0x0f;//*((uint8_t*)test_str);
					//4. increament TX buff addr
					*test_str = *test_str + 1;
					//i++;
					//5. Decrement Len
					Len--;
				}
				else if((SPI2->SPI_CR1 & (1<<SPI_CR1_DFF_BPOS))  == SPI_DFF_16BIT)
				{
					//3. put data in DR
//					SPI2->SPI_DR = *((uint16_t*)test_str);
					SPI2->SPI_DR |= test_str[i];
					//4. increament TX buff addr
					//(uint16_t*)test_str++;
					i++;
					//5. Decrement Len
					Len--;
					Len--;
				}
			}
	}//end of while(1)

	return 0;
}//end of main

#if 0
		SPI_Enable(SPI2);
		//first send length information
		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI2,&dataLen,1);

		//to send data
		SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));

		//lets confirm SPI is not busy
		while( SPI_GetFlagStatus(SPI2,SPI_SR_BUSY_FLAG) );

		//Disable the SPI2 peripheral
		SPI_Disable(SPI2);
#endif
