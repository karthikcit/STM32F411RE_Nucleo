/*
 * spi_driver.c
 *
 *  Created on: 06-May-2020
 *      Author: KarthikM
 */


#include "spi_driver.h"
//#include "rcc_clock.h"



void SPI_Pclk_Control(SPI_RegDef_t *pSPI,uint8_t ENorDIS)
{
	if(ENorDIS == ENABLE)
	{
		if(pSPI == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPI == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPI == SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if(pSPI == SPI4)
		{
			SPI4_PCLK_EN();
		}
		else if(pSPI == SPI5)
		{
			SPI5_PCLK_EN();
		}
	}
	else
	{
		if(pSPI == SPI1)
		{
			SPI1_PCLK_DIS();
		}
		else if(pSPI == SPI2)
		{
			SPI2_PCLK_DIS();
		}
		else if(pSPI == SPI3)
		{
			SPI3_PCLK_DIS();
		}
		else if(pSPI == SPI4)
		{
			SPI4_PCLK_DIS();
		}
		else if(pSPI == SPI5)
		{
			SPI5_PCLK_DIS();
		}
	}
}


void SPI_Init(SPI_Handle_t *pSPIHandle)
{

}

/*
void SPI_DeInit(SPI_RegDef_t *pSPI)
{

}

void SPI_DataSend(SPI_RegDef_t *pSPI,uint8_t *pTx_Buff,uint32_t Len)
{

}

void SPI_DataREceive(SPI_RegDef_t *pSPI,uint8_t *pRx_Buff,uint32_t Len)
{

}


void SPI_IRQ_Interrupt_Config(uint8_t IRQNumber,uint8_t ENorDIS)
{

}

void SPI_IRQ_ITPriority(uint8_t IRQNumber,uint32_t IRQPriority)
{

}


void SPI_IRQHandler(SPI_RegDef_t *pSPI)
{

}


*/
