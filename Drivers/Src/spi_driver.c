/*
 * spi_driver.c
 *
 *  Created on: 06-May-2020
 *      Author: KarthikM
 */


#include "spi_driver.h"


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

void SPI_Enable(SPI_RegDef_t *pSPI)
{
	pSPI->SPI_CR1 |= (SET << SPI_CR1_SPE_BPOS);
}

void SPI_Disable(SPI_RegDef_t *pSPI)
{
	pSPI->SPI_CR1 &= ~(SET << SPI_CR1_SPE_BPOS);
}

void SPI_SSI_SET(SPI_RegDef_t *pSPI)
{
	pSPI->SPI_CR1 |= SET << SPI_CR1_SSI_BPOS;
}


void SPI_SSOE_Enable(SPI_RegDef_t *pSPI)
{
	pSPI->SPI_CR2 |= SPI_CR2_SSOE;
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPI , uint32_t FlagName)
{
	if(pSPI->SPI_SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	uint32_t tempreg_val=0;

	SPI_Disable(pSPIHandle);

	tempreg_val |= pSPIHandle->pSPI_PinConfig.SPI_CPHA << SPI_CR1_CPHA_BPOS;  //clk phase
	tempreg_val |= pSPIHandle->pSPI_PinConfig.SPI_CPOL << SPI_CR1_CPOL_BPOS;	//clk polarity
	if(pSPIHandle->pSPI_PinConfig.SPI_DeviceMode == SPI_DEV_MODE_MASTER)
	{
		tempreg_val |= (SET << SPI_CR1_MSTR_BPOS); //dev mode : SPI_MASTER
	}
	else if(pSPIHandle->pSPI_PinConfig.SPI_DeviceMode == SPI_DEV_MODE_SLAVE)
	{
		tempreg_val &=  ~(SET << SPI_CR1_MSTR_BPOS); //dev mode : SPI_SLAVE
	}

	tempreg_val |= pSPIHandle->pSPI_PinConfig.SPI_ClkSpeed << SPI_CR1_BR_BPOS;	//clk speed
	tempreg_val |= pSPIHandle->pSPI_PinConfig.SPI_DFF << SPI_CR1_DFF_BPOS;		//frame format
	tempreg_val |= pSPIHandle->pSPI_PinConfig.SPI_SSM << SPI_CR1_SSM_BPOS;		//sw slave mngmnt


	if(pSPIHandle->pSPI_PinConfig.SPI_DeviceMode == SPI_BCFG_FD)
	{
		//clear BIDI MODE bit
		tempreg_val &= ~(SET << SPI_CR1_BIDI_MODE_BPOS);

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

	//feed tempreg_val to CR1 reg
	pSPIHandle->pSPI->SPI_CR1 |= tempreg_val;

	//enable SPI
	//SPI_Enable(pSPIHandle);

}


void SPI_DeInit(SPI_RegDef_t *pSPI)
{
	SPI_Pclk_Control(pSPI,DISABLE);
}


uint32_t SPI_RegRead_SR(SPI_RegDef_t *pSPI)
{
	uint32_t value=0;
	value = pSPI->SPI_SR;
	return value;

}

uint32_t SPI_RegRead_DR(SPI_RegDef_t *pSPI)
{
	uint32_t value=0;
	value = pSPI->SPI_DR;
	return value;

}

void SPI_RegWrite_SR(SPI_RegDef_t *pSPI,uint32_t value)
{

}

/* Blocking or Polling */
void SPI_DataSend(SPI_RegDef_t *pSPI,uint8_t *pTx_Buff,uint32_t Len)
{
	while(Len > 0)
	{
		//1. check TXE flag to set
		while(!(pSPI->SPI_SR & (1<<1)));
		//2. check frame formate
		if(((pSPI->SPI_CR1) & (1<<SPI_CR1_DFF_BPOS))  == SPI_DFF_8BIT)
		{
			//3. put data in DR
			pSPI->SPI_DR |= *(pTx_Buff);
			//4. increament TX buff addr
			pTx_Buff++;
			//5. Decrement Len
			Len--;
		}
		else if((pSPI->SPI_CR1 & (1<<SPI_CR1_DFF_BPOS))  == SPI_DFF_16BIT)
		{
			//3. put data in DR
			pSPI->SPI_DR = *((uint16_t*)pTx_Buff);
			//4. increament TX buff addr
			(uint16_t*)pTx_Buff++;
			//5. Decrement Len
			Len--;
			Len--;
		}
	}
}

/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - This is blocking call

 */
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_SR_TXE_FLAG)  == FLAG_RESET );

		//2. check the DFF bit in CR1
		if( (pSPIx->SPI_CR1 & ( 1 << SPI_CR1_DFF_BPOS) ) )
		{
			//16 bit DFF
			//1. load the data in to the DR
			pSPIx->SPI_DR =   *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else
		{
			//8 bit DFF
			pSPIx->SPI_DR =   *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}

}


/*
void SPI_DataREceive(SPI_RegDef_t *pSPI,uint8_t *pRx_Buff,uint32_t Len)
{

}


void SPI_IRQ_Interrupt_Config(uint8_t IRQNumber,uint8_t ENorDIS)
{

}

void SPI_IRQ_ITPriority(uint8_t IRQNumber,uint32_t IRQPriority)
{

}

*/

void SPI_IRQHandler(SPI_RegDef_t *pSPI)
{
	  uint32_t tmp1 = 0, tmp2 = 0;

	/* check to see RXNE is set in the status register */
	//tmp1 = (hspi->Instance->SR & SPI_REG_SR_RXNE_FLAG);
	tmp1 = (pSPI->SPI_SR & SPI_SR_RXNE_FLAG);
	/* check whether RXNEIE bit is enabled in the control register. */
	//tmp2 = (hspi->Instance->CR2 & SPI_REG_CR2_RXNEIE_ENABLE);
	tmp2 = (pSPI->SPI_CR2 & SPI_CR2_RXNEIE_ENABLE);
	if((tmp1 != RESET) && (tmp2 != RESET) )
	  {
		//Rx int handle
	  }
	/* check to see TXE is set in the status register */
	//tmp1 = (hspi->Instance->SR & SPI_REG_SR_TXE_FLAG);
	tmp1 = (pSPI->SPI_SR & SPI_SR_TXE_FLAG);
	//tmp2 = (hspi->Instance->CR2 & SPI_REG_CR2_TXEIE_ENABLE);
	tmp2 = (pSPI->SPI_CR2 & SPI_CR2_TXEIE_ENABLE);

	if((tmp1 != RESET) && (tmp2 != RESET))
	{
		/* TXE flag is set handle the TX of data bytes */
	     //hal_spi_handle_tx_interrupt(hspi);
	    return;
	  }

}



