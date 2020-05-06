/*
 * spi_driver.h
 *
 *  Created on: 06-May-2020
 *      Author: KarthikM
 */

#ifndef INC_SPI_DRIVER_H_
#define INC_SPI_DRIVER_H_

#include "stm32f411xx.h"

typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_ClkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPHA;
	uint8_t SPI_CPOL;
	uint8_t SPI_SSM;

}SPI_PinConfig_t;


/* Create SPI Handle Structure */
typedef struct
{
	//GPIOA_RegDef_t *pGPIOA = (GPIOA_RegDef_t*)GPIOA_BASEADDR;  //specific
	SPI_RegDef_t *pSPI;  /* common */
	SPI_PinConfig_t pSPI_PinConfig;
}SPI_Handle_t;


void SPI_Pclk_Control(SPI_RegDef_t *pSPI,uint8_t ENorDIS);


void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPI);

void SPI_DataSend(SPI_RegDef_t *pSPI,uint8_t *pTx_Buff,uint32_t Len);
void SPI_DataREceive(SPI_RegDef_t *pSPI,uint8_t *pRx_Buff,uint32_t Len);


void SPI_IRQ_Interrupt_Config(uint8_t IRQNumber,uint8_t ENorDIS);
void SPI_IRQ_ITPriority(uint8_t IRQNumber,uint32_t IRQPriority);
void SPI_IRQHandler(SPI_RegDef_t *pSPI);


#endif /* INC_SPI_DRIVER_H_ */
