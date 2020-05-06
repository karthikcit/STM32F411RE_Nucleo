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
	SPI_RegDef_t *pSPI;  /* common */
	SPI_PinConfig_t pSPI_PinConfig;
}SPI_Handle_t;

/*  MACROs for SPI_DeviceMode */
#define SPI_DEV_MODE_MASTER 1
#define SPI_DEV_MODE_SLAVE  0

/*  MACROs for SPI_BusConfig */
#define SPI_BIDIMODE_UNIDIR	0	//2-line unidirectional data mode selected
#define SPI_BIDIMODE_BIDIR	1	// 1-line bidirectional data mode selected
#define SPI_BIDIOE_DISABLE	0	//Output disabled (receive-only mode)
#define SPI_BIDIOE_ENABLE	1	//Output enabled (transmit-only mode)

/*  MACROs for SPI_ClkSpeed */

/*  MACROs for SPI_DFF */
#define SPI_DFF_8BIT	0	//8-bit data frame format
#define SPI_DFF_16BIT	1	//16-bit data frame format

/*  MACROs for SPI_SSM */
#define SPI_SSM_DIS	0	//Software slave management disabled
#define SPI_SSM_ENA 1	//Software slave management enabled

#define SPI_RXONLY_FULLDUPLEX	0	//Full duplex (Transmit and receive)
#define SPI_RXONLY_OPDISABLED	1	//Output disabled (Receive-only mode)

/*  SPI Perpheral Enable */
#define SPI_ENABLE		1
#define SPI_DISABLE		0

/* BAUD RATE CONTROL */
/* SPI MASTER Selection  */
#define SPI_SLAVE	0
#define SPI_MASTER	1

/*  MACROs for SPI_CPOl */
#define SPI_CPOL_ZERO	0
#define SPI_CPOLONE		1

/*  MACROs for SPI_CPHA */
#define SPI_CPHA_ZERO	0	//The first clock transition is the first data capture edge
#define SPI_CPHA_ONE	1	//The second clock transition is the first data capture edge


void SPI_Pclk_Control(SPI_RegDef_t *pSPI,uint8_t ENorDIS);


void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPI);

void SPI_DataSend(SPI_RegDef_t *pSPI,uint8_t *pTx_Buff,uint32_t Len);
void SPI_DataREceive(SPI_RegDef_t *pSPI,uint8_t *pRx_Buff,uint32_t Len);


void SPI_IRQ_Interrupt_Config(uint8_t IRQNumber,uint8_t ENorDIS);
void SPI_IRQ_ITPriority(uint8_t IRQNumber,uint32_t IRQPriority);
void SPI_IRQHandler(SPI_RegDef_t *pSPI);


#endif /* INC_SPI_DRIVER_H_ */
