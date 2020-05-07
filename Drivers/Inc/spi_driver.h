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
#define SPI_BCFG_FD			1	//2-line unidirectional data mode selected
#define SPI_BCFG_HD			2	// 1-line bidirectional data mode selected
#define SPI_BCFG_SIMPLEX_RX	3	//SIMPLEX MODE (receive-only mode)
//#define SPI_BCFG_SIMPLEX_TX	4	//Output enabled (transmit-only mode)  NOT VALID
//below lines rep simplex mode not needed since mode added in SPI_BCFG_SIMPLEX_RX macro
//#define SPI_RXONLY_FULLDUPLEX	0	//Full duplex (Transmit and receive)
//#define SPI_RXONLY_OPDISABLED	1	//Output disabled (Receive-only mode)



/*  MACROs for SPI_ClkSpeed */
#define SPI_PCLK_DIV2		0
#define SPI_PCLK_DIV4		1
#define SPI_PCLK_DIV8		2
#define SPI_PCLK_DIV16		3
#define SPI_PCLK_DIV32		4
#define SPI_PCLK_DIV64		5
#define SPI_PCLK_DIV128		6
#define SPI_PCLK_DIV256		7

/*  MACROs for SPI_DFF */
#define SPI_DFF_8BIT	0	//8-bit data frame format
#define SPI_DFF_16BIT	1	//16-bit data frame format

/*  MACROs for SPI_SSM */
#define SPI_SSM_DIS	0	//Software slave management disabled
#define SPI_SSM_ENA 1	//Software slave management enabled


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


/* Register Bit Field MACROs */
#define SPI_CR1_CPHA_BPOS		0
#define SPI_CR1_CPOL_BPOS		1
#define SPI_CR1_MSTR_BPOS		2
#define SPI_CR1_BR_BPOS			3 //3,4,5
#define SPI_CR1_SPE_BPOS		6
#define SPI_CR1_LSBFIRST_BPOS	7
#define SPI_CR1_SSI_BPOS		8
#define SPI_CR1_SSM_BPOS		9
#define SPI_CR1_RX_ONLY_BPOS	10
#define SPI_CR1_DFF_BPOS		11
#define SPI_CR1_CRC_NEXT_BPOS	12
#define SPI_CR1_CRC_EN_BPOS		13
#define SPI_CR1_BIDI_OE_BPOS	14
#define SPI_CR1_BIDI_MODE_BPOS	15



void SPI_Pclk_Control(SPI_RegDef_t *pSPI,uint8_t ENorDIS);


void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPI);

void SPI_DataSend(SPI_RegDef_t *pSPI,uint8_t *pTx_Buff,uint32_t Len);
void SPI_DataREceive(SPI_RegDef_t *pSPI,uint8_t *pRx_Buff,uint32_t Len);


void SPI_IRQ_Interrupt_Config(uint8_t IRQNumber,uint8_t ENorDIS);
void SPI_IRQ_ITPriority(uint8_t IRQNumber,uint32_t IRQPriority);
void SPI_IRQHandler(SPI_RegDef_t *pSPI);


#endif /* INC_SPI_DRIVER_H_ */
