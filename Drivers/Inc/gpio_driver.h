/*
 * gpio_driver.h
 *
 *  Created on: 18-Apr-2020
 *      Author: KarthikM
 */

#ifndef INC_GPIO_DRIVER_H_
#define INC_GPIO_DRIVER_H_

#include "stm32f411xx.h"
#include "rcc_clock.h"

typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;


/* Create GPIO Handle Structure */
typedef struct
{
	//GPIOA_RegDef_t *pGPIOA = (GPIOA_RegDef_t*)GPIOA_BASEADDR;  //specific
	GPIO_RegDef_t *pGPIO;  /* common */
	GPIO_PinConfig_t pGPIO_PinConfig;
}GPIO_Handle_t;


/* MACRO for GPIO Pin Numbers */
#define GPIO_PIN_0		0
#define GPIO_PIN_1		1
#define GPIO_PIN_2		2
#define GPIO_PIN_3		3
#define GPIO_PIN_4		4
#define GPIO_PIN_5		5
#define GPIO_PIN_6		6
#define GPIO_PIN_7		7
#define GPIO_PIN_8		8
#define GPIO_PIN_9		9
#define GPIO_PIN_10 	10
#define GPIO_PIN_11 	11
#define GPIO_PIN_12 	12
#define GPIO_PIN_13 	13
#define GPIO_PIN_14 	14
#define GPIO_PIN_15 	15

/* MACROs for Probable Pin Related Register Bit values */
#define GPIO_PINMODE_IN			0
#define GPIO_PINMODE_OUT		1
#define GPIO_PINMODE_ALTFN		2
#define GPIO_PINMODE_AN			3


/* MACROs for Probable Pin Related Register Bit values */
#define GPIO_PINMODE_IN			0
#define GPIO_PINMODE_OUT		1
#define GPIO_PINMODE_ALTFN		2
#define GPIO_PINMODE_AN			3





/* MACRO for GPIO Pin Numbers */
#define GPIO_PIN_0		0
#define GPIO_PIN_1		1
#define GPIO_PIN_2		2
#define GPIO_PIN_3		3
#define GPIO_PIN_4		4
#define GPIO_PIN_5		5
#define GPIO_PIN_6		6
#define GPIO_PIN_7		7
#define GPIO_PIN_8		8
#define GPIO_PIN_9		9
#define GPIO_PIN_10 	10
#define GPIO_PIN_11 	11
#define GPIO_PIN_12 	12
#define GPIO_PIN_13 	13
#define GPIO_PIN_14 	14
#define GPIO_PIN_15 	15

/* MACROs for Probable Pin Related Register Bit values */
#define GPIO_PINMODE_IN			0
#define GPIO_PINMODE_OUT		1
#define GPIO_PINMODE_ALTFN		2
#define GPIO_PINMODE_AN			3
#define GPIO_PINMODE_IT_FT		4
#define GPIO_PINMODE_IT_RT		5
#define GPIO_PINMODE_IT_RFT		6

#define GPIO_OUTSPEED_LOW		0
#define GPIO_OUTSPEED_MED		1
#define GPIO_OUTSPEED_FAST		2
#define GPIO_OUTSPEED_HIGH		3

#define GPIO_PUPD_NO_PUPD		0
#define GPIO_PUPD_PU			1
#define GPIO_PUPD_PD			2

/* GPIO output type */
#define GPIO_OT_PUSHPULL	0
#define GPIO_OT_OPENDRAIN	1


/* Function Prototypes  */
/*
 * for clk enable
 * param1 : base addr pointer of peripheral
 * param2 : En/Disable ip
 * return :
 * */
void GPIO_Pclk_Control(GPIO_RegDef_t *pGPIO,uint8_t ENorDIS);


/*
 * param1 : Handle structure ptr
 *
 * */
//void GPIO_Init(GPIO_RegDef_t *pGPIO,GPIO_PinConfig_t pGPIO_PinConfig);  //these 2 params are inside ourhandle struct so v can use that as single param
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIO);

uint8_t GPIO_PinRead(GPIO_RegDef_t *pGPIO,uint8_t PinNumber);
uint16_t GPIO_PortRead(GPIO_RegDef_t *pGPIO);
void GPIO_PinWrite(GPIO_RegDef_t *pGPIO,uint8_t PinNumber,uint8_t value);
void GPIO_PortWrite(GPIO_RegDef_t *pGPIO,uint16_t value);
void GPIO_PinToggle_ByRead(GPIO_RegDef_t *pGPIO,uint8_t PinNumber);
void GPIO_PinToggle(GPIO_RegDef_t *pGPIO,uint8_t PinNumber);


void GPIO_IRQ_Interrupt_Config(uint8_t IRQNumber,uint8_t ENorDIS);
void GPIO_IRQ_ITPriority(uint8_t IRQNumber,uint32_t IRQPriority);
void GPIO_IRQHandler(uint8_t PinNumbers);
#endif /* INC_GPIO_DRIVER_H_ */
