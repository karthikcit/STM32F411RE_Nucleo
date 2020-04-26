/*
 * test.c
 *
 *  Created on: 22-Apr-2020
 *      Author: KarthikM
 */


#include "test.h"


void Test_LED(void)
{
#if LED_TEST
		/* LED ON */
		GPIO_PinWrite(GPIOD,GPIO_PIN_13,ENABLE);
		//sw_delay();
		/* LED OFF */
		GPIO_PinWrite(GPIOD,GPIO_PIN_13,DISABLE);
		//sw_delay();
		/* LED TOGGLE */
		for(uint8_t li=0;li<10;li++)
		{
			GPIO_PinToggle(GPIOD,GPIO_PIN_13);
		}
#endif
}
