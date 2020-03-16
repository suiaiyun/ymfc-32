#ifndef __BSP_LED_H
#define __BSP_LED_H

#include "../../FWLIB/CMSIS/stm32f10x.h"

#define LED_P_GPIO_PORT 	GPIOC
#define LED_P_GPIO_PIN 		GPIO_Pin_13
#define LED_P_GPIO_CLK 		RCC_APB2Periph_GPIOC

#define ON 			1
#define OFF 		0

#define LED_G(sw)	if (sw) \
						 GPIO_ResetBits(GPIOC, GPIO_Pin_13); \
					else GPIO_SetBits(GPIOC, GPIO_Pin_13);

#define LED_G_TOGGLE {LED_P_GPIO_PORT->ODR ^= LED_P_GPIO_PIN;}

void LED_Init_Config(void);

#endif
