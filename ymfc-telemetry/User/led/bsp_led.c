#include "bsp_led.h"

void LED_Init_Config(void)
{
	RCC_APB2PeriphClockCmd(LED_P_GPIO_CLK, ENABLE);

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = LED_P_GPIO_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(LED_P_GPIO_PORT, &GPIO_InitStruct);
}
