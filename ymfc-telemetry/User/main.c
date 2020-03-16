#include "stm32f10x.h"
#include "bsp_led.h"
#include "oled.h"

int main(void)
{
	LED_Init_Config();
	OLED_Init();
	OLED_Clear(0);
	OLED_Display_On();

	OLED_ShowString(1,2,"YMFC Telemetry", 16);
	OLED_ShowString(1,4,"0.96' OLED TEST", 16);
	Delay_ms(5000);

	while (1)
	{
//		OLED_Clear(0);
//
//		OLED_ShowString(6,3,"0.96' OLED TEST", 16);
//		OLED_ShowString(0,6,"ASCII:",16);
//		OLED_ShowString(63,6,"CODE:",16);

		Delay_ms(1000);
	}
}
