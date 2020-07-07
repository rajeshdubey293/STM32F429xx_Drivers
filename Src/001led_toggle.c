/*
 * 001led_toggle.c
 *
 *  Created on: Feb 1, 2019
 *      Author: admin
 */

#include "main.h"




int main(void)
{

	SysTick->LOAD = 3200000-1;
	SysTick->VAL = 0;
	SysTick->CTRL = 5;


	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOG;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PCLK_Control(GPIOG,ENABLE);

	GPIO_Init(&GpioLed);

	while(1)
	{
		if(SysTick->CTRL & 0x10000)
		{
			GPIO_ToggleOutputPin(GPIOG,GPIO_PIN_NO_14);
		}

	}
	return 0;
}
