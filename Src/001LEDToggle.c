/*
 * 001LEDToggle.c
 *
 *  Created on: 14-Mar-2020
 *      Author: vicky
 */
#include"GPIOx_Drivers.h"

void delay(void)
{
	for(uint32_t i = 0; i < 100000; i++);
}

int main()
{
	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx = GPIOG;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_13;
	GpioLed.GPIO_PinConfig.GPIO_PinMode 		= GPIO_MODE_OUTPUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OUT_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_NO_PUPD;

	GPIO_PCLK_Control(GPIOG, ENABLE);

	GPIO_Init(&GpioLed);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOG, GPIO_PIN_NO_13);
		delay();
	}

	return 0;
}
