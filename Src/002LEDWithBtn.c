/*
 * 002LEDWithBtn.c
 *
 *  Created on: 14-Mar-2020
 *      Author: vicky
 */

#include"GPIOx_Drivers.h"

#define HIGH 					1
#define BTN_PRESSED 			HIGH

void delay(void)
{
	for(uint32_t i = 0; i < 100000; i++);
}

int main()
{
	GPIO_Handle_t GpioLed, GpioBtn;
	GpioLed.pGPIOx = GPIOG;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_13;
	GpioLed.GPIO_PinConfig.GPIO_PinMode 		= GPIO_MODE_OUTPUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OUT_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_NO_PUPD;

	GPIO_PCLK_Control(GPIOG, ENABLE);

	GPIO_Init(&GpioLed);

	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode 		= GPIO_MODE_INPUT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_PIN_PD;

	GPIO_PCLK_Control(GPIOA, ENABLE);

	GPIO_Init(&GpioBtn);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == BTN_PRESSED)
		{
			GPIO_ToggleOutputPin(GPIOG, GPIO_PIN_NO_13);
		}
	}

	return 0;
}

