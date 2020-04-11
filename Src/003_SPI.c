/*
 * 003_SPI.c
 *
 *  Created on: Apr 3, 2020
 *      Author: dubey
 */
#include<STM32F429xx.h>
#include<string.h>
#include<GPIOx_Drivers.h>
#include<SPIx_Drivers.h>


#define HIGH 					1
#define BTN_PRESSED 			HIGH


void SPI_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);
	//MISO

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);
	//NSS

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

}
void SPI2_Init(void)
{
	SPI_Handle_t SPI2H;
	SPI2H.pSPIx = SPI2;
	SPI2H.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2H.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2H.SPIConfig.SPI_CLKSpeed = SPI_CLK_SPEED_DIV8;
	SPI2H.SPIConfig.SPI_DFF = SPI_DFF_8Bits;
	SPI2H.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2H.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2H.SPIConfig.SPI_SSM = SPI_SSM_DI;
	SPI_Init(&SPI2H);

}

void BTN_Init()
{
	GPIO_Handle_t GpioBtn;
	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode 		= GPIO_MODE_INPUT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_PIN_PD;

	GPIO_PCLK_Control(GPIOA, ENABLE);

	GPIO_Init(&GpioBtn);
}
void LED_Init(void)
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
}
void delay(void)
{
	for(uint32_t i = 0; i < 500000; i++);
}
int main(void)
{
	char *data = "Hello";
	SPI_GPIOInits();
	SPI2_Init();
	SPI_SSOEConfig(SPI2, ENABLE);

	BTN_Init();
	LED_Init();

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == BTN_PRESSED)
		{
			delay();
			SPI_PeriphControl(SPI2, ENABLE);
			uint8_t DataLen = strlen(data);
			SPI_TransmitData(SPI2, &data, 1);
			GPIO_ToggleOutputPin(GPIOG, GPIO_PIN_NO_13);
			SPI_TransmitData(SPI2, (uint8_t*)data, strlen(data));
		}
	}

	return 0;
}
