/*
 * 007spi_stm32_to_esp8266.c
 *
 *  Created on: Jun 18, 2021
 *      Author: CHOCOPI
 */
#include"stm32f407xx.h"
#include"stm32f407xx_spi_driver.h"
#include"stm32f407xx_gpio_driver.h"
#include<string.h>

void delay(void)
{
	for(uint32_t i=0; i<500000; i++);
}

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_Pinspeed = GPIO_SPEED_FAST;

	//SCKL
	SPIPins.GPIO_PinConfig.GPIO_Pin = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_Pin = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_Pin = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_Pin = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI; //Hardware slave management enabled for NSS

	SPI_Init(&SPI2handle);

}

void GPIO_ButtonInit()
{
	GPIO_Handle_t GPIOBtn;

	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_Pin = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_Pinspeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);
}

int main()
{
	char user_data[] = "Hello World";

	GPIO_ButtonInit();

	SPI2_GPIOInits();

	SPI2_Inits();

	SPI_SSOEConfig(SPI2, ENABLE);

	//SPI_SSIConfig(SPI2, ENABLE);

	//SPI_PeripheralControl(SPI2, ENABLE);

	//SPI_SendData(SPI2 , (uint8_t*)user_data, strlen(user_data));
	while(1)
	{
		while( !GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) );

		//to avoid button debouncing
		delay();

		//enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		//to send data
		SPI_SendData(SPI2 , (uint8_t*)user_data, strlen(user_data));

		//Lets confirm SPI is not busy
		while( SPI_GetFlagStatus( SPI2, SPI_BUSY_FLAG ) ) ;

		//Disable the SPI2 periphrals
		SPI_PeripheralControl(SPI2, DISABLE);

	}
	return 0;

}
