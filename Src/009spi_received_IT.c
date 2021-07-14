/*
 * 009SPI_received_IT.c
 *
 *  Created on: Jun 21, 2021
 *      Author: CHOCOPI
 */


#include<stdio.h>
#include<string.h>
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_gpio_driver.h"

SPI_Handle_t SPI2Handle;

#define MAX_LEN 500

char RcvBuff[MAX_LEN];

volatile char ReadByte;

volatile uint8_t rcvStop = 0;


volatile uint8_t dataAvailable = 0;

void delay()
{
	for(uint32_t i = 0; i < 500000/2; i++ );
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

void Slave_GPIO_InterruptPinInit(void)
{
	GPIO_Handle_t spiIntPin;
	memset(&spiIntPin, 0, sizeof(spiIntPin));

	//this is led gpio configuration
	spiIntPin.pGPIOx = GPIOD;
	spiIntPin.GPIO_PinConfig.GPIO_Pin = GPIO_PIN_NO_6;
	spiIntPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	spiIntPin.GPIO_PinConfig.GPIO_Pinspeed = GPIO_SPEED_LOW;
	spiIntPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&spiIntPin);

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, 15 );
	GPIO_IRQITConfig(IRQ_NO_EXTI9_5, ENABLE);
}

void SPI2_Inits(void)
{
	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;

	SPI_Init(&SPI2Handle);

	SPI_IRQInterruptConfig(IRQ_NO_SPI2, ENABLE);
}

int main()
{
	uint8_t dummy = 0xFF;
	SPI2_GPIOInits();

	Slave_GPIO_InterruptPinInit();

	SPI2_Inits();

	SPI_SSOEConfig(SPI2, ENABLE);

	while(1)
	{
		rcvStop = 0;

		while(!dataAvailable); //wait till the data available interrupt from transmitter device

		GPIO_IRQITConfig(IRQ_NO_EXTI9_5, DISABLE);

		//Enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		while(!rcvStop)
		{
			while( SPI_SendDataIT(&SPI2Handle,&dummy, 1) == SPI_BUSY_IN_TX);
			while( SPI_ReceiveDataIT(&SPI2Handle, (uint8_t*)&ReadByte, 1) == SPI_BUSY_IN_RX);
		}

		//Confirm SPI is not busy
		while( SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		//Disable the SPI2 periphral
		SPI_PeripheralControl(SPI2, DISABLE);

		printf("Rcvd data = %s \n",RcvBuff);

		dataAvailable = 0;

		GPIO_IRQITConfig(IRQ_NO_EXTI9_5, ENABLE);

	}

	return 0;
}

void EXTI9_5_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_6);
	dataAvailable = 1;
}

//Runs when a data byte is received from the peripheral over SPI
void SPI2_IRQHandler(void)
{
	SPI_IRQHandling(&SPI2Handle);
}

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	static uint32_t i = 0;
	if(AppEv == SPI_EVENT_RX_CMPLT)
	{
		RcvBuff[i++] = ReadByte;
		if(ReadByte == '\0' || MAX_LEN)
		{
			rcvStop = 1;
			RcvBuff[i-1] = '\0';
			i = 0;
		}
	}
}
