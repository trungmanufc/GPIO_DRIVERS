/*
 * 008spi_cmd_handling.c
 *
 *  Created on: Jun 19, 2021
 *      Author: CHOCOPI
 */
#include"stm32f407xx.h"
#include"stm32f407xx_spi_driver.h"
#include"stm32f407xx_gpio_driver.h"
#include<string.h>

//command codes
#define COMMAND_LED_CTRL		0x50
#define COMMAND_SENSOR_READ		0x51
#define COMMAND_LED_READ		0x52
#define COMMAND_PRINT			0x53
#define COMMAND_ID_READ			0x54

#define LED_ON		1
#define LED_OFF		0

//arduino analog pin
#define ANALOG_PIN0		0
#define ANALOG_PIN1		1
#define ANALOG_PIN2		2
#define ANALOG_PIN3		3
#define ANALOG_PIN4		4


//arduino led
#define LED_PIN		9

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

uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{
	if(ackbyte == 0xF5)
	{
		//ack
		return 1;
	}
	else
	{
		//nack
		return 0;
	}
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


	uint8_t dummy_read;
	uint8_t dummy_write = 0xff;

	GPIO_ButtonInit();

	//this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	//this function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	//Making SSOE 1 does NSS output enable
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1)
	{
		//wait until button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		//to avoid button de-bouncing
		delay();

		//enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		//1. CMD_LED_CTRL <pin(no1)> 	<value(1)>

		uint8_t commandcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];

		//send command
		SPI_SendData(SPI2, &commandcode, 1);

		//do dummy read to clear off RXNE
		SPI_ReceiveData(SPI2,&dummy_read, 1);

		//send some dumy bits (1byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		//read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if(! SPI_VerifyResponse(ackbyte) )
		{
			//send arguments
			args[0] = LED_PIN;
			args[1]	= LED_ON;

			//send commands
			SPI_SendData(SPI2, args, 2);

		}

		//2. CMD_SENSOR_READ   <analog pin number(1)>
		while( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		//to avoid button de-bouncing
		delay();

		commandcode = COMMAND_SENSOR_READ ;
		//send command
		SPI_SendData(SPI2, &commandcode, 1);

		//do dummy read to clear off RXNE
		SPI_ReceiveData(SPI2,&dummy_read, 1);

		//send some dumy bits (1byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		//read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if( ! SPI_VerifyResponse(ackbyte) )
		{
			//send arguments
			args[0] = ANALOG_PIN0;

			//send commands
			SPI_SendData(SPI2, args, 1);

			//do dummy read to clear off RXNE
			SPI_ReceiveData(SPI2,&dummy_read, 1);

			//insert some delay so that slave can ready with the data
			delay();

			//send some dumy bits (1byte) to fetch the response from the slave
			SPI_SendData(SPI2, &dummy_write, 1);

			uint8_t analog_read;
			SPI_ReceiveData(SPI2, &analog_read, 1);
		}

		//do dummy read to clear off RXNE
		SPI_ReceiveData(SPI2,&dummy_read, 1);

		//insert some delay so that slave can ready with the data
		delay();

		//send some dumy bits (1byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		uint8_t analog_read;
		SPI_ReceiveData(SPI2, &analog_read, 1);

		//lets confirm SPI is not busy
		while( SPI_GetFlagStatus( SPI2, SPI_BUSY_FLAG ) ) ;

		//Disable the SPI2 periphrals
		SPI_PeripheralControl(SPI2, DISABLE);


	}






	return 0;
}
