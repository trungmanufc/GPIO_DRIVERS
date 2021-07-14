/*
 * 015UART_tx.c
 *
 *  Created on: Jul 9, 2021
 *      Author: CHOCOPI
 */

#include<string.h>
#include"stm32f407xx.h"
#include"stm32f407xx_gpio_driver.h"
#include"stm32f407xx_usart_driver.h"

char msg[1024] = "UART Tx testing...\n\r";

USART_Handle_t usart2;

void USART2_Init(void)
{
	usart2.pUSARTx = USART2;
	usart2.USART_Config.USART_Baud = USART_STD_BAUD_9600;
	usart2.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart2.USART_Config.USART_Mode = USART_MODE_TXRX;
	usart2.USART_Config.USART_NoOfStopBits = USART_WORDLEN_8BITS;
	usart2.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART_Init(&usart2);
}

void USART2_GPIOInit(void)
{
	GPIO_Handle_t usart_gpio;

	usart_gpio.pGPIOx = GPIOA;
	usart_gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	usart_gpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usart_gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
	usart_gpio.GPIO_PinConfig.GPIO_Pinspeed = GPIO_SPEED_FAST;
	usart_gpio.GPIO_PinConfig.GPIO_PinAltFunMode = 7;

	//USART2 TX
	usart_gpio.GPIO_PinConfig.GPIO_Pin = GPIO_PIN_NO_2;
	GPIO_Init(&usart_gpio);

	//USART2 RX
	usart_gpio.GPIO_PinConfig.GPIO_Pin = GPIO_PIN_NO_3;
	GPIO_Init(&usart_gpio);

}

int main()
{
	USART2_GPIOInit();

	USART2_Init();

	USART_PeripheralControl( USART2, ENABLE );

	USART_SendData( &usart2, (uint8_t*)msg, strlen(msg) );

	while(1);

	return 0;

	}
