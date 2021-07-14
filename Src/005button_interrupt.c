/*
 * 005button_interrupt.c
 *
 *  Created on: Jun 14, 2021
 *      Author: CHOCOPI
 */


#include"stm32f407xx_gpio_driver.h"
#include"stm32f407xx.h"


void delay(void);

#define LOW 0
#define BUTTON_PRESSED LOW

int count;
int main(void)
{
	GPIO_Handle_t GPIOled;
	GPIOled.pGPIOx = GPIOB;
	GPIOled.GPIO_PinConfig.GPIO_Pin = GPIO_PIN_NO_13;
	GPIOled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOled.GPIO_PinConfig.GPIO_Pinspeed = GPIO_SPEED_FAST;
	GPIOled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIOled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

	GPIO_PeriClockControl(GPIOB, ENABLE);

	GPIO_Init(&GPIOled);

	GPIO_Handle_t GPIO_button;
	GPIO_button.pGPIOx = GPIOD;
	GPIO_button.GPIO_PinConfig.GPIO_Pin = GPIO_PIN_NO_5;
	GPIO_button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIO_button.GPIO_PinConfig.GPIO_Pinspeed = GPIO_SPEED_FAST;
	GPIO_button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;

	//GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GPIO_button);

	//IRQ configuration
	GPIO_IRQPriorityConfig( IRQ_NO_EXTI9_5, 15 );
	GPIO_IRQITConfig( IRQ_NO_EXTI9_5, ENABLE );


	while(1)
	{
			GPIO_WriteToOutputPin(GPIOB, 13, ENABLE);
	}

}


void delay(void)
{
	for(uint32_t i=0; i<500000; i++);
}

void EXTI9_5_IRQHandler(void)
{
	count++;
	delay();
	GPIO_ToggleOutputPin( GPIOB , GPIO_PIN_NO_14);
	GPIO_IRQHandling( GPIO_PIN_NO_5 );

}
