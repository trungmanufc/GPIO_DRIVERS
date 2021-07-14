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
	GPIOled.GPIO_PinConfig.GPIO_Pin = GPIO_PIN_NO_14;
	GPIOled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOled.GPIO_PinConfig.GPIO_Pinspeed = GPIO_SPEED_FAST;
	GPIOled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIOled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

	GPIO_PeriClockControl(GPIOB, ENABLE);

	GPIO_Init(&GPIOled);

	GPIO_Handle_t GPIO_button;
	GPIO_button.pGPIOx = GPIOB;
	GPIO_button.GPIO_PinConfig.GPIO_Pin = GPIO_PIN_NO_12;
	GPIO_button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIO_button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	//GPIO_PeriClockControl(GPIOB, ENABLE);

	GPIO_Init(&GPIO_button);



	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_12) == BUTTON_PRESSED)
		{
			count++;
			delay();
			GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_14);

		}
	}

}


void delay(void)
{
	for(uint32_t i=0; i<500000; i++);
	}
