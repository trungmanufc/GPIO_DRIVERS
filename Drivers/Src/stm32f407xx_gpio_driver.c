#include"stm32f407xx_gpio_driver.h"

void GPIO_DeInit(GPIO_Periph *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
	else if(pGPIOx == GPIOI)
	{
		GPIOH_REG_RESET();
	}

	}


void GPIO_PeriClockControl(GPIO_Periph *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOH_PCLK_EN();
		}

	}
	}

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp=0;

	//enable clock for GPIOx
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx , ENABLE);

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_Pin));
		pGPIOHandle->pGPIOx->MODER &= ~(0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_Pin);//Clearing
		pGPIOHandle->pGPIOx->MODER |= temp;
		temp = 0;
	}
	else
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1. Configure the FTSR
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_Pin );
			//Clear the corresponding RTSR bit
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_Pin );
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1. Configure the RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_Pin );
			//Clear the corresponding FTSR bit
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_Pin );

		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1. Configure the RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_Pin );
			//Configure the FTSR
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_Pin );
		}

		//2. Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_Pin / 4 ;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_Pin % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << ( temp2 * 4 );


		//3. Enable the exti interrupt delivery using IMR
		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_Pin;
	}

	//Configure the speed
	temp =( pGPIOHandle->GPIO_PinConfig.GPIO_Pinspeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_Pin));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0X03 << pGPIOHandle->GPIO_PinConfig.GPIO_Pin);//clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	//Configure the pupd setiings
	temp =( pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_Pin));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x03 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_Pin));//clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	//configure the optype
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_OUT)
	{
		temp= ( pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_Pin);
		pGPIOHandle->pGPIOx->OTYPER &= ~( 0x01 << pGPIOHandle->GPIO_PinConfig.GPIO_Pin );
		pGPIOHandle->pGPIOx->OTYPER |= temp;
		temp = 0;
	}

	//Configure the alt function registers
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint32_t temp1, temp2;

		temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_Pin / 8);
		temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_Pin % 8);
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0x03 << (4 * temp2) );//clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4* temp2));


	}

}
uint16_t GPIO_ReadFromInputPort(GPIO_Periph *pGPIOx)
{
	uint16_t value;

	value = (uint16_t)pGPIOx->IDR;

	return value;

	}
uint8_t GPIO_ReadFromInputPin(GPIO_Periph *pGPIOx, uint8_t PinNumber)
{

	uint8_t value;

	value =(uint8_t) ((pGPIOx->IDR  >>  PinNumber) & 0x01);

	return value;

	}

void GPIO_WriteToOutputPort(GPIO_Periph *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

void GPIO_WriteToOutputPin(GPIO_Periph *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}
void GPIO_ToggleOutputPin(GPIO_Periph *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}


void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << ( IRQNumber % 32 ) );
		}
		else if(IRQNumber >= 64 && IRQNumber <96)
		{
			//program ISER2 register
			*NVIC_ISER2 |= ( 1 << ( IRQNumber % 64 ) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ISER1 register
			*NVIC_ICER1 |= ( 1 << ( IRQNumber % 32 ) );
		}
		else if(IRQNumber >= 64 && IRQNumber <96)
		{
			//program ISER2 register
			*NVIC_ICER2 |= ( 1 << ( IRQNumber % 64 ) );
		}
	}

}

/***********************************************************/
/*                     GPIO_HANDLING                       */

void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. First lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = ( 8 * iprx_section ) + ( 8 - NO_PR_BITS_IMPLEMENTED);

	*( NVIC_PR_BASE_ADDR + iprx ) |= ( IRQPriority << shift_amount );
}


void GPIO_IRQHandling(uint8_t PinNumber)
{
	if(EXTI->PR & ( 1 << PinNumber ))
	{
		EXTI->PR |= ( 1 << PinNumber );
	}
}
