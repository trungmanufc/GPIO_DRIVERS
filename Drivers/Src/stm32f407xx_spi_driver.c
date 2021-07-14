/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Jun 16, 2021
 *      Author: CHOCOPI
 */
#include "stm32f407xx_spi_driver.h"

static void SPI_TXE_InterruptHandle(SPI_Handle_t *pSPIHandle);
static void SPI_RXNE_InterruptHandle(SPI_Handle_t *pSPIHandle);
static void SPI_OVRERR_InterruptHandle(SPI_Handle_t *pSPIHandle);

//SPI Peripheral Clock setup
void SPI_PeriClockControl(SPI_Periph *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
	}
}


void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//peripheral clock enable

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//first configure the SPI_CR1 register
	uint32_t temp_reg = 0;

	//1. Confgigure the DeviceMode
	temp_reg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2 ;

	//2. Configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig ==  SPI_BUS_CONFIG_FD)
	{
		//Clear bidi mode
		temp_reg &= ~( 1 << SPI_CR1_BIDIMODE );
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{	//Set bidi mode
		temp_reg |= ( 1 << SPI_CR1_BIDIMODE );
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//Clear bidi mode
		temp_reg &= ~( 1 << SPI_CR1_BIDIMODE );
		//set RXONLY bit
		temp_reg |= ( 1 << SPI_CR1_RXONLY );
	}

	//3. Configure the SPI Serial clock speed(baud rate)
	temp_reg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4. Configure the DFF
	temp_reg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. Configure the CPOL
	temp_reg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6. Configure the CPHA
	temp_reg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	//7. Configure the SSM
	temp_reg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;
	pSPIHandle->pSPIx->CR1 = temp_reg;



}

uint8_t SPI_GetFlagStatus(SPI_Periph *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void SPI_SendData(SPI_Periph *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while ( Len > 0 )
	{
		//1. Wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//2. Check the DFF bit in CR1
		if(pSPIx->CR1 & ( 1 << SPI_CR1_DFF ))
		{
			//16 bits DFF
			//1. Load the data into the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			//8 bits DFF
			//1. Load the data into the DR
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}


void SPI_ReceiveData(SPI_Periph *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. Wait until RXNE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET) ;

		//2. Check the DFF bits in CR1
		if( (pSPIx->CR1) & ( 1 << SPI_CR1_DFF ))
		{
			//16 bit DFF
			//1. Load the data from the DR to Rxbuffer
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;
		}
		else
		{
			//8 bits DFF
			//1. Load the data from the DR to Rxbuffer
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}

	}

}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		//1. Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		//2. Mark the SPI state as busy in transmission so that
		// no other code can take over same SPI peripheral
		pSPIHandle->TxState = SPI_BUSY_IN_TX ;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_TXEIE );

		//4. Data Transmission will be handled by the ISR code
	}
	return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_RX)
	{
		//1. Save the Rx buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		//2. Mark the SPI state as busy in transmission so that
		// no other code can take over same SPI peripheral
		pSPIHandle->RxState = SPI_BUSY_IN_RX ;

		//3. Enable the RXNEIE control bit to get interrupt whenever RXNE flag is set
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_RXNEIE );

		//4. Data Receive will be handled by the ISR code

	}
	return state;
}

void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;
	// first lets check for TXE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_TXE ) ;
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE ) ;

	if( temp1 && temp2 )
	{
		//handle TXE
		SPI_TXE_InterruptHandle(pHandle);
	}

	// Check for RXNE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE ) ;
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE ) ;

	if( temp1 && temp2 )
	{
		//handle RXNE
		SPI_RXNE_InterruptHandle(pHandle);

	}

	// Check for ovr flag
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_OVR );
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE );

	if( temp1 && temp2 )
	{
		//Handle OVR error
		SPI_OVRERR_InterruptHandle(pHandle);
	}

}

static void SPI_TXE_InterruptHandle(SPI_Handle_t *pSPIHandle)
{
	// Check the DFF bit in CR1
	if(pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF ))
	{
		//16 bits DFF
		//1. Load the data into the DR
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}
	else
	{
		//8 bits DFF
		//1. Load the data into the DR
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(! pSPIHandle->TxLen)
	{
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback( pSPIHandle, SPI_EVENT_TX_CMPLT );

	}
}

static void SPI_RXNE_InterruptHandle(SPI_Handle_t *pSPIHandle)
{
	//do RXing as per the DFF
	if(pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF ))
	{
		//16 bits DFF
		//1. Load the data into the DR
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}
	else
	{
		//8 bits
		*(pSPIHandle->pRxBuffer) = (uint8_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen --;
		pSPIHandle->pRxBuffer ++;
	}
	if(! pSPIHandle->RxLen)
	{
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback( pSPIHandle, SPI_EVENT_RX_CMPLT );
	}
}

static void SPI_OVRERR_InterruptHandle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//1. Clear the OVR flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//2. Inform the application
	SPI_ApplicationEventCallback( pSPIHandle, SPI_EVENT_OVR_ERR );
}

void SPI_PeripheralControl(SPI_Periph *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= ( 1 << SPI_CR1_SPE );
	}
	else
	{
		pSPIx->CR1 &= ~( 1 << SPI_CR1_SPE );
	}
}

void SPI_SSIConfig(SPI_Periph *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= ( 1 << SPI_CR1_SSI );
	}
	else
	{
		pSPIx->CR1 &= ~( 1 << SPI_CR1_SSI );
	}
}

void SPI_SSOEConfig(SPI_Periph *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR2 |= ( 1 << SPI_CR2_SSOE );
	}
	else
	{
		pSPIx->CR2 &= ~( 1 << SPI_CR2_SSOE );
	}
}

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. First lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = ( 8 * iprx_section ) + ( 8 - NO_PR_BITS_IMPLEMENTED);

	*( NVIC_PR_BASE_ADDR + iprx ) |= ( IRQPriority << shift_amount );
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	//TxLen is zero, so close the SPI communication and inform the application
	//that Tx is over
	//this prevents interrupt from setting up of TXE flag
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE );
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	// Reception is complete
	// Lets turn off the RXNEIE interrupt
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE );
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_Periph *pSPIx)
{
		uint8_t temp;
		temp = pSPIx->DR;
		temp = pSPIx->SR;
		(void)temp;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	//This is a weak implementation, the application may overide this function

}
