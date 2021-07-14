/*
 * stm32f407xx_usart_driver.c
 *
 *  Created on: Jul 9, 2021
 *      Author: CHOCOPI
 */


#include"stm32f407xx_usart_driver.h"

/************************************************/
/**************USART set baud rate***************/
/************************************************/
void USART_SetBaudRate(USART_Periph *pUSARTx, uint32_t BaudRate)
{
	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t USARTdiv;

	//Variables to hold Mantissa and Fraction values
	uint32_t M_part, F_part;

	uint32_t tempreg = 0;

	//get the value of APB bus aclock into the variable PCLKx
	if(pUSARTx == USART1 || pUSARTx == USART6)
	{
		//USART1 and USART6 are hanging on APB2 bus
		PCLKx = RCC_GetPCLK2Value();
	}else
	{
		PCLKx = RCC_GetPCLK1Value();
	}

	//Check for OVER8 configuartion bit
	if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8 ))
	{
		//OVER8 = 1, over sampling by 8
		USARTdiv = (( 25 * PCLKx ) / ( 2 * BaudRate ));
	}else
	{
		//OVER8 = 0, over sampling by 16
		USARTdiv = (( 25 * PCLKx ) / ( 4 * BaudRate ));
	}

	//Calculate the Mantissa part
	M_part = USARTdiv / 100;

	//Place the Mantissa part in appopriate bit position, refer USART_BRR
	tempreg |= M_part << 4;

	//Extract the fraction part
	F_part = (USARTdiv - (M_part * 100));

	//Calculate the final fractional
	if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8 ))
	{
		//OVER8 = 1, over sampling by 8
		F_part = ((( F_part * 8 ) + 50) / 100) & ((uint8_t)0x07);
	}else
	{
		//OVER8 = 0, Oversampling by 16
		F_part = ((( F_part * 16 ) + 50) / 100) & ((uint8_t)0x0F);
	}

	//Place the fractional part in appropriate bit position, refer USART_BRR
	tempreg |= F_part;

	//copy the value of tempreg into BRR register;
	pUSARTx->BRR = tempreg;
}


/*****************USART Pripheral Control*******************/
void USART_PeripheralControl(USART_Periph *pUSARTx, uint8_t Cmd)
{
	if(Cmd == ENABLE)
	{
		pUSARTx->CR1 |= (1 << 13);
	}else
	{
		pUSARTx->CR1 &= ~(1 << 13);
	}

}


/***************USART Peripheral Clock Control********************/
void USART_PeriClockControl(USART_Periph *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}else if (pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}else if (pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}
		else if (pUSARTx == UART4)
		{
			UART4_PCLK_EN();
		}
	}
	else
	{
		//TODO
	}

}

//USART Initialization
void USART_Init(USART_Handle_t *pUSARTHandle)
{
	//Temporary variable
	uint32_t tempreg = 0;

	//Configuration of CR1

	//Implement the code to enable the Clock for given USART peripheral
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if( pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX )
	{
		//Implement the code to enable the Receiver bit field
		tempreg |= ( 1 << USART_CR1_RE );
	}else if( pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX )
	{
		//Implement the code to enable the Transmitter bit field
		tempreg |= ( 1 << USART_CR1_TE );
	}else if( pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX )
	{
		//Implement the code to enable both Transmitter and Receiver bit fields
		tempreg |= ( ( 1 << USART_CR1_RE  ) | ( 1 << USART_CR1_TE ) );
	}

	//Implement the code to configure the Word Length configuration item
	tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M;

	//Configuration of parity control bit fields
	if( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN )
	{
		//Implement the code to enale parity control
		tempreg |= ( 1 << USART_CR1_PCE );

		//Implement the code to enable EVEn parity
		//Not required because by default EVEN parity will be selected once you enable the parity control
	}else if( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD )
	{
		//Implement the code to enable parity control
		tempreg |= ( 1 << USART_CR1_PCE );

		//Implement the code to enable ODD parity
		tempreg |= ( 1 << USART_CR1_PS );
	}

	//Program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = tempreg;

	/***********************Configuration of CR2**********************/
	tempreg = 0;

	//Implement the code to configure the number of stop bits inserted using USART frame transmission
	tempreg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << 12;

	//Program the CR2 register
	pUSARTHandle->pUSARTx->CR2 = tempreg;


	/***********************Configuration of CR2**********************/
	tempreg = 0;


	//Configuration of USART hardware flow Control
	if( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS )
	{
		//Implement the code to enable CTS Flow Control
		tempreg |= ( 1 << USART_CR3_CTSE );
	}else if( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS )
	{
		//Implement the code to enable RTS Flow Control
		tempreg |= ( 1 << USART_CR3_RTSE );
	}else if( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS )
	{
		//Implement the code to enable both CTS and RTS Flow Control
		tempreg |= ( ( 1 << USART_CR3_CTSE ) | ( 1 << USART_CR3_RTSE ) );
	}

	pUSARTHandle->pUSARTx->CR3 =tempreg;

	/********************Configuration of BRR(Baudrate Register)***************/
	//Implement the code to configure the baud rate
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);


}



/************USART Get Flag Satus****************/
uint8_t USART_GetFlagStatus(USART_Periph *pUSARTx , uint32_t FlagName)
{
	if(pUSARTx->SR & ( 1 << FlagName ))
	{
		return SET;
	}
	return RESET;
}




/************************************************/
/**********USART send and receive data***********/
/************************************************/
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint16_t *pdata;
	//Loop over until "Len" number of bytes are transferred
	for( uint32_t i = 0; i < Len; i++ )
	{
		//Implement the code to wait until TXE flag is set in the SR
		while (! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE) );

		//Check the USART_WordLength item for 9bits or 8bits in a frame
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//If 9BIT, LOAD the DR with 2 bytes masking the bits other than first 9 bits
			pdata = (uint16_t*)pTxBuffer;
			pUSARTHandle->pUSARTx->DR = ( *pdata & (uint16_t)0x1FF );

			//Check for USART_ParityControl
			if( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE )
			{
				//No parity is used in this transfer.So, 9 bits of user data willbe sent
				//Implement the code to increment pTxbuffer twice
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer. So, 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			//This is 8bit data transfer
			pUSARTHandle->pUSARTx->DR = *pTxBuffer ;

			//Implement the code to increment the buffer address
			pTxBuffer++;
		}
	}

	//Implement the code to wait till TC flag is set in the SR
	while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC));
}

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	//Loop over until "Len" bunber of bytes are transferred
	for(uint32_t i = 0; i < Len; i++ )
	{
		//Implement the code to wait until RXNE flag is set in the SR
		while (! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE) );

		//Check the USART_WordLength item for 9bits or 8bits in a frame
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//We are going to receive 9bit data in a frame

			//Check whether we are using paritycontrol or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used. So, all 9 bits will be of user data

				//Read only the first 9 bits, so mask the DR with 0x01FF
				*((uint16_t*)pRxBuffer) = ( pUSARTHandle->pUSARTx->DR & (uint16_t) 0x01FF );

				//Increase the pRxBuffer twice
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				//Parity bit is used, so 8bits of user data and 1 bit is parity
				*pRxBuffer = ( pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF );

				//Increase pRxBuffer
				pRxBuffer++;
			}
		}
		else
		{
			//We are go to receive 8bit data in a frame

			//Check whether we are using Parity control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity bit is used. So all 8 bits will be of user data

				//Read only the first 8bits of the DR, so mask it with 0xFF
				*pRxBuffer = ( pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF );

			}
			else
			{
				//Parity bit is used. So mask the DR with 0x7F
				*pRxBuffer = ( pUSARTHandle->pUSARTx->DR & 0x7F );
			}

			//Increase pRxBuffer
			pRxBuffer++;
		}
	}
}


/*************************************************/
/*********USART send and receive data IT**********/
/*************************************************/
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t txstate = pUSARTHandle->TxBusyState;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		//Implement the code to enable interrupt for TXE
		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_TXEIE );

		//Implement the code to enable interrupt for TC
		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_TCIE );

	}

	return txstate;
}


/*************************************************/
/***********USART IRQ Interrupt Config************/
/*************************************************/
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if(rxstate != USART_BUSY_IN_RX)
	{
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		//Implement the code to enable interrupt for RXNE
		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_RXNEIE );
	}

	return rxstate;
}




void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 6 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
		}
	}

}

void USART_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}

void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{
	uint32_t temp1 , temp2, temp3;

	uint16_t *pdata;

	/*************************Check for TC flag ********************************************/

	//Implement the code to check the state of TC bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TC );

	//Implement the code to check the state of TCEIE bit
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TCIE );

	if( temp1 && temp2 )
	{
		//This interrupt is because of TC

		//Close transmission and call application callback if TxLen is zero
		if( pUSARTHandle->TxBusyState == USART_BUSY_IN_TX )
		{
			//Check the TxLen. If it is zero then close the data transmission
			if( !pUSARTHandle->TxLen )
			{
				//Implement the code to clear the TC flag
				pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_TC );

				//Implement the code to clear the TCIE control bit


				//Reset the application state
				pUSARTHandle->TxBusyState = USART_READY;

				//Reset Buffer Address to NULL
				pUSARTHandle->pTxBuffer = NULL;

				//Reset the length to zero
				pUSARTHandle->TxLen = 0;

				//Call the application callback with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_TX_CMPLT);
			}
		}
	}

	/*************************Check for TXE flag *************************************/
	//Implement the code to check the state of the TXE bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TXE );

	//Implement the code to check the state of TXEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TXEIE );

	if( temp1 && temp2 )
	{
		//This interrupt is because of TXE
		if( pUSARTHandle->TxBusyState == USART_BUSY_IN_TX )
		{
			if( pUSARTHandle->TxLen > 0 )
			{
				//Check the USART_WordLength item for 9bit or 8bit in a frame
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//if 9BIT load the DR with 2bytes masking the bits other than first 9 bits
					pdata = (uint16_t*) pUSARTHandle->pTxBuffer;
					pUSARTHandle->pUSARTx->DR = ( *pdata & (uint16_t)0x01FF );

					//Check for USART_ParityControl
					if( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE )
					{
						//No parity bit is used in this transfer, so 9bits of user data will be sent
						//Implement the code to increament pTxbuffer twice
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen-=2;
					}
					else
					{
						//Parity bit is used in this transfer, so 9 bits of user data will be sent
						//The 9th bit will be replaced by parity bit by the Hardware
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen--;
					}

				}
				else
				{
					//This is 8bit data
					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer & (uint8_t)0xFF );

					//Implement the code to increment the buffer address
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->TxLen-=1;
				}
			}
			if( pUSARTHandle->TxLen == 0 )
			{
				//TxLen is zero
				//Implement the code to clear the TXEIE bit ( disable interrupt for TXE flag )
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_TXEIE );
			}
		}
	}

	/*************************Check for RXNE flag *************************************/
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_RXNE );
	temp2 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_CR1_RXNEIE );

	if( temp1 && temp2 )
	{
		//This interrupt is beacause of RXNE
		if(pUSARTHandle->RxBusyState == USART_BUSY_IN_RX)
		{
			if(pUSARTHandle->RxLen > 0)
			{
				//Check the USART_WordLength to decide whether we rare going to receive 9bit of data in a frame
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//We are going to receive 9bit data in a frame

					//Now, checking wheteher we are using USART_ParityControl or not
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used, so all 9bits will be of user data

						//Read only first 9 bits so mask the DR with 0x01FF
						*( (uint16_t*)pUSARTHandle->pRxBuffer ) = ( pUSARTHandle->pUSARTx->DR & (uint16_t)0x01FF );

						//Now increment the pRxBuffer twice
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen-=2;
					}
					else
					{
						//Parity is used, so 8 bits will be of user data and 1 bit is parity
						*pUSARTHandle->pRxBuffer = ( pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF );
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen-=1;
					}
				}
				else
				{
					//We are going to receive 8bit data in a frame

					//Now, check whether we are using USART_ParityControl or not
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity bit is used, so all 8bits will be of user data

						//read 8 bits from DR
						*pUSARTHandle->pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF );
					}
					else
					{
						//Parity bit is used, so, 7 bits will be of user data and 1 bit is parity


						//Read only 7 bits, hence mask the DR with 0x7F
						*pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F );
					}

					//Now, increment the pRxBuffer
					pUSARTHandle->pRxBuffer++;
					pUSARTHandle->RxLen-=1;
				}
			}
		}
	}

}



