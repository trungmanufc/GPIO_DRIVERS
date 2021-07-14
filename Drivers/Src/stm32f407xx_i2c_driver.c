/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Jun 22, 2021
 *      Author: CHOCOPI
 */

#include "stm32f407xx_i2c_driver.h"


static void I2C_GenerateStartCondition(I2C_Periph *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_Periph *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_Periph *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);

/*
 * Init and De-init
 */





void I2C_PeriClockControl(I2C_Periph *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		//TODO
	}

}



void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;

	//Enable the clock for the I2Cx peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	//Ack control bit
	tempreg |= ( pI2CHandle->I2C_Config.I2C_ACKControl << 10 ) ;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//Configure the FREQ field of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = ( tempreg & 0x3F );

	//Program the device own address
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode
		ccr_value = ( RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed) );
		tempreg |= ( ccr_value & 0xFFF );
	}else
	{
		//mode is fastmode
		tempreg |= ( 1 << 15 );
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = ( RCC_GetPCLK1Value() / ( 3 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		}else
		{
			ccr_value = ( RCC_GetPCLK1Value() / ( 25 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//TRISE configuration
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode
		tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1;
	}
	else
	{
		tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000000U) +1;
	}

	pI2CHandle->pI2Cx->TRISE = ( tempreg & 0x3F );

}



void I2C_DeInit(I2C_Periph *pI2Cx);



void I2C_PeripheralControl(I2C_Periph *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
		//pI2cBaseAddress->CR1 |= I2C_CR1_PE_Bit_Mask;
	}else
	{
		pI2Cx->CR1 &= ~(1 << 0);
	}

}



static void I2C_GenerateStartCondition(I2C_Periph *pI2Cx)
{
	pI2Cx->CR1 |= ( 1 << I2C_CR1_START );
}



void I2C_GenerateStopCondition(I2C_Periph *pI2Cx)
{
	pI2Cx->CR1 |= ( 1 << I2C_CR1_STOP );
}

void I2C_SlaveEnableDisableCallbackEvents(I2C_Periph *pI2Cx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN );
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN );
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN );
	}
	else
	{
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN );
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN );
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITERREN );
	}
}

static void I2C_ExecuteAddressPhaseWrite(I2C_Periph *pI2Cx, uint8_t SlaveAddr)
{
	//left shift Slave Address by 1 and send the r/nw bit 0
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~( 1 << 0 );

	//Put the Address to the DR
	pI2Cx->DR = SlaveAddr;
}



static void I2C_ExecuteAddressPhaseRead(I2C_Periph *pI2Cx, uint8_t SlaveAddr)
{
	//left shift Slave Address by 1 and send the r/nw bit 1
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= ( 1 << 0 );

	//Put the Address to the DR
	pI2Cx->DR = SlaveAddr;
}



uint8_t I2C_GetFlagStatus(I2C_Periph *pI2Cx , uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}



static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummy_read;
	//check device mode
	if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL ))
	{
		//device is in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize == 1)
			{
				//First disable the ACK
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				//Clear the ADDR flag(read SR1, read SR2)
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}
		}
		else
		{
			//Clear the ADDR flag(read SR1, read SR2)
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;
		}
	}
	else
	{
		//device is in slave mode
	}
}



void I2C_ManageAcking(I2C_Periph *pI2Cx, uint8_t EnorDi)
{
	if ( EnorDi == I2C_ACK_ENABLE )
	{
		pI2Cx->CR1 |= ( 1 << I2C_CR1_ACK );
	}
	else
	{
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_ACK );
	}
}



//Data Send and Receive
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	//1. Generate the Start Condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that start generation is completed by checking the SB flag in SR1
	//   Note: Until SB is cleared SCL will be stretched ( pulled to LOW )
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)) ;

	//3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Confirm that address phase is completed by checking the ADDR flag
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//5. Clear the ADDR flag according tp its software sequence
	I2C_ClearADDRFlag(pI2CHandle);

	//6. Send the data until len becomes 1
	while(Len > 0)
	{
		while(! I2C_GetFlagStatus( pI2CHandle->pI2Cx, I2C_FLAG_TXE ) );
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		Len --;
	}

	//7. When Len becomes zero wait for TXE = 1 and BTF = 1 before generating the STOP condition
	//		Note: TXE = 1, BTF = 1, means that both SR and DR are empty and next transmission should begin
	//		When BTF = 1 SCL will be stretched ( pulled to LOW )
	while ( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));

	while ( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	//8. Generate the Stop Condition
	if(Sr == I2C_DISABLE_SR)
	{
		I2C_GenerateStopCondition( pI2CHandle->pI2Cx );
	}
}



void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	//1. Generate the START condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that generation is completed by checking the SB flag
		while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB) );

	//3. Send the address of the slave with r/nw bit set to R(1)
		I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Wait until address phase is completed by checking the ADDR flag in SR1
		while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	if(Len == 1)
	{
		//Disable Acking
			I2C_ManageAcking( pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		//Clear the ADDR flag
			I2C_ClearADDRFlag( pI2CHandle);

		//Wait until RXNE becomes 1
			while( ! I2C_GetFlagStatus( pI2CHandle->pI2Cx, I2C_FLAG_RXNE) );

		//Generate STOP condition
			I2C_GenerateStopCondition( pI2CHandle->pI2Cx);

		//Read data into buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}


	//Procedure to read data from slave when Len > 1
	if(Len > 1)
	{
		//Clear the ADDR flag
		I2C_ClearADDRFlag( pI2CHandle);

		//Read the data until Len becomes zero
		for( uint32_t i = Len; i > 0; i--)
		{
			//Wait until RXNE becomes 1
			while( ! I2C_GetFlagStatus( pI2CHandle->pI2Cx, I2C_FLAG_RXNE ) );

			if( i == 2 )
			{
				//Clear the ACK bit
				I2C_ManageAcking( pI2CHandle->pI2Cx , I2C_ACK_DISABLE );

				//Generate STOP condition
				I2C_GenerateStopCondition( pI2CHandle->pI2Cx );

			}

			//Read the data from data register in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			//Increment the buffer address
			pRxBuffer++;
		}

		//Re-enable ACking
		I2C_ManageAcking( pI2CHandle->pI2Cx , I2C_ACK_ENABLE);

	}

}


void I2C_SlaveSendData(I2C_Periph *pI2C,uint8_t data)
{
	pI2C->DR = data;
}


uint8_t I2C_SlaveReceiveData(I2C_Periph *pI2C)
{
	return (uint8_t)pI2C->DR;
}


void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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


/*********************************************************************
 * @fn      		  - I2C_IRQPriorityConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}




uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;
	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxbuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN );

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN );

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN );
	}

	return busystate;
}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX) )
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx );

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 = ( 1 << I2C_CR2_ITBUFEN );

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 = ( 1 << I2C_CR2_ITEVTEN );

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 = ( 1 << I2C_CR2_ITERREN );
	}
	return busystate;
}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->TxLen > 0)
	{
		//1. Load the data into DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		//2. Decrement the TxLen
		pI2CHandle->TxLen--;

		//3. Increment the buffer address
		pI2CHandle->pTxBuffer++;
	}
}


void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	//We have to do the data reception
			if(pI2CHandle->RxSize == 1)
			{
				*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;

				pI2CHandle->RxLen--;
			}

			if(pI2CHandle->RxSize > 1)
			{
				if(pI2CHandle->RxLen == 2)
				{
					//Clear the ACK bit
					I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
				}
				//read DR
				*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
				pI2CHandle->pRxBuffer++;
				pI2CHandle->RxLen--;

			}

			if( pI2CHandle->RxLen == 0 )
			{
				//Close the I2C data reception and notify the application

				//1. Generate the stop condition
				if(pI2CHandle->Sr == I2C_DISABLE_SR)
				{
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}

				//2. Close the I2C Rx
				I2C_CloseReceiveData(pI2CHandle);

				//3. Notify the application
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
			}
}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	//Interruot Handling for both master and slave mode of a device
	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITEVTEN );
	temp2 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITBUFEN );

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_SB );
	//1. Handle for Interrupt generated by SB event
	//Note: SB FLAG is only applicable in Master Mode
	if(temp1 && temp3)
	{
		//The interrupt is generated because of SB event
		//This block will not be executed in slave mode because slave SB is always zero
		//In this block lets execute the address phase
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_ADDR );
	//2. Handle for interrupt generated by ADDR event
	//Note: When Master mode: Address is sent
	// 		When Slave mode	: Address matched with own address
	if(temp1 && temp3)
	{
		//The interrupt is generated because of the ADDR event
		I2C_ClearADDRFlag(pI2CHandle);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_BTF );
	//3. Handle for interrupt generated by BTF( Byte Transfer Finished )
	if(temp1 && temp3)
	{
		//BTF flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			//make sure that TXE is also set
			if(pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE ))
			{
					//BTF, TXE = 1
					if(pI2CHandle->TxLen == 0)
					{
					//1. Generate the STOP condition
					if(pI2CHandle->Sr == I2C_DISABLE_SR)
					{
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}

					//2. Generate all the member elements of the handle structure
					I2C_CloseSendData(pI2CHandle);

					//3. Notify the application about transmission complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			;
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_STOPF);
	//4. Handle for Interrupt generated by STOPF event
	//	Note: Stop detection flag is applicable only slave mode, for master this flag will not set in master mode
	if(temp1 && temp3)
	{
		//STOP flag is SET
		//Clear the STOPF flag ( Read the SR1 and Write to CR1 )

		pI2CHandle->pI2Cx->CR1 |= 0x00;

		//Notify the application the STOPF is generated by the master
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);

	}

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE);
	//5. Handle for interrupt generated by TXE event
	if(temp1 && temp2 && temp3)
	{
		//Check for device mode
		if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
		{
			//TXE flag is set
			//We have to do the data transmission
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}else
		{
			//slave
			//make sure that the slave is really in transmitter mode
		    if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA))
		    {
		    	I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_REQ);
		    }
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_RXNE);
	//6. Handle for interrupt generated by RXNE event
	if(temp1 && temp2 && temp3)
	{
		//CHECK FOR DEVICE MODE
		if(pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR2_MSL ))
		{
			//The device is master

			//RXNE flag is set
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}else
		{
			//slave
			//make sure that the slave is really in transmitter mode
		    if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA))
		    {
		    	I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_RCV);
		    }
		}
	}


}

/*********************************************************************
 * @fn      		  - I2C_ER_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Complete the code also define these macros in the driver
						header file
						#define I2C_ERROR_BERR  3
						#define I2C_ERROR_ARLO  4
						#define I2C_ERROR_AF    5
						#define I2C_ERROR_OVR   6
						#define I2C_ERROR_TIMEOUT 7
 */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1, temp2;

	//Know the status of ITERREN control bit in CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);

	/********************Check for bus error*****************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_BERR);
	if( temp1 && temp2 )
	{
		//This is bus error

		//Implement the code to clear the bus error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
	}

	/*****************Check for arbitration lost error****************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
	}

	/*******************Check for ACK failure error******************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF );

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
	}

	/******************Check for Overrun/underrun error****************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR );

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR );
	}

	/********************Check for Time out error*******************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT );
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT );

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT );
	}

}



void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN );

	//Implement the code to disable the ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN );

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;
	I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);

}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN );

	//Implement the code to disable the ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN );

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;

}


