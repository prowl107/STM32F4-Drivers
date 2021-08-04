// /*
//  * stm32f407xx_USART_driver.c
//  *
//  *  Created on: Jun 2, 2021
//  *      Author: milesosborne
//  */

// #include "STM32F4xx_USART.h"
// /*********************************************************************
//  * @fn      	      -
//  *
//  * @brief             -
//  *
//  * @param[in]         -
//  * @param[in]         -
//  * @param[in]         -
//  *
//  * @return            -  none
//  *
//  * @Note              -  none
//  */
// void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t Enable_Disable)
// {
// 	if (Enable_Disable == ENABLE)
// 	{
// 		if (pUSARTx == USART1)
// 		{
// 			USART1_PCLK_EN();
// 		}
// 		else if (pUSARTx == USART2)
// 		{
// 			USART2_PCLK_EN();
// 		}
// 		else if (pUSARTx == USART3)
// 		{
// 			USART3_PCLK_EN();
// 		}
// 		else if (pUSARTx == UART4)
// 		{
// 			UART4_PCLK_EN();
// 		}
// 		else if (pUSARTx == UART5)
// 		{
// 			UART5_PCLK_EN();
// 		}
// 		else if (pUSARTx == USART6)
// 		{
// 			USART6_PCLK_EN();
// 		}
// 	}
// 	else
// 	{
// 		if (pUSARTx == USART1)
// 		{
// 			USART1_PCLK_DI();
// 		}
// 		else if (pUSARTx == USART2)
// 		{
// 			USART2_PCLK_DI();
// 		}
// 		else if (pUSARTx == USART3)
// 		{
// 			USART3_PCLK_DI();
// 		}
// 		else if (pUSARTx == UART4)
// 		{
// 			UART4_PCLK_DI();
// 		}
// 		else if (pUSARTx == UART5)
// 		{
// 			UART5_PCLK_DI();
// 		}
// 		else if (pUSARTx == USART6)
// 		{
// 			USART6_PCLK_DI();
// 		}
// 	}
// }

// /*********************************************************************
//  * @fn      	      -
//  *
//  * @brief             -
//  *
//  * @param[in]         -
//  * @param[in]         -
//  * @param[in]         -
//  *
//  * @return            -  none
//  *
//  * @Note              -  none
//  */
// void USART_Init(USART_Handle_t *pUSARTHandle)
// {
// 	//Temporary variable
// 	uint32_t tempreg = 0;

// 	/******************************** Configuration of CR1******************************************/

// 	//Implement the code to enable the Clock for given USART peripheral
// 	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

// 	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
// 	if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
// 	{
// 		//Implement the code to enable the Receiver bit field
// 		tempreg |= (1 << USART_CR1_RE);
// 	}
// 	else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
// 	{
// 		//Implement the code to enable the Transmitter bit field
// 		tempreg |= (1 << USART_CR1_TE);
// 	}
// 	else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
// 	{
// 		//Implement the code to enable the both Transmitter and Receiver bit fields
// 		tempreg |= ((1 << USART_CR1_RE) | (1 << USART_CR1_TE));
// 	}

// 	//Implement the code to configure the Word length configuration item
// 	tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M;

// 	//Configuration of parity control bit fields
// 	if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
// 	{
// 		//Implement the code to enale the parity control
// 		tempreg |= (1 << USART_CR1_PCE);

// 		//Implement the code to enable EVEN parity
// 		//Not required because by default EVEN parity will be selected once you enable the parity control
// 	}
// 	else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD)
// 	{
// 		//Implement the code to enable the parity control
// 		tempreg |= (1 << USART_CR1_PCE);

// 		//Implement the code to enable ODD parity
// 		tempreg |= (1 << USART_CR1_PS);
// 	}

// 	//Program the CR1 register
// 	pUSARTHandle->pUSARTx->USART_CR1 = tempreg;

// 	/******************************** Configuration of CR2******************************************/

// 	tempreg = 0;

// 	//Implement the code to configure the number of stop bits inserted during USART frame transmission
// 	tempreg |= pUSARTHandle->USART_Config.TODO << TODO;

// 	//Program the CR2 register
// 	pUSARTHandle->pUSARTx->TODO = tempreg;

// 	/******************************** Configuration of CR3******************************************/

// 	tempreg = 0;

// 	//Configuration of USART hardware flow control
// 	if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
// 	{
// 		//Implement the code to enable CTS flow control
// 		tempreg |= (1 << USART_CR3_CTSE);
// 	}
// 	else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
// 	{
// 		//Implement the code to enable RTS flow control
// 		tempreg |= (1 << USART_CR3_RTS);
// 	}
// 	else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
// 	{
// 		//Implement the code to enable both CTS and RTS Flow control
// 		tempreg |= (1 << USART_CR3_RTS);
// 		tempreg |= (1 << USART_CR3_RTS);
// 	}

// 	pUSARTHandle->pUSARTx->TODO = tempreg;

// 	/******************************** Configuration of BRR(Baudrate register)******************************************/

// 	//Implement the code to configure the baud rate
// 	//We will cover this in the lecture. No action required here
// }

// /*********************************************************************
//  * @fn      	      -
//  *
//  * @brief             -
//  *
//  * @param[in]         -
//  * @param[in]         -
//  * @param[in]         -
//  *
//  * @return            -  none
//  *
//  * @Note              -  none
//  */
// void USART_DeInit(USART_RegDef_t *pUSARTx)
// {
// }

// /*********************************************************************
//  * @fn      	      -
//  *
//  * @brief             -
//  *
//  * @param[in]         -
//  * @param[in]         -
//  * @param[in]         -
//  *
//  * @return            -  none
//  *
//  * @Note              -  none
//  */
// void USART_SendData(USART_RegDef_t *pUSARTx, uint8_t *pTxBuffer, uint32_t Len)
// {
// 	uint16_t *pdata;
// 	//Loop over until "Len" number of bytes are transferred
// 	for (uint32_t i = 0; i < len; i++)
// 	{
// 		//Implement the code to wait until TXE flag is set in the SR
// 		while (!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TODO))
// 			;

// 		//Check the USART_WordLength item for 9BIT or 8BIT in a frame
// 		if (pUSARTHandle->USART_Config.TODO == USART_WORDLEN_9BITS)
// 		{
// 			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
// 			pdata = (uint16_t *)pTxBuffer;
// 			pUSARTx->USART_DR = (*pdata & (uint16_t)0x01FF);

// 			//check for USART_ParityControl
// 			if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
// 			{
// 				//No parity is used in this transfer. so, 9bits of user data will be sent
// 				//Implement the code to increment pTxBuffer twice
// 				pTxBuffer++;
// 				pTxBuffer++;
// 			}
// 			else
// 			{
// 				//Parity bit is used in this transfer . so , 8bits of user data will be sent
// 				//The 9th bit will be replaced by parity bit by the hardware
// 				pTxBuffer++;
// 			}
// 		}
// 		else
// 		{
// 			//This is 8bit data transfer
// 			pUSARTHandle->pUSARTx->DR = (*pTxBuffer & (uint8_t)0xFF);

// 			//Implement the code to increment the buffer address
// 			pTxBuffer++;
// 		}
// 	}

// 	//Implement the code to wait till TC flag is set in the SR
// 	while (!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TODO))
// 		;
// }

// /*********************************************************************
//  * @fn      	      -
//  *
//  * @brief             -
//  *
//  * @param[in]         -
//  * @param[in]         -
//  * @param[in]         -
//  *
//  * @return            -  none
//  *
//  * @Note              -  none
//  */
// void USART_ReceiveData(USART_RegDef_t *pUSARTx, uint8_t *pRxBuffer, uint32_t Len)
// {
// 	//Loop over until "Len" number of bytes are transferred
// 	for (uint32_t i = 0; i < TODO; i++)
// 	{
// 		//Implement the code to wait until RXNE flag is set in the SR
// 		while (USART_GetFlagStatus(pUSARTx, USART_FLAG_RXNE))
// 			;

// 		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
// 		if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
// 		{
// 			//We are going to receive 9bit data in a frame

// 			//check are we using USART_ParityControl control or not
// 			if (pUSARTHandle->USART_Config.USART_ParityControl == TODO)
// 			{
// 				//No parity is used. so, all 9bits will be of user data

// 				//read only first 9 bits. so, mask the DR with 0x01FF
// 				*((uint16_t *)pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t)TODO);

// 				//Now increment the pRxBuffer two times
// 				pRxBuffer++;
// 				pRxBuffer++;
// 			}
// 			else
// 			{
// 				//Parity is used, so, 8bits will be of user data and 1 bit is parity
// 				*pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);

// 				//Increment the pRxBuffer
// 				pRxBuffer++;
// 			}
// 		}
// 		else
// 		{
// 			//We are going to receive 8bit data in a frame

// 			//check are we using USART_ParityControl control or not
// 			if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
// 			{
// 				//No parity is used , so all 8bits will be of user data

// 				//read 8 bits from DR
// 				*pRxBuffer = pUSARTx->USART_DR;
// 			}

// 			else
// 			{
// 				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

// 				//read only 7 bits , hence mask the DR with 0X7F
// 				*pRxBuffer = (((uint8_t)pUSARTx->USART_DR) & (0x7F));
// 			}

// 			//increment the pRxBuffer
// 			pRxBuffer++;
// 		}
// 	}
// }

// /*********************************************************************
//  * @fn      	      -
//  *
//  * @brief             -
//  *
//  * @param[in]         -
//  * @param[in]         -
//  * @param[in]         -
//  *
//  * @return            -  none
//  *
//  * @Note              -  none
//  */
// uint8_t USART_SendDataIT(USART_RegDef_t *pUSARTRx, uint8_t *pTxBuffer, uint32_t Len)
// {
// 	uint8_t txstate = pUSARTHandle->TODO;

// 	if (txstate != USART_BUSY_IN_TX)
// 	{
// 		pUSARTHandle->TODO = Len;
// 		pUSARTHandle->pTxBuffer = pTxBuffer;
// 		pUSARTHandle->TxBusyState = USART_MODE_ONLY_TX;

// 		//Implement the code to enable interrupt for TXE
// 		pUSARTRx->USART_CR1 |= (1 << USART_CR1_TXEIE);

// 		//Implement the code to enable interrupt for TC
// 		//Is TC "transmission complete"?
// 		pUSARTRx->USART_CR1 |= (1 << USART_CR1_TCIE);
// 	}

// 	return txstate;
// }

// /*********************************************************************
//  * @fn      	      -
//  *
//  * @brief             -
//  *
//  * @param[in]         -
//  * @param[in]         -
//  * @param[in]         -
//  *
//  * @return            -  none
//  *
//  * @Note              -  none
//  */
// uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
// {
// 	uint8_t rxstate = pUSARTHandle->TODO;

// 	if (rxstate != TODO)
// 	{
// 		pUSARTHandle->RxLen = Len;
// 		pUSARTHandle->pRxBuffer = pRxBuffer;
// 		pUSARTHandle->RxBusyState = TODO;

// 		//Implement the code to enable interrupt for RXNE
// 		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_RXNEIE);
// 	}

// 	return rxstate;
// }

// /*********************************************************************
//  * @fn      	      -
//  *
//  * @brief             -
//  *
//  * @param[in]         -
//  * @param[in]         -
//  * @param[in]         -
//  *
//  * @return            -  none
//  *
//  * @Note              -  none
//  */
// void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
// {
// 	if (Enable_Disable == ENABLE)
// 	{
// 		if (IRQNumber <= 31)
// 		{
// 			//Set ISER0 Register
// 			*NVIC_ISER0 |= (1 << IRQNumber);
// 		}
// 		else if (IRQNumber > 31 && IRQNumber <= 63)
// 		{
// 			//Set ISER1 Register
// 			*NVIC_ISER1 |= (1 << IRQNumber % 32);
// 		}
// 		else if (IRQNumber > 63 && IRQNumber <= 95)
// 		{
// 			//Set ISER2 Register
// 			*NVIC_ISER2 |= (1 << IRQNumber % 64);
// 		}
// 	}
// 	else
// 	{
// 		if (IRQNumber <= 31)
// 		{
// 			//Set ISER0 Register
// 			*NVIC_ICER0 |= (1 << IRQNumber);
// 		}
// 		else if (IRQNumber > 31 && IRQNumber <= 63)
// 		{
// 			//SEt ISER1 Register
// 			*NVIC_ICER1 |= (1 << IRQNumber % 32);
// 		}
// 		else if (IRQNumber > 63 && IRQNumber <= 95)
// 		{
// 			//Set ISER2 Register
// 			*NVIC_ICER2 |= (1 << IRQNumber % 64);
// 		}
// 	}
// }

// /*********************************************************************
//  * @fn      	      -
//  *
//  * @brief             -
//  *
//  * @param[in]         -
//  * @param[in]         -
//  * @param[in]         -
//  *
//  * @return            -  none
//  *
//  * @Note              -  none
//  */
// void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
// {
// 	// Find ipr register
// 	uint8_t iprx = IRQNumber / 4;

// 	//Find priority field
// 	uint8_t iprxField = IRQNumber % 4;

// 	uint8_t shiftAmount = (8 * iprxField) + (8 - NO_PR_BITS_IMPLEMENTED);

// 	//Set priority
// 	volatile uint32_t *IPRregister = NVIC_IPR_BASE_ADDR;
// 	*(IPRregister + iprx) |= (IRQPriority << shiftAmount);
// }

// /*********************************************************************
//  * @fn      	      -
//  *
//  * @brief             -
//  *
//  * @param[in]         -
//  * @param[in]         -
//  * @param[in]         -
//  *
//  * @return            -  none
//  *
//  * @Note              -  none
//  */
// void USART_IRQHandling(USART_Handle_t *pHandle)
// {
// }

// /*********************************************************************
//  * @fn      	      -
//  *
//  * @brief             -
//  *
//  * @param[in]         -
//  * @param[in]         -
//  * @param[in]         -
//  *
//  * @return            -  none
//  *
//  * @Note              -  none
//  */
// void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
// {
// }

// /*********************************************************************
//  * @fn      	      -
//  *
//  * @brief             -
//  *
//  * @param[in]         -
//  * @param[in]         -
//  * @param[in]         -
//  *
//  * @return            -  none
//  *
//  * @Note              -  none
//  */
// uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName)
// {
// 	if (pUSARTx->USART_SR & FlagName)
// 	{
// 		return FLAG_SET;
// 	}
// 	return FLAG_RESET;
// }

// /*********************************************************************
//  * @fn      	      -
//  *
//  * @brief             -
//  *
//  * @param[in]         -
//  * @param[in]         -
//  * @param[in]         -
//  *
//  * @return            -  none
//  *
//  * @Note              -  none
//  */
// void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
// {
// }

// /*********************************************************************
//  * @fn      	      -
//  *
//  * @brief             -
//  *
//  * @param[in]         -
//  * @param[in]         -
//  * @param[in]         -
//  *
//  * @return            -  none
//  *
//  * @Note              -  none
//  */
// void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEv)
// {
// }
