/** @file STM32F4xx_SPI.h
 * 
 * @brief SPI drivers for STM32F4 MCUs
 *
 * @par
 * 
 *  Created on: Mar 25, 2021
 *      Author: milesosborne
 */

#include "STM32F4xx_SPI.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

/*********************************************************************
 * @fn				- spi_peripheral_clock_enable
 *
 * @brief			- Enables the peripheral clock for the SPI interface
 *
 * @param[in]		- pSPIx, Struct containing SPI interface and location of SPI registers
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-  none
 *
 * @Note			-  none
 */
void spi_peripheral_clock_enable(SPI_RegDef_t *pSPIx)
{
	if (pSPIx == SPI1)
	{
		SPI1_PCLK_EN();
	}
	else if (pSPIx == SPI2)
	{
		SPI2_PCLK_EN();
	}
	else if (pSPIx == SPI3)
	{
		SPI3_PCLK_EN();
	}
}

/*********************************************************************
 * @fn				- spi_peripheral_clock_disable
 *
 * @brief			- Disables the peripheral clock for the SPI interface
 *
 * @param[in]		- pSPIx, Struct containing SPI interface and location of SPI registers
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-  none
 *
 * @Note			-  none
 */
void spi_peripheral_clock_disable(SPI_RegDef_t *pSPIx)
{
	if (pSPIx == SPI1)
	{
		SPI1_PCLK_DI();
	}
	else if (pSPIx == SPI2)
	{
		SPI2_PCLK_DI();
	}
	else if (pSPIx == SPI3)
	{
		SPI3_PCLK_DI();
	}
}

/*********************************************************************
 * @fn				-
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-  none
 *
 * @Note			-  none
 */
void spi_init(SPI_Handle_t *pSPIHandle)
{
	//Enable SPI Periheral clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//Configure SPI_CR1 register
	uint32_t tempreg = 0;

	//1. configure the device mode (slave or master)
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. Configure the bust config
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode (bit 14) should be cleared
		tempreg &= ~(1 << 15);
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		tempreg |= (1 << 15);
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//bidi mode should be cleared
		tempreg &= ~(1 << 15);
		//RXONLY (bit 10) must be set
		tempreg |= (1 << 10);
	}

	//3. Configure the spi serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_Speed << SPI_CR1_BR;

	//4. Configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. Configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6. Configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	//Save configuration to SPI_CR1 register
	pSPIHandle->pSPIx->SPI_CR1 = tempreg;
}

/**
 * NOTE: Is a deinit function needed?
 */
#if 0
/*********************************************************************
 * @fn				-
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-  none
 *
 * @Note			-  none
 */
void spi_DeInit(SPI_RegDef_t *pSPIx)
{
}
#endif

/*********************************************************************
 * @fn				-
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-  none
 *
 * @Note			-  none
 */

void spi_transmit_data(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{
	//Transmit data while length is not 0
	while (len > 0)
	{
		//Wait until the Tx buffer is empty before proceeding
		while (!SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG))
		{
			//Deterine if the DFF is 16bit (1) or 8bit (0)
			if ((pSPIx->SPI_CR1) & (1 << SPI_CR1_DFF))
			{
				pSPIx->SPI_DR = *((uint16_t *)pTxBuffer);
				len--;
				len--;
				(uint16_t *)pTxBuffer++;
			}
			else
			{
				pSPIx->SPI_DR = *pTxBuffer;
				len--;
				pTxBuffer++;
			}
		}
	}
	return;
}

/*********************************************************************
 * @fn				-
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-  none
 *
 * @Note			-  none
 */

void spi_recieve_data(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len)
{
	while (len > 0)
	{
		//1. wait until RXNE flag is set
		while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG))
		{
			if (pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
			{
				//16 bit DFF
				//1. Load the data from DR to RxBuffer address
				*((uint16_t *)pRxBuffer) = pSPIx->SPI_DR;
				len -= 2;
				(uint16_t *)pRxBuffer++;
			}
			else
			{
				//8 bit DFF
				//1. Load the data from DR to RxBuffer address
				pRxBuffer = pSPIx->SPI_DR;
				len--;
				pRxBuffer++;
			}
		}
	}
}

/*********************************************************************
 * @fn				-
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-  none
 *
 * @Note			-  none
 */
uint8_t SPI_TransmitDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->TxState;

	if (state != SPI_BUSY_IN_RX)
	{

		/*
		 *1. Save the Tx buffer address and Len information in some global variables
		 */
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = len;

		/*
		 * 2. Mark the SPI state as busy in transmission so that
		 * no other code can take over same SPI peripheral until transmission is over
		 */
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		/*
		 * 3. Enable the TXEIE control bit to get interrupt whenever the TXE flag is set
		 * in SPI_SR
		 */
		pSPIHandle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_TXEIE);

		/*
		 * 4. Data transmission will be handled by the ISR code (implemented later)
		 */
	}

	return state;
}

//NOTE: Revisit interrupt based api at later date
#if 0 
/*********************************************************************
 * @fn				-
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-  none
 *
 * @Note			-  none
 */

uint8_t SPI_RecieveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->RxState;

	if (state != SPI_BUSY_IN_TX)
	{

		/*
		 *1. Save the Rx buffer address and Len information in some global variables
		 */
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = len;

		/*
		 * 2. Mark the SPI state as busy in receiving so that
		 * no other code can take over same SPI peripheral until transmission is over
		 */
		pSPIHandle->RxState = SPI_BUSY_IN_RX;
		/*
		 * 3. Enable the RXNEIE control bit to get interrupt whenever the RXNE flag is set
		 * in SPI_SR
		 */
		pSPIHandle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_RXNEIE);

		/*
		 * 4. Data transmission will be handled by the ISR code (implemented later)
		 */
	}
	return state;
}

/*********************************************************************
 * @fn				-
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-  none
 *
 * @Note			-  none
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if (pSPIx->SPI_SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*********************************************************************
 * @fn      		  - SPI_IRQInterruptConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t Enable_Disable)
{
	if (Enable_Disable == ENABLE)
	{
		if (IRQNumber <= 31)
		{
			//Set ISER0 Register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber <= 63)
		{
			//Set ISER1 Register
			*NVIC_ISER1 |= (1 << IRQNumber % 32);
		}
		else if (IRQNumber > 63 && IRQNumber <= 95)
		{
			//Set ISER2 Register
			*NVIC_ISER2 |= (1 << IRQNumber % 64);
		}
	}
	else
	{
		if (IRQNumber <= 31)
		{
			//Set ISER0 Register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber <= 63)
		{
			//SEt ISER1 Register
			*NVIC_ICER1 |= (1 << IRQNumber % 32);
		}
		else if (IRQNumber > 63 && IRQNumber <= 95)
		{
			//Set ISER2 Register
			*NVIC_ICER2 |= (1 << IRQNumber % 64);
		}
	}
}

/*********************************************************************
 * @fn      		  - SPI_IRQPriorityConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// Find ipr register
	uint8_t iprx = IRQNumber / 4;

	//Find priority field
	uint8_t iprxField = IRQNumber % 4;

	uint8_t shiftAmount = (8 * iprxField) + (8 - NO_PR_BITS_IMPLEMENTED);

	//Set priority
	volatile uint32_t *IPRregister = NVIC_IPR_BASE_ADDR;
	*(IPRregister + iprx) |= (IRQPriority << shiftAmount);
}

/*********************************************************************
 * @fn				-
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-  none
 *
 * @Note			-  none
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1, temp2;

	//1. Check for TXE
	temp1 = pSPIHandle->pSPIx->SPI_SR & (1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & (1 << SPI_CR2_TXEIE);

	if (temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle(pSPIHandle);
	}

	//2. Check for RXNE
	temp1 = pSPIHandle->pSPIx->SPI_SR & (1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & (1 << SPI_CR2_RXNEIE);

	if (temp1 && temp2)
	{
		//handle RXNE
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	//3. Check for ovr flag
	temp1 = pSPIHandle->pSPIx->SPI_SR & (1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & (1 << SPI_CR2_ERRIE);

	if (temp1 && temp2)
	{
		//handle ovr error
		spi_ovr_err_interrupt_handle(pSPIHandle);
	}
}
#endif

/*********************************************************************
 * @fn				-spi_peripheral_enable
 *
 * @brief			-Enables the SPI peripheral for data transmissions
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-  none
 *
 * @Note			-  none
 */
void spi_peripheral_enable(SPI_RegDef_t *pSPIx)
{
	pSPIx->SPI_CR1 |= (1 << SPI_CR1_SPE);
}

/*********************************************************************
 * @fn				-spi_peripheral_disable
 *
 * @brief			-Disables the SPI peripheral
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-  none
 *
 * @Note			-  none
 */
void spi_peripheral_disable(SPI_RegDef_t *pSPIx)
{
	pSPIx->SPI_CR1 &= ~(0 << SPI_CR1_SPE);
}

/*********************************************************************
 * @fn				-
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-  none
 *
 * @Note			-  none
 */
void spi_ssi_config(SPI_RegDef_t *pSPIx, uint8_t Enable_Disable)
{
	if (Enable_Disable == ENABLE)
	{
		pSPIx->SPI_CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->SPI_CR1 &= ~(0 << SPI_CR1_SSI);
	}
}

/**
 * Interrupt based api will be completed at a later datae
 */
#if 0
/*********************************************************************
 * @fn				-
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-  none
 *
 * @Note			-  none
 */
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//Deterine if the DFF is 16bit (1) or 8bit (0)
	if ((pSPIHandle->pSPIx->SPI_CR1) & (1 << SPI_CR1_DFF))
	{
		pSPIHandle->pSPIx->SPI_DR = *((uint16_t *)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t *)pSPIHandle->pTxBuffer++;
	}
	else
	{
		pSPIHandle->pSPIx->SPI_DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if (!pSPIHandle->TxLen)
	{
		//TxLen is zero, close spi communication and inform the application
		pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_TXEIE); //this prevents  interrupts from setting up the TXE flag
		pSPIHandle->pTxBuffer = NULL;
		pSPIHandle->TxLen = 0;
		pSPIHandle->TxState = SPI_READY;
		SPI_ApplicationCallback(pSPIHandle, SPI_EVENT_TX_COMPLETE);
	}
}

/*********************************************************************
 * @fn				-
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-  none
 *
 * @Note			-  none
 */
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if (pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
	{
		//16 bit DFF
		//1. Load the data from DR to RxBuffer address
		*((uint16_t *)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->SPI_DR;
		pSPIHandle->RxLen -= 2;
		(uint16_t *)pSPIHandle->pRxBuffer++;
	}
	else
	{
		//8 bit DFF
		//1. Load the data from DR to RxBuffer address
		pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->SPI_DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if (!pSPIHandle->RxLen)
	{
		//RxLen is zero, close communication and inform the application
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationCallback(pSPIHandle, SPI_EVENT_RX_COMPLETE);
	}
}

/*********************************************************************
 * @fn				-
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-  none
 *
 * @Note			-  none
 */
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//1. Clear the ovr flag
	if (pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		SPI_ClearOVRFlag(pSPIHandle->pSPIx);
	}

	//2. Inform the application
	SPI_ApplicationCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}
#endif

/*********************************************************************
 * @fn				-spi_get_flag_status
 *
 * @brief			-Obtains the status of SPI peripheral flags
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-  none
 *
 * @Note			-  none
 */
uint8_t spi_get_flag_status(SPI_RegDef_t *pSPIx, uint16_t FlagName)
{
	uint16_t tempreg = FlagName;
	uint16_t SPI_SR = pSPIx->SPI_SR;
	if(SPI_SR & tempreg)
	{
		return 1;
	}else{
		return 0;
	}
}

/*********************************************************************
 * @fn				-spi_clear_ovr_flag
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-  none
 *
 * @Note			-  none
 */
void spi_clear_ovr_flag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->SPI_DR;
	temp = pSPIx->SPI_SR;
	(void)temp;
}

/*********************************************************************
 * @fn				-SPI_CloseTransmission
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-  none
 *
 * @Note			-  none
 */
void spi_close_transmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_TXEIE); //this prevents  interrupts from setting up the TXE flag
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

/*********************************************************************
 * @fn				-SPI_CloseReception
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-  none
 *
 * @Note			-  none
 */
void spi_close_reception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_RXNEIE); //Prevents interrupts from setting the RXNEIE flag
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

/*********************************************************************
 * @fn				-SPI_ApplicationCallback
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-  none
 *
 * @Note			-  This is a weak implementation, the application may override this function
 */
__attribute__((weak)) void SPI_ApplicationCallback(SPI_Handle_t *pSPIHandle, uint8_t handleEvent) {}
