/*
 * @file stm32f407xx_i2c_driver.c
 *
 *  Created on: May 29, 2021
 *      Author: milesosborne
 */

#include "stm32f407xx.h"
#include "stm32f407xx_i2c_driver.h"

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
void i2c_peripheral_clock_enable(I2C_RegDef_t *pI2Cx)
{
	if (pI2Cx == I2C1)
	{
		I2C1_PCLK_EN();
	}
	else if (pI2Cx == I2C2)
	{
		I2C2_PCLK_EN();
	}
	else
	{
		I2C3_PCLK_EN();
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
void i2c_peripheral_clock_disable(I2C_RegDef_t *pI2Cx)
{
	if (pI2Cx == I2C1)
	{
		I2C1_PCLK_DI();
	}
	else if (pI2Cx == I2C2)
	{
		I2C2_PCLK_DI();
	}
	else
	{
		I2C3_PCLK_DI();
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
uint32_t rcc_get_pckl1_value(void)
{
	uint32_t pclk1;
	uint8_t clksrc;
	uint16_t AHBprescaler;
	uint8_t APBprescaler;

	//Obtain the clock source from RCC_CFGR bits 3:2
	clksrc = ((RCC->CFGR >> 2) & 0x03);

	//1. Determine clock source
	if (clksrc == 0)
	{
		//HSI oscillator selected as system clock
		//HSI = 16MHz default
		pclk1 = 16000000;
	}
	else if (clksrc == 1)
	{
		//HSE oscillator selected as system clock
		//HSE = 8MHz
		pclk1 = 8000000;
	}
	else
	{
		//TODO: PLL selected as as system clock
	}

	//2.Determine AHB prescaler value from RCC_CFGR bits 7:4
	AHBprescaler = ((RCC->CFGR >> 4) & 0x0F);

	switch (AHBprescaler)
	{
	case (0x08):
		//System clock divided by 2
		AHBprescaler = 2;
		break;
	case (0x09):
		//System clock divided by 4
		AHBprescaler = 4;
		break;
	case (0x0A):
		//System clock divided by 8
		AHBprescaler = 8;
		break;
	case (0x0B):
		//System clock divided by 16
		AHBprescaler = 16;
		break;
	case (0x0C):
		//System clock divided by 64
		AHBprescaler = 64;
		break;
	case (0x0D):
		//System clock divided by 128
		AHBprescaler = 128;
		break;
	case (0x0E):
		//System clock divided by 256
		AHBprescaler = 256;
		break;
	case (0x0F):
		//System clock divided by 512
		AHBprescaler = 512;
		break;
	default:
		// System clock not divided
		AHBprescaler = 1;
		break;
	}

	//3. Determine the value of the APB1 prescaler from RCC_CFGR bits 12:10
	APBprescaler = ((RCC->CFGR >> 10) & 0x03);

	switch (APBprescaler)
	{
	case (0x04):
		//AHB clock divided by 2
		APBprescaler = 2;
		break;
	case (0x05):
		//AHB clock divided by 4
		APBprescaler = 4;
		break;
	case (0x06):
		//AHB clock divided by 8
		APBprescaler = 8;
		break;
	case (0x07):
		//AHB clock divided by 16
		APBprescaler = 16;
		break;
	default:
		//AHB clock not divided
		APBprescaler = 1;
		break;
	}

	//4. Divide PCLK by prescaler values and return
	pclk1 /= AHBprescaler;
	pclk1 /= APBprescaler;

	return pclk1;
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
void i2c_init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;

	//1. Enable I2C peripheral clock
	i2c_peripheral_clock_enable(pI2CHandle->pI2Cx);

	// //Reset I2C Peripheral
	i2c_software_reset(pI2CHandle->pI2Cx);

	//Ensure I2C peripheral is disabled before changing configuration
	pI2CHandle->pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_PE);

	//2. Set I2C mode in I2C_CCR
	if (pI2CHandle->I2C_Config.I2C_Mode == I2C_MODE_FM)
	{
		//Set to fast mode
		tempreg |= (1 << I2C_CCR_FS);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY);
		pI2CHandle->pI2Cx->I2C_CCR |= tempreg;
		tempreg = 0;
	}
	else
	{
		//Set to standard mode (default setting)
		pI2CHandle->pI2Cx->I2C_CCR &= ~(1 << I2C_CCR_FS);
	}

	//3. Configure slave device address
	// tempreg |= pI2CHandle->pI2Cx->I2C_OAR1 << 1;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= (1 << 14); //NOTE: Bit 14 should always be kept at 1 by software
	pI2CHandle->pI2Cx->I2C_OAR1 |= tempreg;
	tempreg = 0;

	// //4. Enable Ack Control
	// pI2CHandle->pI2Cx->I2C_CR1 |= (I2C_ACK_ENABLE << I2C_CR1_ACK);

	//5. Configure serial clock speed (SCL)
	//5.1	Set peripheral clock speed in I2C_CR2_FREQ field
	pI2CHandle->pI2Cx->I2C_CR2 |= ((rcc_get_pckl1_value() / 1000000) & (0x3F));

	//5.2	Configure Clock Control Register (I2C_CCR)
	uint16_t CCRvalue;

	//Get Tpclk1 (time period of peripheral clock in nanoseconds)
	if (pI2CHandle->I2C_Config.I2C_Mode == I2C_MODE_SM)
	{
		//CCR configuration in standard mode
		CCRvalue = rcc_get_pckl1_value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
	}
	else if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
	{
		//CCR configuration in Fast mode (DUTY = 0)
		CCRvalue = rcc_get_pckl1_value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
	}
	else
	{
		//CCR configuration in Fast mode (Duty = 1)
		CCRvalue = rcc_get_pckl1_value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
	}
	pI2CHandle->pI2Cx->I2C_CCR |= (CCRvalue & 0xFFF);

	//6. Configure rise time
	float tRiseValue;
	if (pI2CHandle->I2C_Config.I2C_Mode == I2C_MODE_SM)
	{
		//Standard mode configuration
		// NOTE: rise time = (tr(scl_MAX)/Tpclk1)+1
		// NOTE: tr(scl_MAX) is the maximum allowed rise time listed in I2C specification
		// tRiseValue = (1000 / (1000/rcc_get_pckl1_value()*1000000) + 1);
		tRiseValue = (1000 * 1000000);
		tRiseValue = tRiseValue / rcc_get_pckl1_value();
		tRiseValue = 1000 / tRiseValue;
		tRiseValue += 1;
	}
	else
	{
		//Fast mode configuration
		//tRiseValue = (300 / (1000 / rcc_get_pckl1_value() * 1000000) + 1);
		tRiseValue = (1000 * 1000000);
		tRiseValue = tRiseValue / rcc_get_pckl1_value();
		tRiseValue = 300 / tRiseValue;
		tRiseValue += 1;
	}
	pI2CHandle->pI2Cx->I2C_TRISE = ((uint8_t)tRiseValue & 0x3F);

	//7. Enable I2C communication
	i2c_peripheral_enable(pI2CHandle->pI2Cx);
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
void i2c_deinit(I2C_RegDef_t *pI2Cx)
{
}

/*********************************************************************
 * @fn				- i2c_software_reset 
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
void i2c_software_reset(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_SWRST);	 //Software reset I2C peripheral
	pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_SWRST); // Out of reset
	return;
}

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
void i2c_master_write(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t len, uint8_t slaveAddr)
{
	uint32_t tempreg;

	//1. Generate START condition
	i2c_generate_start(pI2CHandle->pI2Cx);

	//2. Confirm START bit generation and clear I2C_SR1_CB
	//NOTE: SB is cleared by reading I2C_SR1 and writing the device address in I2C_DR
	//NOTE: Until SB is cleared, SCL will be stretched (pulled to LOW)
	while (!i2c_get_flag_status(pI2CHandle->pI2Cx, I2C_FLAG_SB))
	{
	}

	tempreg = pI2CHandle->pI2Cx->I2C_SR1;
	//3. Send the address of the slave with r/nw bit set to 0
	i2c_address_phase_write(pI2CHandle->pI2Cx, slaveAddr);

	//4. Wait until address phase is completed
	//NOTE: Clear ADDR flag by reading I2C_SR1 followed by reading I2C_SR2
	while (!i2c_get_flag_status(pI2CHandle->pI2Cx, I2C_FLAG_ADDR))
	{
	}
	i2c_clear_addr_flag(pI2CHandle->pI2Cx);

	while (len > 0)
	{
		//Wait until TxE flag is set
		//NOTE: TxE flag indicates that the data register is empty/ not empty
		while (!i2c_get_flag_status(pI2CHandle->pI2Cx, I2C_FLAG_TXE))
		{
		}

		//Write data into DR
		pI2CHandle->pI2Cx->I2C_DR |= *pTxBuffer;

		len--;
		if (len > 0)
		{
			pTxBuffer++;
		}
	}

	//Wait uhntil TxE and BTF are set, then generate stop condition
	while (!i2c_get_flag_status(pI2CHandle->pI2Cx, I2C_FLAG_TXE))
	{
	}
	while (!i2c_get_flag_status(pI2CHandle->pI2Cx, I2C_FLAG_BTF))
	{
	}

	i2c_generate_stop(pI2CHandle->pI2Cx);
}
#endif

/*********************************************************************
 * @fn				- i2c_master_write
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
void i2c_master_write(I2C_Handle_t *pI2CHandle, I2C_RegDef_t *pI2Cx, uint8_t slaveAddr, uint8_t command, uint8_t len, uint8_t *pData)
{
	uint32_t tempreg = 0;

	//Wait until bus not busy
	while (pI2Cx->I2C_SR2 & I2C_SR2_BUSY)
		;

	// //4. Enable Ack Control
	// pI2Cx->I2C_CR1 |= (I2C_ACK_ENABLE << I2C_CR1_ACK);

	//1. Generate START condition
	i2c_generate_start(pI2Cx);

	//2. Confirm START bit generation and clear I2C_SR1_CB
	//NOTE: SB is cleared by reading I2C_SR1 and writing the device address in I2C_DR
	//NOTE: Until SB is cleared, SCL will be stretched (pulled to LOW)

	while (!i2c_get_flag_status(pI2Cx, I2C_FLAG_SB))
		;
	tempreg = pI2Cx->I2C_SR1;
	tempreg = 0;

	//3. Send the address of the slave with r/nw bit set to 0
	i2c_address_phase_write(pI2Cx, slaveAddr);

	//4. Wait until address phase is completed
	//NOTE: Clear ADDR flag by reading I2C_SR1 followed by reading I2C_SR2
	while (!i2c_get_flag_status(pI2Cx, I2C_FLAG_ADDR))
	{
	}
	i2c_clear_addr_flag(pI2Cx);

	//Wait until TXE flag is set
	while (!i2c_get_flag_status(pI2Cx, I2C_FLAG_TXE))
	{
	}

	//Send command to slave
	pI2Cx->I2C_DR = command;

	//Wait for Byte transfer complete
	while (!i2c_get_flag_status(pI2Cx, I2C_FLAG_BTF))
		;

	while (len > 0)
	{
		//Wait until TxE flag is set
		//NOTE: TxE flag indicates that the data register is empty/ not empty
		while (!i2c_get_flag_status(pI2Cx, I2C_FLAG_TXE))
		{
		}

		//Write data into DR
		pI2Cx->I2C_DR = *pData;

		//Wait until BTF flag is set
		while (!i2c_get_flag_status(pI2Cx, I2C_FLAG_BTF))
			;

		len--;
		if (len > 0)
		{
			pData++;
		}
	}

	//Wait uhntil TxE and BTF are set, then generate stop condition
	while (!i2c_get_flag_status(pI2Cx, I2C_FLAG_TXE))
	{
	}
	while (!i2c_get_flag_status(pI2Cx, I2C_FLAG_BTF))
	{
	}

	i2c_generate_stop(pI2Cx);

	//Clear data register I2C_DR
	pI2Cx->I2C_DR &= 0;
	return;
}

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
void i2c_master_read(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t len, uint8_t slaveAddr)
{
	uint32_t tempreg;

	//1. Generate START condition
	i2c_generate_start(pI2CHandle->pI2Cx);

	//2. Confirm that start generation is completeed by checking the SB flag in I2C_SR1
	//	Note: Until SB is cleared, SCL will be stretched (pulled to LOW)
	while (!i2c_get_flag_status(pI2CHandle->pI2Cx, I2C_FLAG_SB))
	{
	}

	tempreg = pI2CHandle->pI2Cx->I2C_SR1;
	//3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits)
	i2c_address_phase_read(pI2CHandle->pI2Cx, slaveAddr);

	//4. Wait until address phase is completed by checking the ADDR flag in I2C_SR1
	while (!i2c_get_flag_status(pI2CHandle->pI2Cx, I2C_FLAG_ADDR))
	{
	}

	//Procedure to read only 1 byte from slave
	if (len == 1)
	{
		//Disable Acking
		i2c_ack_disable(pI2CHandle->pI2Cx);

		//Generate STOP condition
		i2c_generate_stop(pI2CHandle->pI2Cx);

		//Read data into buffer
		*pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
		pRxBuffer++;

		//Wait until RXNE becomes 1
		while (!i2c_get_flag_status(pI2CHandle->pI2Cx, I2C_FLAG_RXNE))
			;

		//Read data register to stop transmission
		*pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;

		return;
	}
	else if (len > 1)
	{

		//clear the ADDR flag
		i2c_clear_addr_flag(pI2CHandle->pI2Cx);

		//read the data until len becomes zero
		for (uint32_t i = len; i > 0; i--)
		{
			//wait until RXNE becomes 1
			while (!i2c_get_flag_status(pI2CHandle->pI2Cx, I2C_FLAG_RXNE))
				;

			if (i == 2) //last 2 bytes are remaining
			{
				//disable acking
				i2c_ack_disable(pI2CHandle->pI2Cx);

				//generate STOP condition
				i2c_generate_stop(pI2CHandle->pI2Cx);
			}

			//read the data from data register in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;

			//increment the buffer address
			pRxBuffer++;
		}
	}
	//re-enable acking
	i2c_ack_enable(pI2CHandle->pI2Cx);
}
#endif

/*********************************************************************
 * @fn				-i2c_master_read
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
void i2c_master_read(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr, uint8_t command, uint8_t len, uint8_t *pRxBuffer)
{
	uint32_t tempreg;

	//Wait until bus not busy
	while (pI2Cx->I2C_SR2 & I2C_SR2_BUSY)
		;

	//1. Generate START condition
	i2c_generate_start(pI2Cx);

	//2. Confirm that start generation is completeed by checking the SB flag in I2C_SR1
	//	Note: Until SB is cleared, SCL will be stretched (pulled to LOW)
	while (!i2c_get_flag_status(pI2Cx, I2C_FLAG_SB))
	{
	}

	tempreg = pI2Cx->I2C_SR1;
	//3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits)
	i2c_address_phase_write(pI2Cx, slaveAddr);

	//4. Wait until address phase is completed by checking the ADDR flag in I2C_SR1
	while (!i2c_get_flag_status(pI2Cx, I2C_FLAG_ADDR))
	{
	}

	//Send command to slave
	while (!i2c_get_flag_status(pI2Cx, I2C_FLAG_TXE))
	{
	}
	pI2Cx->I2C_DR |= command;

	//Send repeated start condition
	i2c_generate_start(pI2Cx);

	//Clear SB flag
	while (!i2c_get_flag_status(pI2Cx, I2C_FLAG_SB))
	{
	}
	tempreg = pI2Cx->I2C_SR1;

	//Transmit slave address with read operation
	i2c_address_phase_read(pI2Cx, slaveAddr);

	//Clear ADDR flag
	while (!i2c_get_flag_status(pI2Cx, I2C_FLAG_ADDR))
	{
	}
	i2c_clear_addr_flag(pI2Cx);

	//Read data from data register:
	//Procedure to read only 1 byte from slave
	if (len == 1)
	{
		//Disable acking
		i2c_ack_disable(pI2Cx);

		//Wait until RxNE Flag is set
		while (!i2c_get_flag_status(pI2Cx, I2C_FLAG_RXNE))
		{
		}

		//Generate STOP condition
		i2c_generate_stop(pI2Cx);

		*pRxBuffer = pI2Cx->I2C_DR;
		pRxBuffer++;

		//Generate STOP condition
		i2c_generate_stop(pI2Cx);

		//Procedure to read 2 or more bytes
	}
	else
	{
		for (uint32_t i = len; i > 0; i--)
		{
			//Wait until RxNE flag is set
			while (i2c_get_flag_status(pI2Cx, I2C_FLAG_RXNE))
			{
			}

			//Read DR
			*pRxBuffer = pI2Cx->I2C_DR;
			pRxBuffer++;

			if (i == 2)
			{
				//Disable acking
				i2c_ack_disable(pI2Cx);

				//Generate STOP condition
				i2c_generate_stop(pI2Cx);
			}
		}
	}

	//re-enable acking
	i2c_ack_enable(pI2Cx);
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
void i2c_slave_write(I2C_RegDef_t *pI2C, uint8_t data)
{
	pI2C->I2C_DR = data;
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
uint8_t i2c_slave_read(I2C_RegDef_t *pI2C)
{
	return pI2C->I2C_DR;
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
void i2c_clear_addr_flag(I2C_RegDef_t *pI2Cx)
{
	//NOTE: I2C_SR1 ADDR flag is cleared by software reading SR1 register followed reading SR2
	uint32_t dummyRead1 = pI2Cx->I2C_SR1;
	uint32_t dummyRead2 = pI2Cx->I2C_SR2;
	(void)dummyRead1;
	(void)dummyRead2;
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
void i2c_address_phase_write(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr)
{
	uint8_t temp;
	temp = slaveAddr << 1;
	temp &= ~(1); //slaveAddr is slave address + r/nw bit = 0
	pI2Cx->I2C_DR = temp;
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
void i2c_address_phase_read(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr)
{
	slaveAddr = slaveAddr << 1;
	slaveAddr |= 1; //slaveAddr is slave address + r/nw bit = 1
	pI2Cx->I2C_DR = slaveAddr;
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
void i2c_ack_enable(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_ACK);
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
void i2c_ack_disable(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_ACK);
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
void i2c_generate_start(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_START);
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
void i2c_generate_stop(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_STOP);
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
uint8_t i2c_get_flag_status(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if (pI2Cx->I2C_SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
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
void i2c_peripheral_enable(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_PE);
	//pI2cBaseAddress->CR1 |= I2C_CR1_PE_Bit_Mask;
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
void i2c_peripheral_disable(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_PE);
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
void I2C_ApplicationCallback(I2C_Handle_t *pI2CHandle, uint8_t handleEvent)
{
}
