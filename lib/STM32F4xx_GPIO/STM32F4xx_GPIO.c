/**
 * @file STM32F4xx_GPIO.c
 * 
 * @brief GPIO drivers and api for STM32F4xx microcontrollers and development boards
 * 
 * @author Miles Osborne
 * contact: milesosborne182@gmail.com
 * 
 */
#include "STM32F4xx_GPIO.h"

/*
 * Peripheral Clock Setup
 */

/*********************************************************************
 * @fn      		  - GPIO_clock_enable
 *
 * @brief             - Enables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral (pGPIOx)
 * @param[in]         - 
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_clock_enable(GPIO_RegDef_t *pGPIOx)
{
	uint32_t address = (uint32_t)&pGPIOx;
	switch (address)
	{
	case (GPIOA_BASE):
		GPIOA_PCLK_EN();
		break;
	case (GPIOB_BASE):
		GPIOB_PCLK_EN();
		break;
	case (GPIOC_BASE):
		GPIOC_PCLK_EN();
		break;
	case (GPIOD_BASE):
		GPIOD_PCLK_EN();
		break;
	case (GPIOE_BASE):
		GPIOE_PCLK_EN();
		break;
	case (GPIOF_BASE):
		GPIOF_PCLK_EN();
		break;
	case (GPIOG_BASE):
		GPIOG_PCLK_EN();
		break;
	case (GPIOH_BASE):
		GPIOH_PCLK_EN();
		break;
	case (GPIOI_BASE):
		GPIOI_PCLK_EN();
		break;
	}
}

/*********************************************************************
 * @fn      		  - GPIO_clock_disable
 *
 * @brief             - Disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral (pGPIOx)
 * @param[in]         - 
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_clock_disable(GPIO_RegDef_t *pGPIOx)
{
	uint32_t address = (uint32_t)&pGPIOx;
	switch (address)
	{
	case (GPIOA_BASE):
		GPIOA_PCLK_DI();
		break;
	case (GPIOB_BASE):
		GPIOB_PCLK_DI();
		break;
	case (GPIOC_BASE):
		GPIOC_PCLK_DI();
		break;
	case (GPIOD_BASE):
		GPIOD_PCLK_DI();
		break;
	case (GPIOE_BASE):
		GPIOE_PCLK_DI();
		break;
	case (GPIOF_BASE):
		GPIOF_PCLK_DI();
		break;
	case (GPIOG_BASE):
		GPIOG_PCLK_DI();
		break;
	case (GPIOH_BASE):
		GPIOH_PCLK_DI();
		break;
	case (GPIOI_BASE):
		GPIOI_PCLK_DI();
		break;
	}
}

/*********************************************************************
 * @fn      		  - GPIO_reset
 *
 * @brief             - Sets specified GPIO port back to reset values
 *
 * @param[in]         - base address of the gpio peripheral (pGPIOx)
 * @param[in]         - 
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  The GPIO port/register must be reconfigured before being used
 */
void GPIO_reset(GPIO_RegDef_t *pGPIOx)
{
	uint32_t address = (uint32_t)&pGPIOx;
	switch (address)
	{
	case (GPIOA_BASE):
		GPIOA_REG_RESET();
		break;
	case (GPIOB_BASE):
		GPIOB_REG_RESET();
		break;
	case (GPIOC_BASE):
		GPIOC_REG_RESET();
		break;
	case (GPIOD_BASE):
		GPIOD_REG_RESET();
		break;
	case (GPIOE_BASE):
		GPIOE_REG_RESET();
		break;
	case (GPIOF_BASE):
		GPIOF_REG_RESET();
		break;
	case (GPIOG_BASE):
		GPIOG_REG_RESET();
		break;
	case (GPIOH_BASE):
		GPIOH_REG_RESET();
		break;
	case (GPIOI_BASE):
		GPIOI_REG_RESET();
		break;
	}
}

/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - This configures the mode and configuration of a gpio pin
 *
 * @param[in]         - base address of the gpio peripheral (pGPIOx)
 * @param[in]         - ENABLE or DISABLE macros (Enable_Disable)
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_init(GPIO_Handle_t *pGPIOHandle)
{

	//Enable GPIO peripheral clock
	GPIO_clock_enable(pGPIOHandle->pGPIOx);

	uint32_t configuration; //saves the pinMode configuration

	//1. Check if GPIO pin is in an interrupt mode
	if (pGPIOHandle->PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//Non interrupt mode:
		configuration = pGPIOHandle->PinConfig.GPIO_PinMode << (pGPIOHandle->PinConfig.GPIO_PinNumber * 2);
		pGPIOHandle->pGPIOx->MODER |= configuration;
	}
	else
	{
		if (pGPIOHandle->PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1. Configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->PinConfig.GPIO_PinNumber);
			//Clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			;
			//2. Configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->PinConfig.GPIO_PinNumber);
			//Clear the corresponding FTSR bit
			EXTI->FTSR |= (1 << pGPIOHandle->PinConfig.GPIO_PinNumber);
		}
		else
		{
			//3. Configure both the FTSR and RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->PinConfig.GPIO_PinNumber);
		}

		//1. Configure the GPIO port selection in SYSCFG_EXTICR
		SYSCFG_PCLK_EN();
		uint8_t temp1 = pGPIOHandle->PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG->EXTICRx[temp1] |= (portcode << (temp2 * 4));

		//2. Enable the GPIO port selection in IMR
		EXTI->IMR |= 1 << pGPIOHandle->PinConfig.GPIO_PinNumber;
	}

	//2. Configure the speed
	configuration = pGPIOHandle->PinConfig.GPIO_PinSpeed << (pGPIOHandle->PinConfig.GPIO_PinNumber * 2);
	pGPIOHandle->pGPIOx->OSPEEDR |= configuration;

	//3. COnfigure the pupd settings
	configuration = pGPIOHandle->PinConfig.GPIO_PinPuPdControll << (pGPIOHandle->PinConfig.GPIO_PinNumber * 2);
	pGPIOHandle->pGPIOx->PUPDR |= configuration;

	//4. Configure the output type
	configuration = pGPIOHandle->PinConfig.GPIO_PinOPType << (pGPIOHandle->PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= configuration;

	//5. Configure the alt functionality
	if (pGPIOHandle->PinConfig.GPIO_PinMode == GPIO_MODE_ALT_FUNCTION)
	{
		if (pGPIOHandle->PinConfig.GPIO_PinNumber < GPIO_PIN_NO_8)
		{
			configuration = pGPIOHandle->PinConfig.GPIO_PinAltFuncMode << (pGPIOHandle->PinConfig.GPIO_PinNumber * 4);
			pGPIOHandle->pGPIOx->AFRL |= configuration;
		}
		else
		{
			configuration = pGPIOHandle->PinConfig.GPIO_PinAltFuncMode << ((pGPIOHandle->PinConfig.GPIO_PinNumber - 8) * 4);
			pGPIOHandle->pGPIOx->AFRH |= configuration;
		}
	}
}

/*********************************************************************
 * @fn      	      - GPIO_DeInit
 *
 * @brief             - Resets the specified GPIO Port
 *
 * @param[in]         - base address of the GPIO peripheral (pGPIOx)
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  Assuming that reset will wipe all the GPIO register configurations
 */
void GPIO_deinit(GPIO_RegDef_t *pGPIOx)
{
	uint32_t address = (uint32_t)&pGPIOx;
	switch (address)
	{
	case (GPIOA_BASE):
		GPIOA_REG_RESET();
		break;
	case (GPIOB_BASE):
		GPIOB_REG_RESET();
		break;
	case (GPIOC_BASE):
		GPIOC_REG_RESET();
		break;
	case (GPIOD_BASE):
		GPIOD_REG_RESET();
		break;
	case (GPIOE_BASE):
		GPIOE_REG_RESET();
		break;
	case (GPIOF_BASE):
		GPIOF_REG_RESET();
		break;
	case (GPIOG_BASE):
		GPIOG_REG_RESET();
		break;
	case (GPIOH_BASE):
		GPIOH_REG_RESET();
		break;
	case (GPIOI_BASE):
		GPIOI_REG_RESET();
		break;
	}
}

/*********************************************************************
 * @fn      	      -GPIO_ReadFromInputPin
 *
 * @brief             -REads value from specified pin
 *
 * @param[in]         -base address of the GPIO peripheral (pGPIOx)
 * @param[in]         -pin number (pinNumber)
 * @param[in]         -
 *
 * @return            -  unsiged 8 bit number representing the value read from input, 0 or 1
 *
 * @Note              -  none
 */
uint8_t GPIO_read_input_pin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
{
	uint16_t value;
	value = pGPIOx->IDR;

	//1. Shift bits in desired range to LSB
	value >>= pinNumber;

	//2. Extract the first / LSB bit position
	uint8_t output = (uint8_t)(value & 0x00000001);
	return output;
}

/*********************************************************************
 * @fn      	      -GPIO_ReadFromInputPort
 *
 * @brief             -Reads data from all 16 pins of an GPIO port
 *
 * @param[in]         -base address of the GPIO peripheral (pGPIOx)
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -returns an unsigned 16 bit number
 *
 * @Note              -  none
 */
uint16_t GPIO_read_input_port(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value = (uint16_t)pGPIOx->IDR;
	return value;
}

/*********************************************************************
 * @fn      	      -GPIO_WriteToOuputPin
 *
 * @brief             -Writes specified value to GPIO Pin
 *
 * @param[in]         -base address of the GPIO peripheral (pGPIOx)
 * @param[in]         -pin number (pinNumber)
 * @param[in]         -value to be write (write)
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_write_pin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value)
{
	if (value >= GPIO_PIN_SET)
	{
		//Write 1 to the output data register at the designated pin number
		pGPIOx->ODR |= (1 << pinNumber);
	}
	else
	{
		//Write 0 to the output data register at the designated pin number
		pGPIOx->ODR &= (0 << pinNumber);
	}
}

/*********************************************************************
 * @fn      	      -GPIO_WriteToOutputPort
 *
 * @brief             -Write a value to all 16 pins of GPIO port
 *
 * @param[in]         -base address of the GPIO peripheral (pGPIOx)
 * @param[in]         -value to be write (write)
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_write_port(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = (uint32_t)value;
}

/*********************************************************************
 * @fn      	      -
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
void GPIO_toggle_output_pin(GPIO_RegDef_t *pGPIOx, uint16_t pinNumber)
{
	pGPIOx->ODR ^= (1 << pinNumber);
}

/*********************************************************************
 * @fn      		  - GPIO_InterruptConfig
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
void GPIO_interrupt_config(uint8_t IRQNumber, uint8_t Enable_Disable)
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
 * @fn      		  - GPIO_InterruptPriorityConfig
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
void GPIO_interrupt_priority_config(uint8_t IRQNumber, uint32_t Priority)
{
	// Find ipr register
	uint8_t iprx = IRQNumber / 4;

	//Find priority field
	uint8_t iprxField = IRQNumber % 4;

	uint8_t shiftAmount = (8 * iprxField) + (8 - NO_PR_BITS_IMPLEMENTED);

	//Set priority
	volatile uint32_t *IPRregister = NVIC_IPR_BASE_ADDR;
	*(IPRregister + iprx) |= (Priority << shiftAmount);
}

/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
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
void GPIO_IRQ_handle(uint16_t pinNumber)
{
	//Clear the EXTI PR register corresponding to the pin number
	if (EXTI->PR & (1 << pinNumber))
	{
		//clear
		EXTI->PR |= (1 << pinNumber);
	}
}

/*********************************************************************
 * @fn      		  - GPIO_BASEADDR_TO_CODE
 *
 * @brief             - Takes the GPIO Portx address and converts it to the EXTI interface code
 *
 * @param[in]         - Base address for the GPIO port (GPIO_Port)
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  8 bit unsigned integer representing the EXTI code
 *
 * @Note              -  none
 */
uint8_t GPIO_BASEADDR_TO_CODE(GPIO_RegDef_t *GPIO_Port)
{
	uint32_t address = (uint32_t *)&GPIO_Port;
	if (address == GPIOA_BASE)
	{
		return 0;
	}
	else if (address == GPIOB_BASE)
	{
		return 1;
	}
	else if (address == GPIOC_BASE)
	{
		return 2;
	}
	else if (address == GPIOD_BASE)
	{
		return 3;
	}
	else if (address == GPIOE_BASE)
	{
		return 4;
	}
	else if (address == GPIOF_BASE)
	{
		return 5;
	}
	else if (address == GPIOG_BASE)
	{
		return 6;
	}
	else if (address == GPIOH_BASE)
	{
		return 7;
	}
	else if (address == GPIOI_BASE)
	{
		return 8;
	}
}
