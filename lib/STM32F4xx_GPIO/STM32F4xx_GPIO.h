/**
 * @file STM32F4xx_GPIO.c
 * 
 * @brief GPIO drivers and api for STM32F4xx microcontrollers and development boards
 * 
 * @author Miles Osborne
 * contact: milesosborne182@gmail.com
 * 
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "STM32F4xx_base.h"
#include "STM32F4xx_RCC.h"

/*
 * Handle structure for a GPIO pin
 */
typedef struct
{
	uint8_t GPIO_PinNumber;		  /*!< possible values from @GPIO_PIN_NUMBERS >*/
	uint8_t GPIO_PinMode;		  /*!< possible values from @GPPIO_PIN_MODES >*/
	uint8_t GPIO_PinSpeed;		  /*!< possible values from @GPPIO_PIN_SPEED >*/
	uint8_t GPIO_PinPuPdControll; /*!< possible values from @GPIO_PIN_PU_PD>*/
	uint8_t GPIO_PinOPType;		  /*!< possible values from @GPIO_OUTPUT_TYPE>*/
	uint8_t GPIO_PinAltFuncMode;  /*!< possible values from @GPPIO_PIN_MODES >*/
} GPIO_PinConfig_t;

typedef struct
{
	// pointer to hold the base address of the GPIO peripheral
	GPIO_RegDef_t *pGPIOx;		//This variable holds the base address of the GPIO port to which the pin belongs
	GPIO_PinConfig_t PinConfig; //This hold GPIO pin configuration settings
} GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
*/

#define GPIO_PIN_NO_0 0
#define GPIO_PIN_NO_1 1
#define GPIO_PIN_NO_2 2
#define GPIO_PIN_NO_3 3
#define GPIO_PIN_NO_4 4
#define GPIO_PIN_NO_5 5
#define GPIO_PIN_NO_6 6
#define GPIO_PIN_NO_7 7
#define GPIO_PIN_NO_8 8
#define GPIO_PIN_NO_9 9
#define GPIO_PIN_NO_10 10
#define GPIO_PIN_NO_11 11
#define GPIO_PIN_NO_12 12
#define GPIO_PIN_NO_13 13
#define GPIO_PIN_NO_14 14
#define GPIO_PIN_NO_15 15

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_INPUT 0
#define GPIO_MODE_OUTPUT 1
#define GPIO_MODE_ALT_FUNCTION 2
#define GPIO_MODE_ANALOG 3
#define GPIO_MODE_IT_FT 4  //GPIO pin interrupt mode falling edge trigger
#define GPIO_MODE_IT_RT 5  //GPIO pin interrupt mode rising edge trigger
#define GPIO_MODE_IT_RFT 6 //GPIO pin interrupt mode rising/falling edge trigger
#define INPUT GPIO_MODE_INPUT
#define OUTPUT GPIO_MODE_OUTPUT

/*
 * @GPIO_OUTPUT_TYPE
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP 0 //GPIO pin output mode, push pull config
#define GPIO_OP_TYPE_OD 1 //GPIO pin output mode, open drain config

/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW 0
#define GPIO_SPEED_MEDIUM 1
#define GPIO_SPEED_FAST 2
#define GPIO_SPEED_HIGH 3

/*
 * @GPIO_PIN_PU_PD
 * GPIO pin pull up and pull down configuration macros
 */
#define GPIO_NO_PUPD 0 //No pull-up, no pull-down
#define GPIO_PIN_PU 1  //Pull-up
#define GPIO_PIN_PD 2  //Pull-down

/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN() (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN() (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN() (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN() (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN() (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN() (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN() (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN() (RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN() (RCC->AHB1ENR |= (1 << 8))

/*
 * Clock Disable Macros for GIPOx peripherals
 */
#define GPIOA_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 8))

/*
 * Macros to reset GPIO peripherals
 */
#define GPIOA_REG_RESET()            \
	do                               \
	{                                \
		(RCC->AHB1STR |= (1 << 0));  \
		(RCC->AHB1STR &= ~(1 << 0)); \
	} while (0)
#define GPIOB_REG_RESET()            \
	do                               \
	{                                \
		(RCC->AHB1STR |= (1 << 1));  \
		(RCC->AHB1STR &= ~(1 << 1)); \
	} while (0)
#define GPIOC_REG_RESET()            \
	do                               \
	{                                \
		(RCC->AHB1STR |= (1 << 2));  \
		(RCC->AHB1STR &= ~(1 << 2)); \
	} while (0)
#define GPIOD_REG_RESET()            \
	do                               \
	{                                \
		(RCC->AHB1STR |= (1 << 3));  \
		(RCC->AHB1STR &= ~(1 << 3)); \
	} while (0)
#define GPIOE_REG_RESET()            \
	do                               \
	{                                \
		(RCC->AHB1STR |= (1 << 4));  \
		(RCC->AHB1STR &= ~(1 << 4)); \
	} while (0)
#define GPIOF_REG_RESET()            \
	do                               \
	{                                \
		(RCC->AHB1STR |= (1 << 5));  \
		(RCC->AHB1STR &= ~(1 << 5)); \
	} while (0)
#define GPIOG_REG_RESET()            \
	do                               \
	{                                \
		(RCC->AHB1STR |= (1 << 6));  \
		(RCC->AHB1STR &= ~(1 << 6)); \
	} while (0)
#define GPIOH_REG_RESET()            \
	do                               \
	{                                \
		(RCC->AHB1STR |= (1 << 7));  \
		(RCC->AHB1STR &= ~(1 << 7)); \
	} while (0)
#define GPIOI_REG_RESET()            \
	do                               \
	{                                \
		(RCC->AHB1STR |= (1 << 8));  \
		(RCC->AHB1STR &= ~(1 << 8)); \
	} while (0)

/*****************************************************************************************************
 * 									APIs supported by this driver
 * 					For more information about the APIs check the function defition
 *****************************************************************************************************/

/*
 * Peripheral Clock Setup
 */
void GPIO_clock_enable(GPIO_RegDef_t *pGPIOx);
void GPIO_clock_disable(GPIO_RegDef_t *pGPIOx);
void GPIO_reset(GPIO_RegDef_t *pGPIOx);

/*
 * Init and De-init
 */
void GPIO_init(GPIO_Handle_t *pGPIOHandle);
void GPIO_deinit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write
 */
uint8_t GPIO_read_input_pin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
uint16_t GPIO_read_input_port(GPIO_RegDef_t *pGPIOx); //Reads data from all 16 pins of an GPIO port
void GPIO_write_pin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value);
void GPIO_write_port(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_toggle_output_pin(GPIO_RegDef_t *pGPIOx, uint16_t pinNumber);

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_interrupt_config(uint8_t IRQNumber, uint8_t Enable_Disable);
void GPIO_interrupt_priority_config(uint8_t IRQNumber, uint32_t Priority);
void GPIO_IRQ_handle(uint16_t pinNumber);

/*
 * Other API supported by this peripheral
 */
uint8_t GPIO_BASEADDR_TO_CODE(GPIO_RegDef_t *GPIO_Port);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
