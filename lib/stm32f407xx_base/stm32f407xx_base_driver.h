/** @file stm32f407xx_base_driver.h
 * 
 * @brief Header file for stm32f4xx boards containing peripheral register locations
 * and functions to enable peripheral clocks
 *
 * @par
 * 
 *  Created on: Feb 18, 2021
 *      Author: milesosborne
 */

#include <stdint.h>
#include <stddef.h>

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

//Some generic macros
#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET
#define FLAG_RESET RESET
#define FLAG_SET SET

/*
 * ARM Cortex M4 Processor Specific Peripheral
 */

/* NVIC ISERx register  */
#define NVIC_ISER0 ((volatile uint32_t *)0xE00E100)
#define NVIC_ISER1 ((volatile uint32_t *)0xE00E104)
#define NVIC_ISER2 ((volatile uint32_t *)0xE00E108)
#define NVIC_ISER3 ((volatile uint32_t *)0xE00E10C)
#define NVIC_ISER4 ((volatile uint32_t *)0xE00E110)
#define NVIC_ISER5 ((volatile uint32_t *)0xE00E114)
#define NVIC_ISER6 ((volatile uint32_t *)0xE00E118)
#define NVIC_ISER7 ((volatile uint32_t *)0xE00E11C)

/* NVIC ICERx register */
#define NVIC_ICER0 ((volatile uint32_t *)0xE000E180)
#define NVIC_ICER1 ((volatile uint32_t *)0xE000E184)
#define NVIC_ICER2 ((volatile uint32_t *)0xE000E188)
#define NVIC_ICER3 ((volatile uint32_t *)0xE000E18C)
#define NVIC_ICER4 ((volatile uint32_t *)0xE000E190)
#define NVIC_ICER5 ((volatile uint32_t *)0xE000E194)
#define NVIC_ICER6 ((volatile uint32_t *)0xE000E198)
#define NVIC_ICER7 ((volatile uint32_t *)0xE000E19C)

/* NVIC IPRx register */
#define NVIC_IPR_BASE_ADDR ((volatile uint32_t *)0xE000E400)
#define NO_PR_BITS_IMPLEMENTED 4

/*
 * Memory Map
 */

/* Base address of Flash and SRAM Memories */
#define FLASH_BASEADDR 0x08000000U
#define SRAM1_BASEADDR 0x20000000U
#define SRAM2_BASEADDR 0x2001C000U
#define ROM 0x1FFF0000U
#define SRAM SRAM1_BASEADDR

/* AHBx and APBx Peripheral base addresses */
#define APB1PERIPH_BASE 0x40000000U
#define APB2PERIPH_BASE 0x40010000U
#define AHB1PERIPH_BASE 0x40020000U
#define AHB2PERIPH_BASE 0x50000000U

/* AHB1 Peripheral base addresses (AHB1PERIPH + address offset) */
#define GPIOA_BASE (AHB1PERIPH_BASE + 0x00000000U)
#define GPIOB_BASE (AHB1PERIPH_BASE + 0x00000400U)
#define GPIOC_BASE (AHB1PERIPH_BASE + 0x00000800U)
#define GPIOD_BASE (AHB1PERIPH_BASE + 0x00000C00U)
#define GPIOE_BASE (AHB1PERIPH_BASE + 0x00001000U)
#define GPIOF_BASE (AHB1PERIPH_BASE + 0x00001400U)
#define GPIOG_BASE (AHB1PERIPH_BASE + 0x00001800U)
#define GPIOH_BASE (AHB1PERIPH_BASE + 0x00001C00U)
#define GPIOI_BASE (AHB1PERIPH_BASE + 0x00002000U)
#define GPIOJ_BASE (AHB1PERIPH_BASE + 0x00002400U)
#define GPIOK_BASE (AHB1PERIPH_BASE + 0x00002800U)
#define CRC_BASE (AHB1PERIPH_BASE + 0x00003000U)
#define RCC_BASE (AHB1PERIPH_BASE + 0x00003800U)
#define FLASH_INTERFACE_BASE (AHB1PERIPH_BASE + 0x00003C00U)
#define BKPSRAM_BASE (AHB1PERIPH_BASE + 0x00004000U)
#define DMA1_BASE (AHB1PERIPH_BASE + 0x00006000U)
#define DMA2_BASE (AHB1PERIPH_BASE + 0x00006400U)
#define ETHERNET_MAC_BASE (AHB1PERIPH_BASE + 0x00008000U)
#define DMA2D_BASE (AHB1PERIPH_BASE + 0x0000B000U)
#define USB_HS_BASE (AHB1PERIPH_BASE + 0x00040000U)

/* APB1 peripheral base addresses (APB1PERIPH_BASE + address offset) */
#define UART8_BASE (APB1PERIPH_BASE + 0x00007C00U)
#define UART7_BASE (APB1PERIPH_BASE + 0x00007800U)
#define I2C1_BASE (APB1PERIPH_BASE + 0x00005400U)
#define I2C2_BASE (APB1PERIPH_BASE + 0x00005800U)
#define I2C3_BASE (APB1PERIPH_BASE + 0x00005C00U)
#define USART2_BASE (APB1PERIPH_BASE + 0x00004400U)
#define USART3_BASE (APB1PERIPH_BASE + 0x00004800U)
#define UART4_BASE (APB1PERIPH_BASE + 0x00004C00U)
#define USART5_BASE (APB1PERIPH_BASE + 0x00005000U)
#define SPI2_BASE (APB1PERIPH_BASE + 0x00003800U)
#define SPI3_BASE (APB1PERIPH_BASE + 0x00003C00U)

/* APB2 peripheral base addresses (APB2PERIPH_BASE + address offset) */
#define SPI1_BASE (APB2PERIPH_BASE + 0x00002C00U)
#define SPI4_BASE (APB2PERIPH_BASE + 0x00003000U)
#define USART1_BASE (APB2PERIPH_BASE + 0x00001000U)
#define USART6_BASE (APB2PERIPH_BASE + 0x00001400U)
#define SYSCFG_BASE (APB2PERIPH_BASE + 0x00003400U)
#define EXTI_BASE (APB2PERIPH_BASE + 0x00003C00U)

/*
 * RCC configuration 
 */

/* RCC register definition */
typedef struct
{
	volatile uint32_t CR;			// RCC clock control register										Address offset: 0x00
	volatile uint32_t PLLCFGR;		// RCC PLL configuration register									Address offset: 0x04
	volatile uint32_t CFGR;			// RCC clock configuration register									Address offset: 0x08
	volatile uint32_t CIR;			// RCC clock interrupt register										Address offset: 0x0C
	volatile uint32_t AHB1STR;		// RCC AHB1 peripheral reset register								Address offset: 0x10
	volatile uint32_t AHB2RSTR;		// RCC AHB2 peripheral reset register								Address offset: 0x14
	volatile uint32_t AHB3RSTR;		// RCC AHB3 peripheral reset register								Address offset: 0x18
	volatile uint32_t RESERVED0;	// Reserved
	volatile uint32_t APB1RSTR;		// RCC APB1 peripheral reset register								Address offset: 0x20
	volatile uint32_t APB2RSTR;		// RCC APB2 peripheral reset register								Address offset: 0x24
	volatile uint32_t RESERVED1[2]; // Reserved
	volatile uint32_t AHB1ENR;		// RCC AHB1 peripheral clock enable register						Address offset: 0x30
	volatile uint32_t AHB2ENR;		// RCC AHB2 peripheral clock enable register						Address offset: 0x34
	volatile uint32_t AHB3ENR;		// RCC APB1 peripheral clock enable register						Address offset: 0x38
	volatile uint32_t RESERVED2;	// Reserved
	volatile uint32_t APB1ENR;		// RCC APB1 peripheral clock enable register						Address offset: 0x40
	volatile uint32_t APB2ENR;		// RCC APB2 peripheral clock enable register						Address offset: 0x44
	volatile uint32_t RESERVED3[2]; // Reserved
	volatile uint32_t AHB1LPENR;	// RCC AHB1 peripheral clock enable in low power mode register		Address offset: 0x50
	volatile uint32_t AHB2LPENR;	// RCC AHB2 peripheral clock enable in low power mode register		Address offset: 0x54
	volatile uint32_t AHB3LPENR;	// RCC AHB3 peripheral clock enable in low power mode register		Address offset: 0x58
	volatile uint32_t RESERVED4;	// Reserved
	volatile uint32_t APB1LPENR;	// RCC APB1 peripheral clock enable in low power mode register		Address offset: 0x60
	volatile uint32_t APB2LPENR;	// RCC APB2 peripheral clock enabled in low power mode register		Address offset: 0x64
	volatile uint32_t RESERVED5[2]; // Reserved
	volatile uint32_t BDCR;			// RCC Backup domain control register								Address offset: 0x70
	volatile uint32_t CSR;			// RCC clock control & status register								Address offset: 0x74
	volatile uint32_t RESERVED6[2]; // Reserved
	volatile uint32_t SSCGR;		// RCC spread spectrum clock generation register					Address offset: 0x80
	volatile uint32_t PLLI2SCFGR;	// RCC PLLI2S configuration register								Address offset: 0x84
} RCC_RegDef_t;

/* RCC peripheral definition */
#define RCC ((RCC_RegDef_t *)RCC_BASE)

/*
 * GPIO configuration 
 */
typedef struct
{
	volatile uint32_t MODER;   //GPIO port mode register						Address offset: 0x00
	volatile uint32_t OTYPER;  //GPIO port output type register				Address offset: 0x04
	volatile uint32_t OSPEEDR; //GPIO port output speed register				Address offset: 0x08
	volatile uint32_t PUPDR;   //GPIO port pull up/down register				Address offset: 0x0C
	volatile uint32_t IDR;	   //GPIO port input data register					Address offset: 0x10
	volatile uint32_t ODR;	   //GPIO port output data register				Address offset: 0x14
	volatile uint32_t BSRR;	   //GPIO port bit set/reset register				Address offset: 0x18
	volatile uint32_t LCKR;	   //GPIO port configuration lock register			Address offset: 0x1C
	volatile uint32_t AFRL;	   //GPIO port alternate function LOW register		Address offset: 0x20
	volatile uint32_t AFRH;	   //GPIO port alternate function HIGH register	Address offset: 0x24
} GPIO_RegDef_t;

/* GPIO peripheral definition (Periphral base address typecasted to GPIO_RegDef_t) */
#define GPIOA ((GPIO_RegDef_t *)GPIOA_BASE)
#define GPIOB ((GPIO_RegDef_t *)GPIOB_BASE)
#define GPIOC ((GPIO_RegDef_t *)GPIOC_BASE)
#define GPIOD ((GPIO_RegDef_t *)GPIOD_BASE)
#define GPIOE ((GPIO_RegDef_t *)GPIOE_BASE)
#define GPIOF ((GPIO_RegDef_t *)GPIOF_BASE)
#define GPIOG ((GPIO_RegDef_t *)GPIOG_BASE)
#define GPIOH ((GPIO_RegDef_t *)GPIOH_BASE)
#define GPIOI ((GPIO_RegDef_t *)GPIOI_BASE)
#define GPIOJ ((GPIO_RegDef_t *)GPIOJ_BASE)
#define GPIOK ((GPIO_RegDef_t *)GPIOK_BASE)

/*
 * SYSCFG register definition
 */
typedef struct
{
	volatile uint32_t MEMRMP;	  //SYSCFG Memory remap register							Address offset: 0x00
	volatile uint32_t PMC;		  //SYSCFG peripheral mode configuration register			Address offset: 0x04
	volatile uint32_t EXTICRx[4]; //SYSCFG External interrupt configuration register 1-4	Address offset: 0x08 - 0x14
	volatile uint32_t CMPCR;	  //Compensation cell controll register					Address offset: 0x20
} SYSCFG_RegDef_t;
#define SYSCFG ((SYSCFG_RegDef_t *)SYSCFG_BASE)

/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15))
//#define SPI4_PCLK_EN() (RCC->APB1ENR |= (1 << ))
//#define SPI5_PCLK_EN() (RCC->APB2ENR |= ())#define GPIOB_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 1))
//#define SPI6_PCLK_EN() (RCC->APB2ENR |= ())

/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCLK_EN() (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN() (RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN() (RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN() (RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN() (RCC->APB2ENR |= (1 << 5))
//#define UART7_PCLK_EN //NOT LISTED IN APB1ENR
//#define UART8_PCLK_EN //NOT LISTED IN APB1ENR

/*
 * Clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))


/*
 * Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI() (RCC->APB2ENR |= ~(1 << 12))
#define SPI2_PCLK_DI() (RCC->APB1ENR |= ~(1 << 14))
#define SPI3_PCLK_DI() (RCC->APB1ENR |= ~(1 << 15))

/*
 * Clock Disable Macros for USARTx peripherals
 */
#define USART1_PCLK_DI() (RCC->APB2ENR |= ~(1 << 4))
#define USART2_PCLK_DI() (RCC->APB1ENR |= ~(1 << 17))
#define USART3_PCLK_DI() (RCC->APB1ENR |= ~(1 << 18))
#define UART4_PCLK_DI() (RCC->APB1ENR |= ~(1 << 19))
#define UART5_PCLK_DI() (RCC->APB1ENR |= ~(1 << 20))
#define USART6_PCLK_DI() (RCC->APB2ENR |= ~(1 << 5))

/*
 * Clock Disable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_DI() (RCC->APB2ENR |= ~(1 << 14))


/*
 * EXTI configuration 
 */

/* EXTI register definition  */
typedef struct
{
	volatile uint32_t IMR;	 //Interrupt mask register				Address offset: 0x00
	volatile uint32_t EMR;	 //Event mask register 					Address offset: 0x04
	volatile uint32_t RTSR;	 //Rising trigger selection register		Address offset: 0x08
	volatile uint32_t FTSR;	 //Falling trigger selection 			Address offset: 0x0C
	volatile uint32_t SWIER; //Software interrupt event register		Address offset: 0x10
	volatile uint32_t PR;	 //Pending register	 					Address offset: 0x14
} EXTI_RegDef_t;
#define EXTI ((EXTI_RegDef_t *)EXTI_BASE)

/*
 * EXTI Line IRQ Numbers:
 */
#define IRQ_NO_EXTI0 6		//EXTI Line0 interrupt
#define IRQ_NO_EXTI1 7		//EXTI Line1 interrupt
#define IRQ_NO_EXTI2 8		//EXTI Line2 interrupt
#define IRQ_NO_EXTI3 9		//EXTI Line3 interrupt
#define IRQ_NO_EXTI4 10		//EXTI Line4 interrupt
#define IRQ_NO_EXTI9_5 23	//EXTI Line[9:5] interrupts
#define IRQ_NO_EXTI15_10 40 //EXTI Line[15:10] interrupts

/*
 * SPI Interrupt IRQ Numbers:
 */
#define IRQ_NO_SPI1 35 //SPI1 global interrupt
#define IRQ_NO_SPI2 36 //SPI2 global interrupt
#define IRQ_NO_SPI3 51 //SPI3 global interrupt

/*
 * I2C Interrupt IRQ Numbers:
 */
#define IRQ_NO_I2C1_EV 31 //I2C1 event interrupt
#define IRQ_NO_I2C1_ER 32 //I2C1 error interrupt
#define IRQ_NO_I2C2_EV 33 //I2C2 event interrupt
#define IRQ_NO_I2C2_ER 34 //I2C2 error interrupt
#define IRQ_NO_I2C3_EV 73 //I2C3 event interrupt
#define IRQ_NO_I2C3_ER 74 //I2C3 error interrupt


/*
 * I2C register definition
 */
typedef struct
{
	uint32_t I2C_CR1;	//I2C Control register 1
	uint32_t I2C_CR2;	//I2C Control register 2
	uint32_t I2C_OAR1;	//I2C Own address register 1
	uint32_t I2C_OAR2;	//I2C Own address register 2
	uint32_t I2C_DR;	//I2C Data register
	uint32_t I2C_SR1;	//I2C Status register 1
	uint32_t I2C_SR2;	//I2C Status register 2
	uint32_t I2C_CCR;	//I2C Clock control register
	uint32_t I2C_TRISE; //I2C TRISE register
	uint32_t I2C_FLTR;	//I2C FLTR register
} I2C_RegDef_t;

#define I2C1 ((I2C_RegDef_t *)I2C1_BASE)
#define I2C2 ((I2C_RegDef_t *)I2C2_BASE)
#define I2C3 ((I2C_RegDef_t *)I2C3_BASE)

/*
 * USART register definition
 */
typedef struct
{
	uint32_t USART_SR;	 //Status register
	uint32_t USART_DR;	 //Data register
	uint32_t USART_CR1;	 //Baud rate register
	uint32_t USART_CR2;	 //Control register 2
	uint32_t USART_CR3;	 //Control register 3
	uint32_t USART_GTPR; //Guard time and prescaler register
} USART_RegDef_t;

#define USART1 ((USART_RegDef_t *)USART1_BASE)
#define USART2 ((USART_RegDef_t *)USART2_BASE)
#define USART3 ((USART_RegDef_t *)USART3_BASE)
#define UART4 ((USART_RegDef_t *)UART4_BASE)
#define UART5 ((USART_RegDef_t *)USART5_BASE)
#define USART6 ((USART_RegDef_t *)USART6_BASE)
#define UART7 ((USART_RegDef_t *)UART7_BASE)
#define UART8 ((USART_RegDef_t *)UART8_BASE)

// /*
//  * Bit field definitions of I2C peripherals
//  */

// //Status register (USART_SR)
// #define USART_SR_PE		0	//Parity error
// #define USART_SR_FE		1	//Framing error
// #define USART_SR_NF		2	//Noise detected flag
// #define USART_SR_ORE	3	//Overrun error
// #define USART_SR_IDLE	4	//IDLE line detected
// #define	USART_SR_RXNE	5	//Read data register not empty
// #define USART_SR_TC		6	//Transmission complete
// #define USART_SR_TXE	7	//Transmit data register empty
// #define USART_SR_LBD	8	//LIN break detection flag
// #define USART_SR_CTS	9	//CTS flag

// //Data register (USART_DR)
// #define USART_DR_DR		0	//Data value

// //Baud rate register (USART_BRR)
// #define USART_BRR_DIV_FRACTION30	0	//Fraction of USARTDIV
// #define USART_BRR_DIV_MANTISSA110	4	//Mantissa of USARTDIV

// //Control register 1 (USART_CR1)
// #define USART_CR1_SBK		0	//Send break
// #define USART_CR1_RWU		1	//Receiver wakeup
// #define USART_CR1_RE		2	//Receiver enable
// #define USART_CR1_TE		3	//Transmitter enable
// #define USART_CR1_IDLEIE	4	//IDLE interrupt enable
// #define USART_CR1_RXNEIE	5	//RXNE interrupt enable
// #define USART_CR1_TCIE		6	//Transmission complete interrupt enable
// #define	USART_CR1_TXEIE		7	//TXE interrupt enable
// #define USART_CR1_PEIE		8	//PE interrupt enable
// #define USART_CR1_PS		9	//Parity selection
// #define USART_CR1_PCE		10	//Parity control enable
// #define USART_CR1_WAKE		11	//Wakeup method
// #define USART_CR1_M			12	//Word length
// #define USART_CR1_UE		13	//USART enable
// #define USART_CR1_OVER8		15	//Oversampling mode

// //Control register 2 (USART_CR2)
// #define USART_CR2_ADD		0	//Address of the USART node
// #define USART_CR2_LBDL 		5	//Lin break detection length
// #define USART_CR2_LBDIE		6	//LIN break detection interrupt enable
// #define USART_CR2_LBCL		8	//Last bit clock pulse
// #define USART_CR2_CPHA		9	//Clock phase
// #define USART_CR2_CPOL		10	//Clock polarity
// #define USART_CR2_CLKEN 	11	//Clock enable
// #define USART_CR2_STOP		12	//STOP bits
// #define USART_CR2_LINEN		14	//LIN mode enable

// //Control register 3 (USART_CR3)
// #define USART_CR3_EIE		0	//Error interrupt enable
// #define USART_CR3_IREN 		1	//IrDA mode enable
// #define USART_CR3_IRLP		2	//IrDA low-power
// #define USART_CR3_HDSEL		3	//Half-duplex selection
// #define USART_CR3_NACK		4	//Smartcard NACK enable
// #define USART_CR3_SCEN		5	//Smartcard mode enable
// #define USART_CR3_DMAR		6	//DMA enable receiver
// #define USART_CR3_DMAT		7	//DMA enable transmitter
// #define USART_CR3_RTS		8	//RTS enable
// #define USART_CR3_CTSE		9	//CTS enable
// #define USART_CR3_CTSIE		10	//CTS interrupt enable
// #define USART_CR3_ONEBIT	11	//One sample bit method enable

// //Guard time and prescaler register (USART_GTPR)
// #define USART_GTPR_PSC		0	//Prescaler value
// #define USART_GTPR_GT		8	//Guard time value

#endif /* INC_STM32F407XX_H_ */
