/**
 * @file STM32F4xx_rcc.h
 * 
 * @brief Driver for the RCC peripheral on STM32F4xx MCUs, 
 * contains functions and macros to enable/disable peripherals
 * 
 * @author Miles Osborne
 */

#ifndef STM32F4xx_RCC_H
#define STM32F4xx_RCC_H

#include "STM32F4xx_base.h"

/*
 * RCC configuration 
 */

/* RCC register definition */
typedef struct
{
    volatile uint32_t CR;           // RCC clock control register										Address offset: 0x00
    volatile uint32_t PLLCFGR;      // RCC PLL configuration register									Address offset: 0x04
    volatile uint32_t CFGR;         // RCC clock configuration register									Address offset: 0x08
    volatile uint32_t CIR;          // RCC clock interrupt register										Address offset: 0x0C
    volatile uint32_t AHB1STR;      // RCC AHB1 peripheral reset register								Address offset: 0x10
    volatile uint32_t AHB2RSTR;     // RCC AHB2 peripheral reset register								Address offset: 0x14
    volatile uint32_t AHB3RSTR;     // RCC AHB3 peripheral reset register								Address offset: 0x18
    volatile uint32_t RESERVED0;    // Reserved
    volatile uint32_t APB1RSTR;     // RCC APB1 peripheral reset register								Address offset: 0x20
    volatile uint32_t APB2RSTR;     // RCC APB2 peripheral reset register								Address offset: 0x24
    volatile uint32_t RESERVED1[2]; // Reserved
    volatile uint32_t AHB1ENR;      // RCC AHB1 peripheral clock enable register						Address offset: 0x30
    volatile uint32_t AHB2ENR;      // RCC AHB2 peripheral clock enable register						Address offset: 0x34
    volatile uint32_t AHB3ENR;      // RCC APB1 peripheral clock enable register						Address offset: 0x38
    volatile uint32_t RESERVED2;    // Reserved
    volatile uint32_t APB1ENR;      // RCC APB1 peripheral clock enable register						Address offset: 0x40
    volatile uint32_t APB2ENR;      // RCC APB2 peripheral clock enable register						Address offset: 0x44
    volatile uint32_t RESERVED3[2]; // Reserved
    volatile uint32_t AHB1LPENR;    // RCC AHB1 peripheral clock enable in low power mode register		Address offset: 0x50
    volatile uint32_t AHB2LPENR;    // RCC AHB2 peripheral clock enable in low power mode register		Address offset: 0x54
    volatile uint32_t AHB3LPENR;    // RCC AHB3 peripheral clock enable in low power mode register		Address offset: 0x58
    volatile uint32_t RESERVED4;    // Reserved
    volatile uint32_t APB1LPENR;    // RCC APB1 peripheral clock enable in low power mode register		Address offset: 0x60
    volatile uint32_t APB2LPENR;    // RCC APB2 peripheral clock enabled in low power mode register		Address offset: 0x64
    volatile uint32_t RESERVED5[2]; // Reserved
    volatile uint32_t BDCR;         // RCC Backup domain control register								Address offset: 0x70
    volatile uint32_t CSR;          // RCC clock control & status register								Address offset: 0x74
    volatile uint32_t RESERVED6[2]; // Reserved
    volatile uint32_t SSCGR;        // RCC spread spectrum clock generation register					Address offset: 0x80
    volatile uint32_t PLLI2SCFGR;   // RCC PLLI2S configuration register								Address offset: 0x84
} RCC_RegDef_t;

/* RCC peripheral definition */
#define RCC ((RCC_RegDef_t *)RCC_BASE)

/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15))
//#define SPI4_PCLK_EN() (RCC->APB1ENR |= (1 << ))
//#define SPI5_PCLK_EN() (RCC->APB2ENR |= ())
//#define SPI6_PCLK_EN() (RCC->APB2ENR |= ())

/*
 * Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI() (RCC->APB2ENR |= ~(1 << 12))
#define SPI2_PCLK_DI() (RCC->APB1ENR |= ~(1 << 14))
#define SPI3_PCLK_DI() (RCC->APB1ENR |= ~(1 << 15))

/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23))

/*
 * Clock Disable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI() (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 23))


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
 * Clock Disable Macros for USARTx peripherals
 */
#define USART1_PCLK_DI() (RCC->APB2ENR |= ~(1 << 4))
#define USART2_PCLK_DI() (RCC->APB1ENR |= ~(1 << 17))
#define USART3_PCLK_DI() (RCC->APB1ENR |= ~(1 << 18))
#define UART4_PCLK_DI() (RCC->APB1ENR |= ~(1 << 19))
#define UART5_PCLK_DI() (RCC->APB1ENR |= ~(1 << 20))
#define USART6_PCLK_DI() (RCC->APB2ENR |= ~(1 << 5))

/*
 * Clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))

#endif

/* End of file */