/** @file STM32F4xx_SPI.h
 * 
 * @brief SPI drivers for STM32F4 MCUs
 *
 * @par
 * 
 *  Created on: Mar 25, 2021
 *      Author: milesosborne
 */

#ifndef INC_STM32F4XX_SPI_DRIVER_H_
#define INC_STM32F4XX_SPI_DRIVER_H_

#include "STM32F4xx_base.h"
#include "STM32F4xx_RCC.h"

/*
 *  Configuration structure for SPIx peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode; //Master or Slave modes
	uint8_t SPI_BusConfig;	//Full duplex, Half duplex, Simplex configuration
	uint8_t SPI_DFF;		//Data frame format
	uint8_t SPI_CPHA;		//Clock phase
	uint8_t SPI_CPOL;		//Clock polarity
	uint8_t SPI_SSM;		//Software slave management
	uint8_t SPI_Speed;		//Sets baud rate min prescaler
} SPI_Config_t;

/*
 * SPI Registers
 */
typedef struct
{
	volatile uint32_t SPI_CR1;	   // SPI control register 1
	volatile uint32_t SPI_CR2;	   // SPI control register 2
	volatile uint32_t SPI_SR;	   // SPI status register
	volatile uint32_t SPI_DR;	   // SPI data register
	volatile uint32_t SPI_CRCPR;   // SPI CRC polynomial register
	volatile uint32_t SPI_RXCRCR;  // SPI RX CRC register
	volatile uint32_t SPI_TXCRCR;  // SPI TX CRC register
	volatile uint32_t SPI_I2SCFGR; // SPI_I2S configuration register
	volatile uint32_t SPI_I2SPR;   // SPI_I2S prescaler register
} SPI_RegDef_t;

/*
 * User handle structure for SPIx peripheral
 */
typedef struct
{
	SPI_RegDef_t *pSPIx; //Base address of SPIx peripheral
	SPI_Config_t SPIConfig;

	//SPI Interrupt method parameters
	uint8_t *pTxBuffer; //To store the Tx buffer address
	uint8_t *pRxBuffer; //To store the Rx buffer address
	uint32_t TxLen;		//To store the Tx data length
	uint32_t RxLen;		//To store the Rx data length
	uint8_t TxState;	//To store the Tx state
	uint8_t RxState;	//To store the Rx state
} SPI_Handle_t;

/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER 1
#define SPI_DEVICE_MODE_SLAVE 0

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD 1				//Full duplex
#define SPI_BUS_CONFIG_HD 2				//Half duplex
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY 3 //Simplex (recieve only)

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS 0	 //Sets data frame format to 8bits (default)
#define SPI_DFF_16BITS 1 //Sets data frame format to 16bits

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_LOW 0	//the leading/first clock transition is the data capture edge
#define SPI_CPHA_HIGH 1 //The trailing/trailing clock transition is the data capture edge

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_LOW 0	//Sets clock idle state to LOW
#define SPI_CPOL_HIGH 1 //Sets clock idle state to HIGH

/*
 * @SPI_SSM
 */
#define SPI_SSM_DI 0 //SPI Software slave management disabled, Hardware management enabled
#define SPI_SSM_EN 1 //SPI Software slave management enable

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2 0	//baud rate = fPCLK/2
#define SPI_SCLK_SPEED_DIV4 1	//baud rate = fPCLK/4
#define SPI_SCLK_SPEED_DIV8 2	//baud rate = fPCLK/8
#define SPI_SCLK_SPEED_DIV16 3	//baud rate = fPCLK/16
#define SPI_SCLK_SPEED_DIV32 4	//baud rate = fPCLK/32
#define SPI_SCLK_SPEED_DIV64 5	//baud rate = fPCLK/64
#define SPI_SCLK_SPEED_DIV128 6 //baud rate = fPCLK/128
#define SPI_SCLK_SPEED_DIV256 7 //baud rate = fPCLK/256

/*
 * Possible SPI Application States
 */
#define SPI_READY 0
#define SPI_BUSY_IN_RX 1
#define SPI_BUSY_IN_TX 2

/*
 * Possible SPI Application Events
 */
#define SPI_EVENT_TX_COMPLETE 1
#define SPI_EVENT_RX_COMPLETE 2
#define SPI_EVENT_OVR_ERR 3
#define SPI_EVENT_CRC_ERR 4

/*
 * SPI related status flags definitions
 * Contains bit masking information for each flag
 * in the status register (SPI_SR)
 */
#define SPI_RXNE_FLAG (1 << SPI_SR_RXNE)
#define SPI_TXE_FLAG (1 << SPI_SR_TXE)
#define SPI_UDR_FLAG (1 << SPI_SR_UDR)
#define SPI_CRC_FLAG (1 << SPI_SR_CRC_ERR)
#define SPI_MODF_FLAG (1 << SPI_SR_MODF)
#define SPI_OVR_FLAG (1 << SPI_SR_OVR)
#define SPI_BSY_FLAG (1 << SPI_SR_BSY)

/*
 * Bit field definitions of SPI peripherals
 */
//SPI control register 1 (SPI_CR1)
#define SPI_CR1_CPHA 		0	//Clock phase
#define SPI_CR1_CPOL 		1	//Clock polarity
#define SPI_CR1_MSTR 		2	//Master selection
#define SPI_CR1_BR			3	//Baud rate control
#define SPI_CR1_SPE			6	//SPI enable
#define SPI_CR1_LSB_FIRST	7	//Frame format
#define SPI_CR1_SSI			8	//Internal slave select
#define SPI_CR1_SSM			9	//Software slave management
#define SPI_CR1_RXONLY		10	//Receive only
#define	SPI_CR1_DFF			11	//Data frame format
#define	SPI_CRC_NEXT		12	//CRC transfer next
#define	SPI_CR1_CRC_EN		13	//Hardware CRC calculation enable
#define SPI_CR1_BIDI_OE		14	//Output enable in bidirectional mode
#define SPI_CR1_BIDI_MODE	15	//Biderectional data mode enable

//SPI control register 2 (SPI_CR2)
#define SPI_CR2_RXDMAEN		0	//Rx buffed DMA enable
#define SPI_CR2_TXDMAEN		1	//Tx buffer DMA enable
#define SPI_CR2_SSOE		2	//SS outpute enable
#define SPI_CR2_FRF			4	//Frame formate
#define SPI_CR2_ERRIE		5	//Error interrupt enable
#define SPI_CR2_RXNEIE		6	//RX bufer not empty interrupt enable
#define SPI_CR2_TXEIE		7	//Tx buffer empty interrupt enable

//SPI status register (SPI_SR)
#define SPI_SR_RXNE			0	//Receive buffer not empty
#define SPI_SR_TXE			1	//Transmit buffer empty
#define SPI_SR_CHSIDE		2	//Channel side
#define SPI_SR_UDR			3	//Underrun flag
#define SPI_SR_CRC_ERR		4	//CRC error flag
#define SPI_SR_MODF			5	//Mode fault
#define SPI_SR_OVR			6	//Overrun flag
#define SPI_SR_BSY			7	//Busy flag
#define SPI_SR_FRE			8	//Frame format error

#define SPI1 ((SPI_RegDef_t *)SPI1_BASE)
#define SPI2 ((SPI_RegDef_t *)SPI2_BASE)
#define SPI3 ((SPI_RegDef_t *)SPI3_BASE)
#define SPI4 ((SPI_RegDef_t *)SPI4_BASE)

/******************************APIs supported by this driver****************************/

/*
 * Peripheral clock control
 */
void SPI_peripheral_clock_enable(SPI_RegDef_t *pSPIx);
void SPI_peripheral_clock_disable(SPI_RegDef_t *pSPIx);

/*
 * Initialization / Deinitialization
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data send & receive
 */

//blocking based api
void SPI_TransmitData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);

//Interrupt based api
uint8_t SPI_TransmitDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len);
uint8_t SPI_RecieveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len);

/*
 * IRQ Configuration and handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNUmber, uint8_t Enable_Disable);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 * Other peripheral control APIs
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t Enable_Disable);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t Enable_Disable);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * SPI_ApplicationEventCallback
 */
void SPI_ApplicationCallback(SPI_Handle_t *pSPIHandle, uint8_t handleEvent);

#endif /* INC_STM32F4XX_SPI_DRIVER_H_ */
