/*
 * stm32f407xx_USART_driver.h
 *
 *  Created on: Jun 2, 2021
 *      Author: milesosborne
 */

#include "stm32f407xx_base_driver.h"
#ifndef INC_STM32F407XX_USART_DRIVER_H_
#define INC_STM32F407XX_USART_DRIVER_H_

/*
 * USART configuration structure
 */
typedef struct
{
	uint8_t USART_Mode;
	uint32_t USART_Baud;
	uint8_t USART_NoOfStopBits;
	uint8_t USART_WordLength;
	uint8_t USART_ParityControl;
	uint8_t USART_HWFlowControl;
} USART_Config_t;

/*
 * USART handle structure
 */
typedef struct
{
	USART_RegDef_t *pUSARTx;
	USART_Config_t USART_Config;
} USART_Handle_t;

/*
 *@USART_Mode
 *Possible options for USART_Mode
 */
#define USART_MODE_ONLY_TX 0
#define USART_MODE_ONLY_RX 1
#define USART_MODE_TXRX 2

/*
 *@USART_Baud
 *Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200 1200
#define USART_STD_BAUD_2400 2400
#define USART_STD_BAUD_9600 9600
#define USART_STD_BAUD_19200 19200
#define USART_STD_BAUD_38400 38400
#define USART_STD_BAUD_57600 57600
#define USART_STD_BAUD_115200 115200
#define USART_STD_BAUD_230400 230400
#define USART_STD_BAUD_460800 460800
#define USART_STD_BAUD_921600 921600
#define USART_STD_BAUD_2M 2000000
#define SUART_STD_BAUD_3M 3000000

/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD 2
#define USART_PARITY_EN_EVEN 1
#define USART_PARITY_DISABLE 0

/*
 *@USART_WordLength
 *Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS 0
#define USART_WORDLEN_9BITS 1

/*
 *@USART_NoOfStopBits
 *Possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1 0
#define USART_STOPBITS_0_5 1
#define USART_STOPBITS_2 2
#define USART_STOPBITS_1_5 3

/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE 0
#define USART_HW_FLOW_CTRL_CTS 1
#define USART_HW_FLOW_CTRL_RTS 2
#define USART_HW_FLOW_CTRL_CTS_RTS 3

/*
 * USART related flags definition
 */
#define USART_FLAG_NF (1 << USART_SR_NF)
#define USART_FLAG_RXNE (1 << USART_SR_RXNE)
#define USART_FLAG_TC (1 << USART_SR_TC)
#define USART_FLAG_TXE (1 << USART_SR_TXE)
#define USART_FLAG_LBD (1 << USART_SR_LBD)
#define USART_FLAG_CTS (1 << USART_SR_CTS)

/*
 * Bit field definitions of I2C peripherals
 */

//Status register (USART_SR)
#define USART_SR_PE 0	//Parity error
#define USART_SR_FE 1	//Framing error
#define USART_SR_NF 2	//Noise detected flag
#define USART_SR_ORE 3	//Overrun error
#define USART_SR_IDLE 4 //IDLE line detected
#define USART_SR_RXNE 5 //Read data register not empty
#define USART_SR_TC 6	//Transmission complete
#define USART_SR_TXE 7	//Transmit data register empty
#define USART_SR_LBD 8	//LIN break detection flag
#define USART_SR_CTS 9	//CTS flag

//Data register (USART_DR)
#define USART_DR_DR 0 //Data value

//Baud rate register (USART_BRR)
#define USART_BRR_DIV_FRACTION30 0	//Fraction of USARTDIV
#define USART_BRR_DIV_MANTISSA110 4 //Mantissa of USARTDIV

//Control register 1 (USART_CR1)
#define USART_CR1_SBK 0	   //Send break
#define USART_CR1_RWU 1	   //Receiver wakeup
#define USART_CR1_RE 2	   //Receiver enable
#define USART_CR1_TE 3	   //Transmitter enable
#define USART_CR1_IDLEIE 4 //IDLE interrupt enable
#define USART_CR1_RXNEIE 5 //RXNE interrupt enable
#define USART_CR1_TCIE 6   //Transmission complete interrupt enable
#define USART_CR1_TXEIE 7  //TXE interrupt enable
#define USART_CR1_PEIE 8   //PE interrupt enable
#define USART_CR1_PS 9	   //Parity selection
#define USART_CR1_PCE 10   //Parity control enable
#define USART_CR1_WAKE 11  //Wakeup method
#define USART_CR1_M 12	   //Word length
#define USART_CR1_UE 13	   //USART enable
#define USART_CR1_OVER8 15 //Oversampling mode

//Control register 2 (USART_CR2)
#define USART_CR2_ADD 0	   //Address of the USART node
#define USART_CR2_LBDL 5   //Lin break detection length
#define USART_CR2_LBDIE 6  //LIN break detection interrupt enable
#define USART_CR2_LBCL 8   //Last bit clock pulse
#define USART_CR2_CPHA 9   //Clock phase
#define USART_CR2_CPOL 10  //Clock polarity
#define USART_CR2_CLKEN 11 //Clock enable
#define USART_CR2_STOP 12  //STOP bits
#define USART_CR2_LINEN 14 //LIN mode enable

//Control register 3 (USART_CR3)
#define USART_CR3_EIE 0		//Error interrupt enable
#define USART_CR3_IREN 1	//IrDA mode enable
#define USART_CR3_IRLP 2	//IrDA low-power
#define USART_CR3_HDSEL 3	//Half-duplex selection
#define USART_CR3_NACK 4	//Smartcard NACK enable
#define USART_CR3_SCEN 5	//Smartcard mode enable
#define USART_CR3_DMAR 6	//DMA enable receiver
#define USART_CR3_DMAT 7	//DMA enable transmitter
#define USART_CR3_RTS 8		//RTS enable
#define USART_CR3_CTSE 9	//CTS enable
#define USART_CR3_CTSIE 10	//CTS interrupt enable
#define USART_CR3_ONEBIT 11 //One sample bit method enable

//Guard time and prescaler register (USART_GTPR)
#define USART_GTPR_PSC 0 //Prescaler value
#define USART_GTPR_GT 8	 //Guard time value

/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
/*
 * Peripheral Clock setup
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);

/*
 * Data Send and Receive
 */
void USART_SendData(USART_RegDef_t *pUSARTx, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_RegDef_t *pUSARTx, uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pHandle);

/*
 * Other Peripheral Control APIs
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);

/*
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEv);

#endif /* INC_STM32F407XX_USART_DRIVER_H_ */
